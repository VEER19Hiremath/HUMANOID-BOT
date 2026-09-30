import collections
import fcntl
import json
import math
import os
import queue
import re
import shutil
import socket
import struct
import subprocess
import tempfile
import termios
import threading
import time

import numpy as np

try:
    from gtts import gTTS
except ImportError:  # pragma: no cover - optional runtime dependency
    gTTS = None

try:
    from vosk import Model, KaldiRecognizer
except ImportError:  # pragma: no cover - optional runtime dependency
    Model = None
    KaldiRecognizer = None

try:
    import sounddevice as sd
except (ImportError, OSError):  # pragma: no cover - optional runtime dependency
    sd = None

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String

from tf2_ros import Buffer, TransformListener

from hospital_delivery.room_config import (
    ROOM_ALIASES, ROOM_COORDS, goal_yaw, rooms_from_map)


# 16 kHz mono int16 = 32000 bytes/s; more than this unread = over 1 s behind.
MAX_AUDIO_LAG_BYTES = 32000
PREROLL_BYTES = 16000     # 0.5 s of 16 kHz audio before speech starts
HANGOVER_S = 0.8          # keep listening this long after the voice drops
HFP_SOCK = "/tmp/hospital_hfp_mic.sock"
# Exists only while scripts/hfp_mic.py has headset audio flowing.
HFP_LINK_FLAG = "/tmp/hospital_hfp_mic.up"


def headset_linked():
    """True while the headset bridge (its pid is in the flag) streams audio."""
    try:
        with open(HFP_LINK_FLAG) as f:
            os.kill(int(f.read().strip()), 0)   # a stale flag from a killed bridge
    except (OSError, ValueError):
        return False
    return os.path.exists(HFP_SOCK)


class VoiceDeliveryNode(Node):

    def __init__(self):
        super().__init__("voice_delivery_node")

        model_path = os.path.expanduser("~/models/vosk-model-small-en-us-0.15")
        self.model = None

        if Model is None or KaldiRecognizer is None:
            self.get_logger().warn(
                "Vosk is not installed; voice recognition is disabled."
            )
        else:
            self.get_logger().info("Loading Vosk model...")
            if os.path.isdir(model_path):
                try:
                    self.model = Model(model_path)
                except Exception as exc:
                    self.get_logger().warn(
                        f"Failed to load Vosk model at {model_path}: {exc}"
                    )
            else:
                self.get_logger().warn(
                    f"Vosk model directory not found: {model_path}; "
                    "voice recognition disabled."
                )

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose",
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        # Cancel ALL navigate_to_pose goals (an empty request = every goal).
        # Cancelling through the goal handle failed whenever the "accepted"
        # reply was lost on DDS: no handle, so "stop" left Nav2 driving.
        self.cancel_all_client = self.create_client(
            CancelGoal, "navigate_to_pose/_action/cancel_goal")
        # Typed commands go through the same path as speech, e.g.
        #   ros2 topic pub --once /voice_command std_msgs/String "data: go to room one"
        self.create_subscription(
            String, "voice_command", lambda msg: self.command_queue.put(msg.data), 10)
        self.active_goal = None
        self._goal_lock = threading.Lock()
        self._pending_room = None
        self._nav_busy = False

        self.command_queue = queue.Queue()
        self._listen_thread = None
        self._listening = False
        self._capture_source_id = None
        self._mute_mic = False
        self._dropped_audio_s = 0.0
        self._shutdown = False
        self._partial_text = ""
        self._quiet_since = None
        self._last_live_cmd = ""
        self._last_live_at = 0.0

        self._noise_floor = 280.0
        self._preroll = collections.deque()
        self._in_speech = False
        self._last_speech = 0.0
        self._stable_partial = ""
        self._stable_since = 0.0

        # Stop when the mic is flat this long during a goal (0 = off). Off by
        # default: in a quiet room a live headset reads rms 1-3 / peak 3-5,
        # the same as a dead link, so it stopped every trip after 8 s quiet.
        self.declare_parameter("mic_watchdog_s", 0.0)
        # Lowest word confidence (0-1) a room command needs.
        self.declare_parameter("min_confidence", 0.4)
        self._mic_alive_rms = 6.0
        self._mic_alive_at = time.monotonic()
        self._mic_lost = False
        self.create_timer(1.0, self._mic_watchdog)

        # Rooms come from the map start.sh loaded ("# room:" lines), so each
        # map carries its own; room_config.ROOM_COORDS is the fallback.
        self.declare_parameter("map_yaml", "")
        map_yaml = self.get_parameter("map_yaml").value
        self.rooms = rooms_from_map(map_yaml) if map_yaml else None
        if self.rooms:
            self.get_logger().info(f"Rooms from {map_yaml}: {self.rooms}")
        else:
            self.rooms = ROOM_COORDS
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.status_names = {
            GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
            GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
            GoalStatus.STATUS_EXECUTING: "EXECUTING",
            GoalStatus.STATUS_CANCELING: "CANCELING",
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
        }

        self.get_logger().info(
            "Voice control ready; mic stays open for stop / redirect while driving."
        )

        self.speak("Hospital robot ready")

    def speak(self, text):
        """Announce without blocking the command loop or the microphone."""
        threading.Thread(
            target=self._speak_now, args=(text,), daemon=True
        ).start()

    def _speak_now(self, text):
        """Speak while muting only during playback, so a new command still queues."""
        self.get_logger().info(text)
        print(f"\n[TTS] {text}\n")

        try:
            # Prefer Google TTS (natural voice); fall back to espeak.
            played = False
            if gTTS is not None:
                mp3_path = None
                try:
                    with tempfile.NamedTemporaryFile(
                        suffix=".mp3", delete=False
                    ) as tmp:
                        mp3_path = tmp.name
                    gTTS(text=text, lang="en").save(mp3_path)
                    player = None
                    if shutil.which("ffplay"):
                        player = [
                            "ffplay", "-nodisp", "-autoexit",
                            "-loglevel", "quiet", mp3_path,
                        ]
                    elif shutil.which("mpg123"):
                        player = ["mpg123", "-q", mp3_path]
                    if player:
                        self._mute_mic = True
                        subprocess.run(
                            player,
                            check=False,
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL,
                        )
                        played = True
                except Exception as exc:
                    self.get_logger().warning(
                        f"Google TTS failed ({exc}); trying espeak"
                    )
                finally:
                    if mp3_path and os.path.exists(mp3_path):
                        try:
                            os.remove(mp3_path)
                        except OSError:
                            pass

            if not played and shutil.which("espeak") is not None:
                try:
                    self._mute_mic = True
                    subprocess.run(
                        ["espeak", text],
                        check=False,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        shell=False,
                    )
                except OSError as exc:
                    self.get_logger().warning(f"Text-to-speech failed: {exc}")
        finally:
            time.sleep(1.0)
            self._mute_mic = False

    def _chunk_rms(self, data):
        # numpy: the per-sample Python loop cost a big share of a core and
        # helped push recognition ~20 s behind while driving.
        samples = np.frombuffer(data[:len(data) - len(data) % 2], dtype="<i2")
        if samples.size == 0:
            return 0.0
        return float(np.sqrt(np.mean(samples.astype(np.float32) ** 2)))

    def _pipewire_capture_cmd(self):
        if shutil.which("arecord"):
            return [
                "arecord", "-D", "pipewire", "-f", "S16_LE",
                "-r", "16000", "-c", "1", "-t", "raw", "-",
            ]
        return None

    def _wpctl_status(self):
        if not shutil.which("wpctl"):
            return ""
        try:
            proc = subprocess.run(
                ["wpctl", "status"],
                capture_output=True,
                text=True,
                timeout=3,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            return ""
        return proc.stdout or ""

    def _section_lines(self, status, start, stops):
        lines = []
        collecting = False
        for line in status.splitlines():
            if start in line:
                collecting = True
                continue
            if collecting and any(stop in line for stop in stops):
                break
            if collecting:
                lines.append(line)
        return lines

    def _first_id(self, line):
        match = re.search(r"(\d+)\.", line)
        return match.group(1) if match else None

    def _enable_headset_profile(self, status):
        device_id = None
        for line in self._section_lines(status, "Devices:", ("Sinks:", "Sources:")):
            if "bluez" not in line:
                continue
            device_id = self._first_id(line)
            if device_id:
                break
        if not device_id or not shutil.which("pw-cli"):
            return
        try:
            enum = subprocess.run(
                ["pw-cli", "enum-params", device_id, "EnumProfile"],
                capture_output=True,
                text=True,
                timeout=3,
                check=False,
            ).stdout or ""
        except (OSError, subprocess.TimeoutExpired):
            return
        index = None
        expect = None
        for raw in enum.splitlines():
            line = raw.strip()
            if "Profile:index" in line:
                expect = "index"
                continue
            if "Profile:name" in line:
                expect = "name"
                continue
            if expect == "index" and line.startswith("Int"):
                parts = line.split()
                if len(parts) >= 2:
                    index = parts[1]
                expect = None
            elif expect == "name" and "headset-head-unit" in line and index:
                subprocess.run(
                    ["wpctl", "set-profile", device_id, index],
                    capture_output=True,
                    timeout=3,
                    check=False,
                )
                self.get_logger().info(
                    f"Headset microphone profile set on Bluetooth device {device_id}"
                )
                return
            elif line.startswith("Prop:"):
                expect = None

    def _is_headset_mic(self, source_id):
        """True only for a Bluetooth headset capture node, not a speaker or monitor."""
        try:
            info = subprocess.run(
                ["wpctl", "inspect", source_id],
                capture_output=True,
                text=True,
                timeout=3,
                check=False,
            ).stdout or ""
        except (OSError, subprocess.TimeoutExpired):
            return False
        if "bluez_input" in info:
            return True
        return "headset-head-unit" in info and "bluez_output" not in info

    def _headset_source_id(self, status):
        for line in self._section_lines(
            status, "Sources:", ("Source endpoints:", "Streams:", "Video")
        ):
            if "monitor" in line.lower():
                continue
            source_id = self._first_id(line)
            if source_id and self._is_headset_mic(source_id):
                return source_id
        return None

    def _mic_sources(self, status):
        """Real capture nodes. Monitors of speakers are not microphones."""
        found = []
        for line in self._section_lines(
            status, "Sources:", ("Source endpoints:", "Streams:", "Video")
        ):
            if "monitor" in line.lower():
                continue
            source_id = self._first_id(line)
            if not source_id:
                continue
            found.append((source_id, "*" in line))
        return found

    def _prepare_capture_source(self):
        """Use whichever microphone is the system default.

        arecord -D pipewire follows that default, so a USB mic, a headset,
        or any other input works without naming the device. A Bluetooth
        device left in music mode has no mic; its headset profile is turned
        on, and the default is changed only when nothing else is selected.
        """
        status = self._wpctl_status()
        if not status:
            return None
        if "bluez" in status and self._headset_source_id(status) is None:
            self._enable_headset_profile(status)
            time.sleep(0.4)
            status = self._wpctl_status()
        sources = self._mic_sources(status)
        if not sources:
            return None
        default_id = next((sid for sid, is_default in sources if is_default), None)
        if default_id:
            return default_id
        source_id = sources[0][0]
        try:
            subprocess.run(
                ["wpctl", "set-default", source_id],
                capture_output=True,
                timeout=3,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            return None
        return source_id

    def _new_recognizer(self, sample_rate):
        """Limit recognition to room commands so headset audio is not free-dictated."""
        recognizer = KaldiRecognizer(self.model, sample_rate)
        phrases = ["[unk]", "stop", "go home", "reception"]
        # Only the rooms of the loaded map: fewer words, fewer mix-ups.
        for room, aliases in ROOM_ALIASES.items():
            if room not in self.rooms:
                continue
            # Vosk's vocabulary has no digits ("room 1", "room1"): it drops the
            # unknown word, leaving a bare "room". Spoken forms cover them.
            phrases.extend(
                a for a in aliases
                if not any(ch.isdigit() for ch in a) and a not in phrases
            )
        recognizer.SetGrammar(json.dumps(phrases))
        recognizer.SetWords(True)   # per-word confidence in final results
        return recognizer

    def _start_continuous_listen(self):
        if self.model is None or KaldiRecognizer is None:
            self.get_logger().warn(
                "Voice recognition unavailable (missing Vosk model)."
            )
            return
        if sd is None and self._pipewire_capture_cmd() is None:
            self.get_logger().warn(
                "Voice recognition unavailable (missing microphone capture)."
            )
            return
        if self._listen_thread is not None and self._listen_thread.is_alive():
            return
        self._listening = True
        self._listen_thread = threading.Thread(
            target=self._listen_loop,
            name="voice_mic",
            daemon=True,
        )
        self._listen_thread.start()
        self.get_logger().info("Continuous microphone listening started")

    def _is_speech(self, rms):
        """True when this chunk is above the headset/room floor.

        Headset HFP levels are often rms 20–80; the old floor of 140
        dropped almost every phrase so room commands never fired.
        """
        if rms < self._noise_floor * 1.25:
            self._noise_floor = (0.97 * self._noise_floor) + (0.03 * rms)
        return rms > max(22.0, self._noise_floor * 1.35)

    def _accept_phrase(self, text):
        text = (text or "").lower().strip()
        if not text:
            return False
        if text in ("stop", "stop robot"):
            return True
        return self.extract_room(text) is not None

    @staticmethod
    def _is_stop(text):
        return text in ("stop", "stop robot")

    @staticmethod
    def _confidence(result):
        """Lowest word confidence of a final Vosk result, or None."""
        words = result.get("result") or []
        confs = [w.get("conf", 0.0) for w in words if w.get("word") != "[unk]"]
        return min(confs) if confs else None

    def _queue_command(self, text, conf=None, final=True):
        """Queue one command. The same text is ignored for 3 s.

        "stop" is taken from anything, even a partial guess (a false stop is
        harmless). A room command needs a finished phrase whose every word
        the recognizer is sure of: partial guesses and unsure words sent the
        robot off on background talk.
        """
        text = (text or "").lower().strip()
        if not self._accept_phrase(text):
            return False
        if not self._is_stop(text):
            min_conf = float(self.get_parameter("min_confidence").value)
            if not final:
                return False
            if conf is not None and conf < min_conf:
                self.get_logger().warning(
                    f"Ignoring unsure phrase '{text}' (confidence {conf:.2f} < {min_conf})")
                return False
        now = time.monotonic()
        if text == self._last_live_cmd and now - self._last_live_at < 3.0:
            return False
        self._last_live_cmd = text
        self._last_live_at = now
        self._partial_text = ""
        self._quiet_since = None
        self._stable_partial = ""
        self.get_logger().info(f"Detected: {text}")
        self.command_queue.put(text)
        return True

    def _consider_live(self, heard, recognizer, sample_rate):
        """Submit a partial only after it holds still.

        A one-word hit such as "home" must hold longer. Room noise was
        matching that word and cancelling the trip about every 20 s.
        """
        text = (heard or "").lower().strip()
        if not self._accept_phrase(text):
            return recognizer
        if text != self._stable_partial:
            self._stable_partial = text
            self._stable_since = time.monotonic()
            return recognizer
        # Only "stop" acts on a partial guess (at once). Room commands wait
        # for the finished phrase and its confidence (_on_recognized_text).
        if self._is_stop(text) and self._queue_command(text, final=False):
            return self._new_recognizer(sample_rate)
        return recognizer

    def _on_recognized_text(self, recognizer):
        result = json.loads(recognizer.Result())
        text = (result.get("text") or "").lower().strip()
        self._partial_text = ""
        self._quiet_since = None
        self._stable_partial = ""
        self._queue_command(text, conf=self._confidence(result))

    def _finish_partial(self, recognizer, sample_rate):
        """Vosk ends a phrase only after quiet audio. The mic gate used to drop
        that quiet audio, so 'go to room one' stayed a partial and never drove."""
        text, conf, final = "", None, True
        try:
            result = json.loads(recognizer.FinalResult() or "{}")
            text = (result.get("text") or "").lower().strip()
            conf = self._confidence(result)
        except Exception:
            text = ""
        if not text:
            # No finished phrase: the last partial may still stop the robot,
            # but never send it anywhere.
            text, final = (self._partial_text or "").lower().strip(), False
        self._partial_text = ""
        self._quiet_since = None
        self._stable_partial = ""
        recognizer = self._new_recognizer(sample_rate)
        self._queue_command(text, conf=conf, final=final)
        return recognizer

    def _mic_watchdog(self):
        limit = float(self.get_parameter("mic_watchdog_s").value)
        if limit <= 0.0 or self.model is None:
            return
        silent_for = time.monotonic() - self._mic_alive_at
        if silent_for < limit:
            if self._mic_lost:
                self._mic_lost = False
                self.get_logger().info("Microphone back")
            return
        if self._mic_lost or not (self._nav_busy or self.active_goal is not None):
            return
        self._mic_lost = True
        self.get_logger().error(
            f"Microphone silent for {silent_for:.0f} s while driving; stopping "
            "(cannot hear stop)."
        )
        self.stop_robot(announce=False)
        self.speak("Microphone lost, stopping")

    def _consume_audio(self, recognizer, data, sample_rate):
        """Feed the recognizer whole phrases, gated by the mic level.

        Only audio above the room floor is speech, but a phrase's soft start
        and the quiet gaps between its words sit below it: with a noisy USB
        mic "go home" reached the recognizer as just "home" (floor test
        2026-09-30). So the last PREROLL_S of audio is fed when speech starts,
        and audio keeps flowing HANGOVER_S after it drops.
        """
        rms = self._chunk_rms(data)
        if rms > self._mic_alive_rms:
            self._mic_alive_at = time.monotonic()
        if self._mute_mic:
            self._partial_text = ""
            self._quiet_since = None
            self._stable_partial = ""
            self._preroll.clear()
            self._in_speech = False
            return self._new_recognizer(sample_rate)
        now = time.monotonic()
        if self._is_speech(rms):
            self._last_speech = now
            if not self._in_speech:
                self._in_speech = True
                for chunk in self._preroll:
                    recognizer = self._feed(recognizer, chunk, sample_rate)
                self._preroll.clear()
            return self._feed(recognizer, data, sample_rate)
        if self._in_speech and now - self._last_speech < HANGOVER_S:
            return self._feed(recognizer, data, sample_rate)
        if self._in_speech:
            self._in_speech = False
            if self._partial_text:
                recognizer = self._finish_partial(recognizer, sample_rate)
        self._preroll.append(data)
        while sum(len(c) for c in self._preroll) > PREROLL_BYTES:
            self._preroll.popleft()
        return recognizer

    def _feed(self, recognizer, data, sample_rate):
        if recognizer.AcceptWaveform(data):
            self._on_recognized_text(recognizer)
            return recognizer
        partial = json.loads(recognizer.PartialResult() or "{}")
        heard = (partial.get("partial") or "").strip()
        if heard and heard != self._partial_text:
            self._partial_text = heard
            self.get_logger().info(f"Hearing: {heard}")
        if heard:
            recognizer = self._consider_live(heard, recognizer, sample_rate)
        return recognizer

    def _listen_pipewire(self):
        sample_rate = 16000
        recognizer = self._new_recognizer(sample_rate)
        source_id = self._prepare_capture_source()
        self._capture_source_id = source_id
        proc = subprocess.Popen(
            self._pipewire_capture_cmd(),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        if not source_id:
            proc.terminate()
            raise RuntimeError("no system microphone")
        self.get_logger().info(f"Listening from system microphone {source_id}")
        chunks = 0
        loudest = 0.0
        try:
            while self._listening and rclpy.ok() and not self._shutdown:
                data = proc.stdout.read(4000)
                if not data:
                    err = proc.stderr.read().decode("utf-8", "replace").strip()
                    raise RuntimeError(err or "microphone capture stopped")
                if headset_linked():
                    self.get_logger().info("Headset connected: switching to it")
                    return
                rms = self._chunk_rms(data)
                loudest = max(loudest, rms)
                chunks += 1
                # 4000 bytes is 0.125 s. Report the mic level about every 2 s.
                if chunks % 16 == 0:
                    self.get_logger().info(
                        f"Mic level rms={rms:.0f} peak_rms={loudest:.0f}"
                    )
                    loudest = 0.0
                    if rms < 30.0:
                        fresh = self._prepare_capture_source()
                        if fresh and fresh != self._capture_source_id:
                            raise RuntimeError(
                                f"microphone {fresh} is ready"
                            )
                recognizer = self._consume_audio(recognizer, data, sample_rate)
        finally:
            proc.terminate()
            try:
                proc.wait(timeout=1)
            except subprocess.TimeoutExpired:
                proc.kill()

    @staticmethod
    def _socket_pending(sock):
        """Bytes waiting in the socket (unread audio)."""
        try:
            return struct.unpack("i", fcntl.ioctl(sock.fileno(), termios.FIONREAD, b"\0\0\0\0"))[0]
        except OSError:
            return 0

    def _listen_hfp(self):
        """Read the call-link PCM published by scripts/hfp_mic.py."""
        sample_rate = 16000
        recognizer = self._new_recognizer(sample_rate)
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.settimeout(2.0)
        sock.connect(HFP_SOCK)
        self.get_logger().info("Listening from the Bluetooth headset")
        loudest = 0.0
        last_report = time.monotonic()
        try:
            gone_since = None
            while self._listening and rclpy.ok() and not self._shutdown:
                # Gone for 2 s = disconnected (not a single missed check).
                if headset_linked():
                    gone_since = None
                elif gone_since is None:
                    gone_since = time.monotonic()
                elif time.monotonic() - gone_since > 2.0:
                    self.get_logger().warning("Headset disconnected")
                    return
                try:
                    data = sock.recv(4000)
                except socket.timeout:
                    continue
                if not data:
                    raise RuntimeError("headset call link closed")
                # Never fall more than ~1 s behind: on a loaded Pi the backlog
                # grew to ~20 s and "stop" was heard far too late (floor test
                # 2026-09-30). Drop the old audio and listen to what's live.
                pending = self._socket_pending(sock)
                if pending > MAX_AUDIO_LAG_BYTES:
                    dropped = 0
                    while self._socket_pending(sock) > 8000:
                        chunk = sock.recv(32000)
                        if not chunk:
                            break
                        dropped += len(chunk)
                    self._dropped_audio_s += dropped / 32000.0
                    recognizer = self._new_recognizer(sample_rate)
                    self._partial_text = ""
                    self._stable_partial = ""
                    self._quiet_since = None
                    continue
                rms = self._chunk_rms(data)
                loudest = max(loudest, rms)
                now = time.monotonic()
                if now - last_report >= 2.0:
                    self.get_logger().info(
                        f"Mic level rms={rms:.0f} peak_rms={loudest:.0f}"
                        + (f" (behind: dropped {self._dropped_audio_s:.1f} s)"
                           if self._dropped_audio_s else "")
                    )
                    self._dropped_audio_s = 0.0
                    loudest = 0.0
                    last_report = now
                recognizer = self._consume_audio(recognizer, data, sample_rate)
        finally:
            sock.close()

    def _listen_loop(self):
        """Listen on whichever microphone works, switching as devices come and go.

        A Bluetooth headset (scripts/hfp_mic.py, any paired hands-free
        headset) wins while its audio flows; otherwise the system default
        microphone (USB or wired) through PipeWire.
        """
        last_warning = 0.0
        while self._listening and not self._shutdown and rclpy.ok():
            try:
                if headset_linked():
                    self._listen_hfp()
                    continue
                if self._pipewire_capture_cmd() is not None:
                    self._listen_pipewire()
                    continue
            except Exception as exc:
                if time.monotonic() - last_warning > 30:
                    self.get_logger().warning(
                        f"No microphone yet ({exc}): connect a Bluetooth headset "
                        "or plug in a USB microphone"
                    )
                    last_warning = time.monotonic()
            time.sleep(1.0)

    def stop_robot(self, announce=True):
        stop_msg = Twist()
        with self._goal_lock:
            goal = self.active_goal
            self.active_goal = None
            self._pending_room = None
            self._nav_busy = False
        self.cancel_all_client.call_async(CancelGoal.Request())
        if goal is not None:
            try:
                goal.cancel_goal_async()
            except Exception as e:
                self.get_logger().warning(f"Could not cancel active goal: {e}")
        self.get_logger().info("Stop: all navigation goals cancelled")
        for _ in range(20):
            self.cmd_vel_pub.publish(stop_msg)
            time.sleep(0.05)
        if announce:
            self.speak("Stopping the robot")

    def extract_room(self, text):
        normalized = (text or "").lower().strip()
        normalized = normalized.replace("goe", "go")
        for wrong, right in (
            ("go to toronto", "go to room two"),
            ("the one", "room one"),
            ("the two", "room two"),
            ("the three", "room three"),
            ("gotta", "go to"),
            ("got a", "go to"),
            ("got to", "go to"),
            ("rome", "room"),
            ("to tall", "room"),
        ):
            normalized = normalized.replace(wrong, right)

        token_map = {
            "rom": "room",
            "real": "room",
            "rule": "room",
            "little": "room",
            "total": "room",
            "lunch": "one",
        }
        tokens = [token_map.get(t, t) for t in normalized.split()]
        normalized = " ".join(tokens)

        # A room number wins over a stray "reception" in the same phrase.
        word_rooms = {
            "one": "room1", "1": "room1",
            "two": "room2", "2": "room2",
            "three": "room3", "3": "room3",
            "four": "room4", "4": "room4",
            "five": "room5", "5": "room5",
        }
        for token in normalized.split():
            if token in word_rooms:
                return word_rooms[token]

        # Longer aliases first so "go to room one" wins over "room one"
        alias_hits = []
        for room, phrases in ROOM_ALIASES.items():
            for phrase in phrases:
                if phrase in normalized:
                    alias_hits.append((len(phrase), room))
        if alias_hits:
            alias_hits.sort(reverse=True)
            return alias_hits[0][1]

        if normalized in ("home", "reception"):
            return "home"

        return None

    def is_navigation_command(self, text):
        t = (text or "").lower()
        navigation_phrases = (
            "go to", "gotta", "got to", "got a", "navigate to",
            "take me to", "bring me to", "send me to", "deliver to",
            "return to", "go home", "return home", "reception", "go room",
            "room one", "room two", "room three", "room four", "room five",
            "room 1", "room 2", "room 3", "room 4", "room 5",
        )
        stripped = t.strip()
        # Bare numbers (and a bare "home") are not commands: noise produces them.
        if stripped == "reception":
            return True
        return any(phrase in t for phrase in navigation_phrases)

    def navigate_to_room(self, room_name):
        """Send a Nav2 goal without blocking the mic. New goals cancel the old one."""
        if room_name not in self.rooms:
            self.get_logger().error(f"Unknown room: {room_name}")
            self.speak("Unknown room")
            return

        x, y = self.rooms[room_name]
        with self._goal_lock:
            # Same-room "already going" blocked retries after Nav2 abort left
            # _nav_busy stuck True. Always allow a fresh goal; Nav2 preempts.
            changing = self.active_goal is not None or self._nav_busy
            if (
                self._nav_busy
                and self._pending_room == room_name
                and self.active_goal is not None
            ):
                self.get_logger().info(f"Re-commanding {room_name}")
            self._pending_room = room_name
            self._nav_busy = True

        if changing:
            # Do not cancel the old goal here. That cancel was arriving after
            # the new goal was accepted and aborting it at once.
            self.get_logger().info(f"Changing destination to {room_name}")
            self.speak(f"Going to {room_name.replace('room', 'room ')}")
        else:
            self.get_logger().info(f"Navigating to {room_name}")
            self.speak(f"Going to {room_name.replace('room', 'room ')}")

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToPose action server unavailable.")
            self.speak("Navigation server unavailable")
            with self._goal_lock:
                self._nav_busy = False
                self._pending_room = None
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        # Arrive driving straight in; the turn-around happens on departure,
        # in the open space around the room.
        yaw = goal_yaw(self._robot_xy(), (x, y))
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(
            lambda fut, room=room_name: self._on_goal_response(fut, room)
        )

    def _robot_xy(self):
        """Robot position on the map; home if TF isn't available yet."""
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return self.rooms.get("home", (0.0, 0.0))

    def _on_goal_response(self, future, room_name):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to send navigation goal: {exc}")
            with self._goal_lock:
                if self._pending_room == room_name:
                    self._nav_busy = False
                    self._pending_room = None
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Goal rejected.")
            self.speak("Goal rejected")
            with self._goal_lock:
                if self._pending_room == room_name:
                    self._nav_busy = False
                    self._pending_room = None
            return

        with self._goal_lock:
            if self._pending_room not in (None, room_name):
                try:
                    goal_handle.cancel_goal_async()
                except Exception:
                    pass
                return
            self.active_goal = goal_handle
            self._pending_room = room_name

        self.get_logger().info(f"Goal accepted for {room_name}")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda fut, room=room_name, handle=goal_handle: self._on_nav_result(
                fut, room, handle
            )
        )

    def _on_nav_result(self, future, room_name, goal_handle):
        with self._goal_lock:
            # A redirect already replaced this trip. Do not stop the new one.
            if self._pending_room != room_name or self.active_goal is not goal_handle:
                if self.active_goal is goal_handle:
                    self.active_goal = None
                # Recover stuck busy when the finishing handle was orphaned.
                if self._pending_room == room_name and self.active_goal is None:
                    self._nav_busy = False
                    self._pending_room = None
                return
            self.active_goal = None
            self._pending_room = None
            self._nav_busy = False

        for _ in range(10):
            self.cmd_vel_pub.publish(Twist())
            time.sleep(0.05)

        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f"Navigation result error: {exc}")
            self.speak("Navigation failed")
            return

        if result is None:
            self.speak("Navigation failed")
            return

        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Arrived at {room_name}")
            self.speak("Arrived")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error(f"Navigation to {room_name} ABORTED")
            self.speak("Could not reach. Try again")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warning(f"Navigation to {room_name} cancelled")
        else:
            status_name = self.status_names.get(status, str(status))
            self.get_logger().error(
                f"Navigation ended with status: {status_name}"
            )
            self.speak("Navigation failed")

    def _handle_command(self, command):
        command = (command or "").lower().strip()
        if not command:
            return

        for wrong, right in (
            ("good or a one", "go to room one"),
            ("good or one", "go to room one"),
            ("go to a one", "go to room one"),
            ("good room one", "go to room one"),
        ):
            command = command.replace(wrong, right)

        self.get_logger().info(f"Heard: {command}")
        self.get_logger().info(f"Recognized command: {command}")

        if any(word in command for word in ("exit", "quit", "stop program", "shutdown")):
            self.speak("Shutting down")
            self._shutdown = True
            self._listening = False
            self.stop_robot(announce=False)
            return

        if any(
            word in command
            for word in (
                "stop the wheel",
                "stop wheel",
                "stop the robot",
                "stop moving",
                "halt",
                "freeze",
                "brake",
            )
        ) or command.strip() in ("stop", "stop robot"):
            self.stop_robot()
            return

        room = self.extract_room(command)
        if room is not None and "[unk]" in command and any(
                f"room {n}" in command for n in ("one", "two", "three", "four", "five")) or \
                "go home" in command:
            # A clearly heard "room <number>" / "go home" counts even if the
            # start of the phrase was garbled ("[unk] room one" on the
            # headset): drop the unclear part.
            command = " ".join(w for w in command.split() if w != "[unk]")
        if room is not None and "[unk]" in command:
            # Part of the phrase was noise or other talk ("[unk] gotta toronto"
            # redirected the robot on the floor, 2026-09-30). Only clean
            # phrases may send or redirect it; "stop" is handled above.
            self.get_logger().warning(f"Ignoring unclear phrase '{command}'")
            return
        if room is not None and self.is_navigation_command(command):
            self.navigate_to_room(room)
            return
        if room is not None:
            # Bare "one", "the one", "a one", "two": Vosk makes these out of
            # background talk and motor noise. They sent the robot off six
            # times in ten minutes on the bench (2026-09-30).
            self.get_logger().warning(
                f"Ignoring short phrase '{command}': say 'room one' or 'go to room one'")
            return

        if not self.is_navigation_command(command):
            self.get_logger().warning(f"Ignoring non-navigation speech: {command}")
            return

        self.get_logger().warning("No valid room detected.")
        self.speak("Please say a valid room number.")

    def run(self):
        self._start_continuous_listen()
        self.speak("Voice control started. Listening continuously.")

        while rclpy.ok() and not self._shutdown:
            try:
                rclpy.spin_once(self, timeout_sec=0.05)
                try:
                    command = self.command_queue.get_nowait()
                except queue.Empty:
                    continue
                self._handle_command(command)
            except KeyboardInterrupt:
                self.speak("Shutting down")
                break
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
                self.speak("An error occurred.")

        self._listening = False
        if rclpy.ok():
            self.stop_robot(announce=False)


def main(args=None):
    rclpy.init(args=args)
    node = VoiceDeliveryNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        # start.sh stop sends SIGINT, which already shut the context down.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
