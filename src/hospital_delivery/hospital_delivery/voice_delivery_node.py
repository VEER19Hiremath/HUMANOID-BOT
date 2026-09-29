import json
import os
import queue
import re
import shutil
import socket
import subprocess
import tempfile
import threading
import time

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
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist

from hospital_delivery.room_config import ROOM_ALIASES, ROOM_COORDS


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
        self.active_goal = None
        self._goal_lock = threading.Lock()
        self._pending_room = None
        self._nav_busy = False

        self.command_queue = queue.Queue()
        self._listen_thread = None
        self._listening = False
        self._capture_source_id = None
        self._mute_mic = False
        self._shutdown = False
        self._partial_text = ""
        self._quiet_since = None
        self._last_live_cmd = ""
        self._last_live_at = 0.0

        self._noise_floor = 280.0
        self._stable_partial = ""
        self._stable_since = 0.0

        self.rooms = ROOM_COORDS

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
        if len(data) < 2:
            return 0.0
        total = 0
        count = 0
        for i in range(0, len(data) - 1, 2):
            sample = int.from_bytes(data[i:i + 2], "little", signed=True)
            total += sample * sample
            count += 1
        if count == 0:
            return 0.0
        return (total / count) ** 0.5

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
        for aliases in ROOM_ALIASES.values():
            phrases.extend(aliases)
        recognizer.SetGrammar(json.dumps(phrases))
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

    def _queue_command(self, text):
        """Queue one navigation phrase. The same text is ignored for 3 s."""
        text = (text or "").lower().strip()
        if not self._accept_phrase(text):
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
        words = text.split()
        if words == ["home"]:
            hold = 0.9
        elif len(words) >= 3:
            hold = 0.2
        else:
            hold = 0.65
        if time.monotonic() - self._stable_since < hold:
            return recognizer
        if self._queue_command(text):
            return self._new_recognizer(sample_rate)
        return recognizer

    def _on_recognized_text(self, recognizer):
        result = json.loads(recognizer.Result())
        text = (result.get("text") or "").lower().strip()
        self._partial_text = ""
        self._quiet_since = None
        self._stable_partial = ""
        self._queue_command(text)

    def _finish_partial(self, recognizer, sample_rate):
        """Vosk ends a phrase only after quiet audio. The mic gate used to drop
        that quiet audio, so 'go to room one' stayed a partial and never drove."""
        text = ""
        try:
            result = json.loads(recognizer.FinalResult() or "{}")
            text = (result.get("text") or "").lower().strip()
        except Exception:
            text = ""
        if not text:
            text = (self._partial_text or "").lower().strip()
        self._partial_text = ""
        self._quiet_since = None
        self._stable_partial = ""
        recognizer = self._new_recognizer(sample_rate)
        self._queue_command(text)
        return recognizer

    def _consume_audio(self, recognizer, data, sample_rate):
        """Feed the recognizer only while the mic is above the room floor."""
        rms = self._chunk_rms(data)
        if self._mute_mic or not self._is_speech(rms):
            if self._mute_mic:
                self._partial_text = ""
                self._quiet_since = None
                self._stable_partial = ""
                return self._new_recognizer(sample_rate)
            if self._partial_text:
                if self._quiet_since is None:
                    self._quiet_since = time.monotonic()
                elif time.monotonic() - self._quiet_since >= 0.5:
                    recognizer = self._finish_partial(recognizer, sample_rate)
            return recognizer
        self._quiet_since = None
        if recognizer.AcceptWaveform(data):
            self._on_recognized_text(recognizer)
        else:
            partial = json.loads(recognizer.PartialResult() or "{}")
            heard = (partial.get("partial") or "").strip()
            if heard:
                self._partial_text = heard
                self.get_logger().info(f"Hearing: {heard}")
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
        if source_id:
            self.get_logger().info(
                f"Listening continuously from microphone {source_id}"
            )
        else:
            self.get_logger().warning("No microphone is connected.")
        chunks = 0
        loudest = 0.0
        try:
            while self._listening and rclpy.ok() and not self._shutdown:
                data = proc.stdout.read(4000)
                if not data:
                    err = proc.stderr.read().decode("utf-8", "replace").strip()
                    raise RuntimeError(err or "microphone capture stopped")
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

    def _listen_hfp(self):
        """Read the call-link PCM published by scripts/hfp_mic.py."""
        sample_rate = 16000
        recognizer = self._new_recognizer(sample_rate)
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.settimeout(2.0)
        sock.connect("/tmp/hospital_hfp_mic.sock")
        self.get_logger().info("Listening continuously from the headset call link")
        loudest = 0.0
        last_report = time.monotonic()
        try:
            while self._listening and rclpy.ok() and not self._shutdown:
                try:
                    data = sock.recv(4000)
                except socket.timeout:
                    continue
                if not data:
                    raise RuntimeError("headset call link closed")
                rms = self._chunk_rms(data)
                loudest = max(loudest, rms)
                now = time.monotonic()
                if now - last_report >= 2.0:
                    self.get_logger().info(
                        f"Mic level rms={rms:.0f} peak_rms={loudest:.0f}"
                    )
                    loudest = 0.0
                    last_report = now
                recognizer = self._consume_audio(recognizer, data, sample_rate)
        finally:
            sock.close()

    def _listen_loop(self):
        # Prefer the headset call-link when present; fall back to PipeWire / sounddevice.
        # A leftover /tmp/hospital_hfp_mic.sock with no server must not block forever.
        hfp_failures = 0
        hfp_wait = 0
        while self._listening and not self._shutdown and rclpy.ok() and hfp_wait < 8:
            if os.path.exists("/tmp/hospital_hfp_mic.sock"):
                try:
                    self._listen_hfp()
                    hfp_failures = 0
                    continue
                except Exception as exc:
                    hfp_failures += 1
                    self.get_logger().warning(
                        f"Headset call link failed ({exc}); "
                        f"retry {hfp_failures}/3"
                    )
                    if not self._listening or self._shutdown:
                        return
                    if hfp_failures >= 3:
                        try:
                            os.unlink("/tmp/hospital_hfp_mic.sock")
                        except OSError:
                            pass
                        self.get_logger().warning(
                            "Dropping stale headset socket; falling back to system mic"
                        )
                        break
                    time.sleep(1.0)
                    continue
            hfp_wait += 1
            time.sleep(1.0)

        if self._pipewire_capture_cmd() is not None:
            while self._listening and not self._shutdown and rclpy.ok():
                try:
                    self._listen_pipewire()
                    return
                except Exception as exc:
                    self.get_logger().warning(
                        f"PipeWire microphone failed ({exc}); retrying"
                    )
                    if not self._listening or self._shutdown:
                        return
                    time.sleep(1.0)

        if sd is None:
            self.get_logger().error("No microphone capture backend available")
            self._listening = False
            return

        sample_rate = 16000
        audio_q = queue.Queue()

        def callback(indata, frames, time_info, status):
            audio_q.put(bytes(indata))

        recognizer = self._new_recognizer(sample_rate)
        try:
            with sd.RawInputStream(
                samplerate=sample_rate,
                blocksize=4000,
                dtype="int16",
                channels=1,
                callback=callback,
            ):
                self.get_logger().info("Listening continuously...")
                while self._listening and rclpy.ok() and not self._shutdown:
                    try:
                        data = audio_q.get(timeout=0.25)
                    except queue.Empty:
                        continue
                    if self._mute_mic and audio_q.qsize() > 20:
                        try:
                            while True:
                                audio_q.get_nowait()
                        except queue.Empty:
                            pass
                    recognizer = self._consume_audio(
                        recognizer, data, sample_rate
                    )
        except Exception as exc:
            self.get_logger().error(f"Microphone listen loop failed: {exc}")
        finally:
            self._listening = False

    def stop_robot(self, announce=True):
        stop_msg = Twist()
        with self._goal_lock:
            goal = self.active_goal
            self.active_goal = None
            self._pending_room = None
            self._nav_busy = False
        if goal is not None:
            try:
                goal.cancel_goal_async()
            except Exception as e:
                self.get_logger().warning(f"Could not cancel active goal: {e}")
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
        if stripped in ("home", "reception", "one", "two", "three", "four", "five"):
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
        goal_msg.pose.pose.orientation.w = 1.0

        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(
            lambda fut, room=room_name: self._on_goal_response(fut, room)
        )

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
        if room is not None:
            self.navigate_to_room(room)
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
        self.stop_robot(announce=False)


def main(args=None):
    rclpy.init(args=args)
    node = VoiceDeliveryNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
