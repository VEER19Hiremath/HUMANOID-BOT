#!/usr/bin/env python3

import json
import os
import queue
import shutil
import subprocess
import tempfile
import threading
import time

try:
    from gtts import gTTS
except ImportError:  # pragma: no cover
    gTTS = None

try:
    from vosk import Model, KaldiRecognizer
except ImportError:  # pragma: no cover - optional runtime dependency
    Model = None
    KaldiRecognizer = None

try:
    import sounddevice as sd
except ImportError:  # pragma: no cover - optional runtime dependency
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
            self.get_logger().warn("Vosk is not installed; voice recognition is disabled.")
        else:
            self.get_logger().info("Loading Vosk model...")
            if os.path.isdir(model_path):
                try:
                    self.model = Model(model_path)
                except Exception as exc:  # pragma: no cover - runtime environment may vary
                    self.get_logger().warn(f"Failed to load Vosk model at {model_path}: {exc}")
            else:
                self.get_logger().warn(f"Vosk model directory not found: {model_path}; voice recognition disabled.")

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose"
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.active_goal = None
        self._goal_lock = threading.Lock()
        self._pending_room = None
        self._nav_busy = False

        # Continuous mic: background thread → command_queue → main loop
        self.command_queue = queue.Queue()
        self._listen_thread = None
        self._listening = False
        self._mute_mic = False
        self._shutdown = False

        # Shared room coordinates used across navigation commands.
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
        """Speak while briefly muting recognition so TTS is not heard as a command."""
        self.get_logger().info(text)
        print(f"\n[TTS] {text}\n")

        self._mute_mic = True
        try:
            # Prefer Google TTS (natural voice); fall back to espeak offline.
            if gTTS is not None:
                mp3_path = None
                try:
                    with tempfile.NamedTemporaryFile(suffix=".mp3", delete=False) as tmp:
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
                        subprocess.run(
                            player,
                            check=False,
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL,
                        )
                        return
                except Exception as exc:
                    self.get_logger().warning(f"Google TTS failed ({exc}); trying espeak")
                finally:
                    if mp3_path and os.path.exists(mp3_path):
                        try:
                            os.remove(mp3_path)
                        except OSError:
                            pass

            if shutil.which("espeak") is None:
                return
            try:
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
            # Small settle so speaker bleed does not become a command.
            time.sleep(0.15)
            self._mute_mic = False

    def _start_continuous_listen(self):
        if self.model is None or sd is None or KaldiRecognizer is None:
            self.get_logger().warn(
                "Voice recognition unavailable (missing Vosk model or sounddevice)."
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

    def _listen_loop(self):
        sample_rate = 16000
        audio_q = queue.Queue()

        def callback(indata, frames, time_info, status):
            if status:
                self.get_logger().warning(str(status))
            audio_q.put(bytes(indata))

        recognizer = KaldiRecognizer(self.model, sample_rate)

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

                    if self._mute_mic:
                        # Drop TTS echo; reset so partials do not bleed into next phrase.
                        if audio_q.qsize() > 20:
                            try:
                                while True:
                                    audio_q.get_nowait()
                            except queue.Empty:
                                pass
                        recognizer = KaldiRecognizer(self.model, sample_rate)
                        continue

                    if recognizer.AcceptWaveform(data):
                        result = json.loads(recognizer.Result())
                        text = (result.get("text") or "").lower().strip()
                        if text:
                            self.get_logger().info(f"Detected: {text}")
                            self.command_queue.put(text)
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

        # Nav2 may publish another velocity before its cancellation completes.
        for _ in range(20):
            self.cmd_vel_pub.publish(stop_msg)
            time.sleep(0.05)

        if announce:
            self.speak("Stopping the robot")

    def extract_room(self, text):

        normalized = (text or "").lower().strip()
        normalized = normalized.replace("goe", "go")
        # Prefer longer / more specific replacements first.
        # Avoid bare-token rewrites that hit unrelated speech ("from", "run", …).
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
        # Whole-token fixes only (avoid rewriting words like "from"/"lunch")
        token_map = {
            "rom": "room",
            "real": "room",
            "rule": "room",
            "little": "room",
            "total": "room",
            "lunch": "one",
        }
        tokens = normalized.split()
        tokens = [token_map.get(t, t) for t in tokens]
        normalized = " ".join(tokens)

        # Longer aliases first so "go to room one" wins over "room one"
        alias_hits = []
        for room, phrases in ROOM_ALIASES.items():
            for phrase in phrases:
                if phrase in normalized:
                    alias_hits.append((len(phrase), room))
        if alias_hits:
            alias_hits.sort(reverse=True)
            return alias_hits[0][1]

        # Word or digit room numbers, with or without "room"
        word_rooms = {
            "one": "room1", "1": "room1",
            "two": "room2", "2": "room2",
            "three": "room3", "3": "room3",
            "four": "room4", "4": "room4",
            "five": "room5", "5": "room5",
        }
        for token, room_key in word_rooms.items():
            if (
                f"go to {token}" in normalized
                or f"go {token}" in normalized
                or f"room {token}" in normalized
                or f"{token} please" in normalized
            ):
                return room_key

        return None

    def is_navigation_command(self, text):
        t = (text or "").lower()
        navigation_phrases = (
            "go to",
            "gotta",
            "got to",
            "got a",
            "navigate to",
            "take me to",
            "bring me to",
            "send me to",
            "deliver to",
            "return to",
            "go home",
            "return home",
            "reception",
            "go room",
            "room one",
            "room two",
            "room three",
            "room four",
            "room five",
            "room 1",
            "room 2",
            "room 3",
            "room 4",
            "room 5",
        )
        return any(phrase in t for phrase in navigation_phrases)

    
    def navigate_to_room(self, room_name):
        """Send a Nav2 goal without blocking the mic. New goals cancel the old one."""
        if room_name not in self.rooms:
            self.get_logger().error(f"Unknown room: {room_name}")
            self.speak("Unknown room")
            return

        x, y = self.rooms[room_name]
        with self._goal_lock:
            changing = self.active_goal is not None or self._nav_busy
            self._pending_room = room_name
            self._nav_busy = True

        if changing:
            self.get_logger().info(f"Changing destination to {room_name}")
            with self._goal_lock:
                goal = self.active_goal
                self.active_goal = None
            if goal is not None:
                try:
                    goal.cancel_goal_async()
                except Exception as e:
                    self.get_logger().warning(f"Could not cancel previous goal: {e}")
            for _ in range(10):
                self.cmd_vel_pub.publish(Twist())
                time.sleep(0.02)
            self.speak(f"Changing to {room_name.replace('room', 'Room ')}")
        else:
            self.get_logger().info(f"Navigating to {room_name}")
            self.speak(f"Navigating to {room_name.replace('room', 'Room ')}")

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
            # A newer redirect may have superseded this room.
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
            still_ours = self.active_goal is goal_handle
            if still_ours:
                self.active_goal = None
                if self._pending_room == room_name:
                    self._pending_room = None
                    self._nav_busy = False

        if not still_ours:
            return

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
            self.speak(f"Arrived at {room_name.replace('room', 'Room ')}")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error(f"Navigation to {room_name} ABORTED")
            self.speak("Navigation aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warning(f"Navigation to {room_name} cancelled")
        else:
            status_name = self.status_names.get(status, str(status))
            self.get_logger().error(f"Navigation ended with status: {status_name}")
            self.speak("Navigation failed")

    def _handle_command(self, command):
        command = (command or "").lower().strip()
        if not command:
            return

        self.get_logger().info(f"Heard: {command}")
        self.get_logger().info(f"Recognized command: {command}")

        if any(word in command for word in (
            "exit", "quit", "stop program", "shutdown",
        )):
            self.speak("Shutting down")
            self._shutdown = True
            self._listening = False
            self.stop_robot(announce=False)
            return

        if any(word in command for word in (
            "stop the wheel",
            "stop wheel",
            "stop the robot",
            "stop moving",
            "halt",
            "freeze",
            "brake",
        )) or command.strip() in ("stop", "stop robot"):
            self.stop_robot()
            return

        if not self.is_navigation_command(command):
            self.get_logger().warning(f"Ignoring non-navigation speech: {command}")
            return

        room = self.extract_room(command)
        if room is None:
            self.get_logger().warning("No valid room detected.")
            self.speak("Please say a valid room number.")
            return

        self.navigate_to_room(room)

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
    except KeyboardInterrupt:
        pass
    finally:
        node._listening = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
