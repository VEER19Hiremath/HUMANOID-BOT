#!/usr/bin/env python3

import json
import os
import queue
import shutil
import subprocess
import tempfile
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
            "Voice control ready; navigation server will be checked per command."
        )

        self.speak("Hospital robot ready")

    def speak(self, text):

        self.get_logger().info(text)
        print(f"\n[TTS] {text}\n")

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



    # def record_audio(self, duration=4):

    #     sample_rate = 16000

    #     self.speak("Listening")

    #     self.get_logger().info("Recording...")

    #     audio = sd.rec(
    #         int(duration * sample_rate),
    #         samplerate=sample_rate,
    #         channels=1,
    #         dtype='int16'
    #     )

    #     sd.wait()

    #     temp_file = tempfile.NamedTemporaryFile(
    #         suffix=".wav",
    #         delete=False
    #     )

    #     write(
    #         temp_file.name,
    #         sample_rate,
    #         audio
    #     )

    #     return temp_file.name



    def recognize_command(self):

        if self.model is None or sd is None or KaldiRecognizer is None:
            self.get_logger().warn("Voice recognition is unavailable because the Vosk model or audio dependency is missing.")
            return ""

        self.speak("Listening")

        sample_rate = 16000

        q = queue.Queue()

        def callback(indata, frames, time, status):

            if status:
                self.get_logger().warning(str(status))

            q.put(bytes(indata))

        recognizer = KaldiRecognizer(self.model, sample_rate)

        with sd.RawInputStream(
            samplerate=sample_rate,
            blocksize=8000,
            dtype="int16",
            channels=1,
            callback=callback,
        ):

            self.get_logger().info("Listening...")

            while True:

                data = q.get()

                if recognizer.AcceptWaveform(data):

                    result = json.loads(
                        recognizer.Result()
                    )

                    text = result.get("text", "").lower()

                    self.get_logger().info(
                        f"Detected: {text}"
                    )

                    return text
    def stop_robot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0
        if self.active_goal is not None:
            try:
                self.active_goal.cancel_goal_async()
            except Exception as e:
                self.get_logger().warning(f"Could not cancel active goal: {e}")

        try:
            self.nav_client.cancel_all_goals_async()
        except Exception as e:
            self.get_logger().warning(f"Could not cancel nav goals: {e}")

        # Nav2 may publish another velocity before its cancellation completes.
        # Hold zero on the final command topic long enough for cancellation to
        # propagate through the controller and velocity smoother.
        for _ in range(20):
            self.cmd_vel_pub.publish(stop_msg)
            time.sleep(0.05)

        self.speak("Stopping the robot")

    def extract_room(self, text):

        normalized = (text or "").lower().strip()
        normalized = normalized.replace("goe", "go")
        # Common Vosk mishearings
        for wrong, right in (
            ("rome", "room"),
            ("rom", "room"),
            ("real", "room"),
            ("rule", "room"),
            ("run", "room"),
            ("little", "room"),
            ("total", "room"),
            ("to tall", "room"),
            ("lunch", "one"),
        ):
            normalized = normalized.replace(wrong, right)

        for room, phrases in ROOM_ALIASES.items():
            for phrase in phrases:
                if phrase in normalized:
                    return room

        # Accept room numbers without a preceding "room" keyword, e.g. "go to 1" or "1 please"
        for room_name, room_key in {"1": "room1", "2": "room2", "3": "room3", "4": "room4", "5": "room5"}.items():
            if f"go to {room_name}" in normalized or f"go {room_name}" in normalized or f"{room_name} please" in normalized or f"{room_name}" in normalized:
                if normalized.count(room_name) == 1:
                    return room_key

        return None

    def is_navigation_command(self, text):
        navigation_phrases = (
            "go to",
            "go ",
            "navigate to",
            "take me to",
            "bring me to",
            "send me to",
            "deliver to",
            "return to",
        )
        return any(phrase in (text or "") for phrase in navigation_phrases)
    
    def navigate_to_room(self, room_name):

        if room_name not in self.rooms:

            self.get_logger().error(f"Unknown room: {room_name}")
            self.speak("Unknown room")
            return

        x, y = self.rooms[room_name]

        self.get_logger().info(f"Navigating to {room_name}")

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose = PoseStamped()

        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Face forward
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.speak(f"Navigating to {room_name.replace('room', 'Room ')}")

        if not self.nav_client.wait_for_server(timeout_sec=10.0):

            self.get_logger().error(
                "NavigateToPose action server unavailable."
            )

            self.speak("Navigation server unavailable")

            return

        send_goal_future = self.nav_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(
            self,
            send_goal_future
        )

        goal_handle = send_goal_future.result()

        if goal_handle is None:

            self.get_logger().error(
                "Failed to send navigation goal."
            )

            self.speak("Failed to send goal")

            return

        if not goal_handle.accepted:

            self.get_logger().error("Goal rejected.")

            self.speak("Goal rejected")

            return

        self.active_goal = goal_handle
        self.get_logger().info("Goal accepted.")

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        result = result_future.result()
        self.active_goal = None

        for _ in range(10):
            self.cmd_vel_pub.publish(Twist())
            time.sleep(0.05)

        if result is None:

            self.get_logger().error(
                "Navigation returned no result."
            )

            self.speak("Navigation failed")

            return

        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:

            self.get_logger().info(
                f"Arrived at {room_name}"
            )

            self.speak(
                f"Arrived at {room_name.replace('room', 'Room ')}"
            )

        elif status == GoalStatus.STATUS_ABORTED:

            self.get_logger().error(
                f"Navigation to {room_name} ABORTED"
            )

            self.speak("Navigation aborted")

        elif status == GoalStatus.STATUS_CANCELED:

            self.get_logger().warning(
                f"Navigation to {room_name} cancelled"
            )

            self.speak("Navigation cancelled")

        else:

            status_name = self.status_names.get(
                status,
                str(status)
            )

            self.get_logger().error(
                f"Navigation ended with status: {status_name}"
            )

            self.speak("Navigation failed")
    
    def run(self):

        self.speak("Voice control started")

        while rclpy.ok():

            try:

                command = self.recognize_command()

                if not command:
                    continue

                command = command.lower().strip()

                self.get_logger().info(f"Heard: {command}")

                self.get_logger().info(
                    f"Recognized command: {command}"
                )

                # Exit command
                if any(word in command for word in [
                    "exit",
                    "quit",
                    "stop program",
                    "shutdown"
                ]):

                    self.speak("Shutting down")

                    break

                if any(word in command for word in [
                    "stop the wheel",
                    "stop wheel",
                    "stop the robot",
                    "stop moving",
                    "halt",
                    "freeze",
                    "brake",
                    "stop"
                ]):
                    self.stop_robot()
                    continue

                if not self.is_navigation_command(command):
                    self.get_logger().warning(
                        f"Ignoring non-navigation speech: {command}"
                    )
                    continue

                room = self.extract_room(command)

                if room is None:

                    self.get_logger().warning(
                        "No valid room detected."
                    )

                    self.speak(
                        "Please say a valid room number."
                    )

                    continue

                self.navigate_to_room(room)

            except KeyboardInterrupt:

                self.speak("Shutting down")

                break

            except Exception as e:

                self.get_logger().error(
                    f"Error: {e}"
                )

                self.speak(
                    "An error occurred."
                )
    

def main(args=None):

    rclpy.init(args=args)

    node = VoiceDeliveryNode()

    try:

        node.run()

    except KeyboardInterrupt:

        node.get_logger().info(
            "Voice delivery node interrupted."
        )

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()


# #!/usr/bin/env python3

# import os
# import tempfile

# import whisper
# import sounddevice as sd
# from scipy.io.wavfile import write

# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient

# from nav2_msgs.action import NavigateToPose
# from geometry_msgs.msg import PoseStamped


# class VoiceDeliveryNode(Node):

#     def __init__(self):
#         super().__init__('voice_delivery_node')

#         self.get_logger().info("Loading Whisper model...")

#         self.model = whisper.load_model("base")

#         self.nav_client = ActionClient(
#             self,
#             NavigateToPose,
#             'navigate_to_pose'
#         )

#         self.rooms = {

#             # REPLACE THESE WITH YOUR ACTUAL COORDINATES

#             "room1": (8.5, 3.0),

#             "room2": (-1.5, 3.0),

#             "room3": (-1.5, -3.0),

#             "room4": (-8.5, 3.0),

#             "room5": (-8.5, -3.0),

#             "home": (0.0, 0.0)
#         }

#         self.get_logger().info("Waiting for Nav2...")
#         self.nav_client.wait_for_server()

#         self.get_logger().info("Ready")

#     def speak(self, text):

#         print(f"\n[TTS] {text}\n")

#         os.system(f'espeak "{text}"')

#     def record_audio(self, duration=4):

#         sample_rate = 16000

#         self.speak("Listening")

#         self.get_logger().info("Recording...")

#         audio = sd.rec(
#             int(duration * sample_rate),
#             samplerate=sample_rate,
#             channels=1,
#             dtype='int16'
#         )

#         sd.wait()

#         temp_file = tempfile.NamedTemporaryFile(
#             suffix=".wav",
#             delete=False
#         )

#         write(temp_file.name, sample_rate, audio)

#         return temp_file.name

#     def recognize_command(self):

#         wav_file = self.record_audio()

#         self.get_logger().info("Transcribing...")

#         result = self.model.transcribe(
#             wav_file,
#             language="en"
#         )

#         text = result["text"].lower()

#         self.get_logger().info(
#             f"Detected: {text}"
#         )

#         os.remove(wav_file)

#         return text

#     def extract_room(self, text):

#         if "room 1" in text or "room one" in text:
#             return "room1"

#         if "room 2" in text or "room two" in text:
#             return "room2"

#         if "room 3" in text or "room three" in text:
#             return "room3"

#         if "room 4" in text or "room four" in text:
#             return "room4"

#         if "room 5" in text or "room five" in text:
#             return "room5"
        
#         if "home" in text:
#             return "home"

#         return None

#     def navigate_to_room(self, room_name):

#         if room_name not in self.rooms:

#             self.get_logger().error(
#                 f"Unknown room: {room_name}"
#             )

#             return False

#         x, y = self.rooms[room_name]

#         self.speak(
#             f"Navigating to {room_name}"
#         )

#         goal_msg = NavigateToPose.Goal()

#         goal_msg.pose = PoseStamped()

#         goal_msg.pose.header.frame_id = "map"

#         goal_msg.pose.header.stamp = (
#             self.get_clock().now().to_msg()
#         )

#         goal_msg.pose.pose.position.x = x
#         goal_msg.pose.pose.position.y = y

#         goal_msg.pose.pose.orientation.w = 1.0

#         send_goal_future = self.nav_client.send_goal_async(
#             goal_msg
#         )

#         rclpy.spin_until_future_complete(
#             self,
#             send_goal_future
#         )

#         goal_handle = send_goal_future.result()

#         if not goal_handle.accepted:

#             self.get_logger().error(
#                 "Goal rejected"
#             )

#             return False

#         self.get_logger().info(
#             "Goal accepted"
#         )

#         result_future = goal_handle.get_result_async()

#         rclpy.spin_until_future_complete(
#             self,
#             result_future
#         )

#         self.speak(
#             f"Arrived at {room_name}"
#         )

#         return True

#     def run(self):

#         self.speak(
#             "Hospital robot ready"
#         )

#         while rclpy.ok():

#             try:

#                 text = self.recognize_command()

#                 room = self.extract_room(text)

#                 if room is None:

#                     self.speak(
#                         "Room not recognized"
#                     )

#                     continue

#                 self.navigate_to_room(room)

#             except KeyboardInterrupt:

#                 break

#             except Exception as e:

#                 self.get_logger().error(
#                     str(e)
#                 )


# def main(args=None):

#     rclpy.init(args=args)

#     node = VoiceDeliveryNode()

#     node.run()

#     node.destroy_node()

#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()