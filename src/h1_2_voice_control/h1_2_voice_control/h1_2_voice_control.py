from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


DEFAULT_CODE_STOP = [1, 2]
DEFAULT_CODE_FORWARD = [4]
DEFAULT_CODE_SQUAT = [5]
DEFAULT_CODE_STAND = [6]
DEFAULT_CODE_RIGHT = [7]
DEFAULT_CODE_NAV_A = [19]
DEFAULT_CODE_NAV_B = [20]
DEFAULT_CODE_NAV_C = [21]
DEFAULT_CODE_NAV_START = [33]
DEFAULT_CODE_REACH_HAND = [45]
DEFAULT_CODE_RETRACT_HAND = [46]
DEFAULT_CODE_WAVE = [52]
NO_SPEECH_CODE = 999
FRAME_HEADER = b"\xAA\x55"
FRAME_SIZE = 5
FRAME_FOOTER = 0xFB
COMMAND_FRAME_TYPE = 0x00


def parse_code_list(value):
    if isinstance(value, str):
        parts = [part.strip() for part in value.split(",")]
        return [int(part, 0) for part in parts if part]
    if isinstance(value, (list, tuple)):
        return [int(item) for item in value]
    return [int(value)]


def extract_command_codes(buffer):
    codes = []
    while True:
        header_index = buffer.find(FRAME_HEADER)
        if header_index < 0:
            if buffer[-1:] == FRAME_HEADER[:1]:
                del buffer[:-1]
            else:
                buffer.clear()
            break
        if header_index:
            del buffer[:header_index]
        if len(buffer) < FRAME_SIZE:
            break
        if buffer[FRAME_SIZE - 1] != FRAME_FOOTER:
            del buffer[0]
            continue
        frame = bytes(buffer[:FRAME_SIZE])
        del buffer[:FRAME_SIZE]
        if frame[2] == COMMAND_FRAME_TYPE:
            codes.append(frame[3])
    return codes


class H12VoiceControl(Node):
    def __init__(self, speech_factory=None):
        super().__init__("h1_2_voice_control")
        self.declare_parameter("serial_port", "/dev/myspeech")
        self.declare_parameter("voice_command_topic", "/h1_2/voice_command")
        self.declare_parameter("voice_feedback_topic", "/h1_2/voice_feedback")
        self.declare_parameter("poll_rate", 10.0)
        self.declare_parameter("code_stop", DEFAULT_CODE_STOP)
        self.declare_parameter("code_forward", DEFAULT_CODE_FORWARD)
        self.declare_parameter("code_squat", DEFAULT_CODE_SQUAT)
        self.declare_parameter("code_stand", DEFAULT_CODE_STAND)
        self.declare_parameter("code_right", DEFAULT_CODE_RIGHT)
        self.declare_parameter("code_nav_a", DEFAULT_CODE_NAV_A)
        self.declare_parameter("code_nav_b", DEFAULT_CODE_NAV_B)
        self.declare_parameter("code_nav_c", DEFAULT_CODE_NAV_C)
        self.declare_parameter("code_nav_start", DEFAULT_CODE_NAV_START)
        self.declare_parameter("code_reach_hand", DEFAULT_CODE_REACH_HAND)
        self.declare_parameter("code_retract_hand", DEFAULT_CODE_RETRACT_HAND)
        self.declare_parameter("code_wave", DEFAULT_CODE_WAVE)

        self.serial_port = self.get_parameter("serial_port").value
        self.code_stop = set(parse_code_list(self.get_parameter("code_stop").value))
        self.code_forward = set(parse_code_list(self.get_parameter("code_forward").value))
        self.code_squat = set(parse_code_list(self.get_parameter("code_squat").value))
        self.code_stand = set(parse_code_list(self.get_parameter("code_stand").value))
        self.code_right = set(parse_code_list(self.get_parameter("code_right").value))
        self.code_nav_a = set(parse_code_list(self.get_parameter("code_nav_a").value))
        self.code_nav_b = set(parse_code_list(self.get_parameter("code_nav_b").value))
        self.code_nav_c = set(parse_code_list(self.get_parameter("code_nav_c").value))
        self.code_nav_start = set(parse_code_list(self.get_parameter("code_nav_start").value))
        self.code_reach_hand = set(parse_code_list(self.get_parameter("code_reach_hand").value))
        self.code_retract_hand = set(parse_code_list(self.get_parameter("code_retract_hand").value))
        self.code_wave = set(parse_code_list(self.get_parameter("code_wave").value))
        self.known_codes = (
            self.code_stop
            | self.code_forward
            | self.code_squat
            | self.code_stand
            | self.code_right
            | self.code_nav_a
            | self.code_nav_b
            | self.code_nav_c
            | self.code_nav_start
            | self.code_reach_hand
            | self.code_retract_hand
            | self.code_wave
        )

        self.voice_pub = self.create_publisher(
            Int32, self.get_parameter("voice_command_topic").value, 10
        )
        self.feedback_sub = self.create_subscription(
            Int32, self.get_parameter("voice_feedback_topic").value, self.handle_feedback, 10
        )

        self.speech = None
        self.speech_error_logged = False
        self.rx_buffer = bytearray()
        self.pending_codes = deque()

        self.speech_factory = speech_factory or self.create_speech
        self.try_open_speech()

        rate = max(float(self.get_parameter("poll_rate").value), 1.0)
        self.create_timer(1.0 / rate, self.update)
        self.get_logger().info(
            f"H1-2 voice control ready: serial={self.serial_port}, "
            f"voice_command_topic={self.get_parameter('voice_command_topic').value}, "
            f"voice_feedback_topic={self.get_parameter('voice_feedback_topic').value}"
        )

    def create_speech(self):
        try:
            from Speech_Lib import Speech
        except ImportError as exc:
            raise RuntimeError("Speech_Lib is not installed") from exc
        return Speech(com=self.serial_port)

    def try_open_speech(self):
        if self.speech is not None:
            return
        try:
            self.speech = self.speech_factory()
            self.speech_error_logged = False
        except Exception as exc:
            if not self.speech_error_logged:
                self.get_logger().error(f"Voice module unavailable on {self.serial_port}: {exc}")
                self.speech_error_logged = True

    def update(self):
        self.try_open_speech()
        code = self.read_speech_code()
        if code is not None and code != NO_SPEECH_CODE:
            self.handle_code(code)

    def read_speech_code(self):
        if self.speech is None:
            return None
        try:
            if self.pending_codes:
                return self.pending_codes.popleft()
            if hasattr(self.speech, "ser"):
                waiting = getattr(self.speech.ser, "in_waiting", None)
                if waiting is None:
                    waiting = self.speech.ser.inWaiting()
                if waiting:
                    self.rx_buffer.extend(self.speech.ser.read(waiting))
                    self.pending_codes.extend(extract_command_codes(self.rx_buffer))
                return self.pending_codes.popleft() if self.pending_codes else None
            return int(self.speech.speech_read())
        except Exception as exc:
            self.get_logger().error(f"Failed to read voice command: {exc}")
            self.speech = None
            self.rx_buffer.clear()
            self.pending_codes.clear()
            return None

    def handle_code(self, code):
        message = Int32()
        message.data = int(code)
        self.voice_pub.publish(message)
        if code in self.known_codes:
            self.get_logger().info(f"Published voice command code={code}")
        else:
            self.get_logger().warn(f"Published unknown voice command code={code}")

    def handle_feedback(self, message):
        code = int(message.data)
        if code not in self.known_codes:
            return
        self.acknowledge(code)

    def acknowledge(self, code):
        if self.speech is None:
            return
        try:
            self.speech.void_write(code)
        except Exception as exc:
            self.get_logger().warn(f"Failed to play voice feedback for code {code}: {exc}")

def main():
    rclpy.init()
    node = H12VoiceControl()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
