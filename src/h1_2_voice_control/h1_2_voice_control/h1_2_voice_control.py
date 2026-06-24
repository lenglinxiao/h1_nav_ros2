import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


DEFAULT_CODE_STOP = [0, 1, 2]
DEFAULT_CODE_FORWARD = [4]
DEFAULT_CODE_BACKWARD = [5]
DEFAULT_CODE_LEFT = [6]
DEFAULT_CODE_RIGHT = [7]
DEFAULT_CODE_WAVE = [52]
NO_SPEECH_CODE = 999


def parse_code_list(value):
    if isinstance(value, str):
        parts = [part.strip() for part in value.split(",")]
        return [int(part, 0) for part in parts if part]
    if isinstance(value, (list, tuple)):
        return [int(item) for item in value]
    return [int(value)]


class H12VoiceControl(Node):
    def __init__(self, speech_factory=None):
        super().__init__("h1_2_voice_control")
        self.declare_parameter("serial_port", "/dev/myspeech")
        self.declare_parameter("voice_command_topic", "/h1_2/voice_command")
        self.declare_parameter("voice_feedback_topic", "/h1_2/voice_feedback")
        self.declare_parameter("poll_rate", 10.0)
        self.declare_parameter("code_stop", DEFAULT_CODE_STOP)
        self.declare_parameter("code_forward", DEFAULT_CODE_FORWARD)
        self.declare_parameter("code_backward", DEFAULT_CODE_BACKWARD)
        self.declare_parameter("code_left", DEFAULT_CODE_LEFT)
        self.declare_parameter("code_right", DEFAULT_CODE_RIGHT)
        self.declare_parameter("code_wave", DEFAULT_CODE_WAVE)

        self.serial_port = self.get_parameter("serial_port").value
        self.code_stop = set(parse_code_list(self.get_parameter("code_stop").value))
        self.code_forward = set(parse_code_list(self.get_parameter("code_forward").value))
        self.code_backward = set(parse_code_list(self.get_parameter("code_backward").value))
        self.code_left = set(parse_code_list(self.get_parameter("code_left").value))
        self.code_right = set(parse_code_list(self.get_parameter("code_right").value))
        self.code_wave = set(parse_code_list(self.get_parameter("code_wave").value))
        self.known_codes = (
            self.code_stop
            | self.code_forward
            | self.code_backward
            | self.code_left
            | self.code_right
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
            return int(self.speech.speech_read())
        except Exception as exc:
            self.get_logger().error(f"Failed to read voice command: {exc}")
            self.speech = None
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
