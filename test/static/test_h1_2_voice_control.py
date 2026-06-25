from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
TOOLS = ROOT / "src" / "h1_2_nav_tools"
VOICE = ROOT / "src" / "h1_2_voice_control"
NODE = VOICE / "h1_2_voice_control" / "h1_2_voice_control.py"
BRIDGE = TOOLS / "h1_2_nav_tools" / "h1_2_voice_action_bridge.py"
LAUNCH = VOICE / "launch" / "h1_2_voice_control.launch.py"
BRIDGE_LAUNCH = TOOLS / "launch" / "h1_2_voice_action_bridge.launch.py"


def read(path):
    return path.read_text(encoding="utf-8")


def test_voice_control_node_registers_as_console_script_and_depends_on_serial():
    setup = read(VOICE / "setup.py")
    package_xml = read(VOICE / "package.xml")
    tools_setup = read(TOOLS / "setup.py")
    tools_package_xml = read(TOOLS / "package.xml")

    assert "h1_2_voice_control = h1_2_voice_control.h1_2_voice_control:main" in setup
    assert "<exec_depend>python3-serial</exec_depend>" in package_xml
    assert "h1_2_voice_action_bridge = h1_2_nav_tools.h1_2_voice_action_bridge:main" in tools_setup
    assert "<exec_depend>h1_2_voice_control</exec_depend>" in tools_package_xml


def test_voice_control_defaults_match_speech_module_and_voice_topics():
    source = read(NODE)
    launch = read(LAUNCH)

    assert 'self.declare_parameter("serial_port", "/dev/myspeech")' in source
    assert 'self.declare_parameter("voice_command_topic", "/h1_2/voice_command")' in source
    assert 'self.declare_parameter("voice_feedback_topic", "/h1_2/voice_feedback")' in source
    assert 'DeclareLaunchArgument("serial_port", default_value="/dev/myspeech")' in launch
    assert 'DeclareLaunchArgument("voice_command_topic", default_value="/h1_2/voice_command")' in launch
    assert 'DeclareLaunchArgument("voice_feedback_topic", default_value="/h1_2/voice_feedback")' in launch
    assert 'package="h1_2_voice_control"' in launch


def test_voice_control_default_code_map_matches_updated_speech_firmware():
    source = read(NODE)
    launch = read(LAUNCH)

    assert "DEFAULT_CODE_STOP = [1, 2]" in source
    assert "DEFAULT_CODE_FORWARD = [4]" in source
    assert "DEFAULT_CODE_SQUAT = [5]" in source
    assert "DEFAULT_CODE_STAND = [6]" in source
    assert "DEFAULT_CODE_RIGHT = [7]" in source
    assert "DEFAULT_CODE_NAV_A = [19]" in source
    assert "DEFAULT_CODE_NAV_B = [20]" in source
    assert "DEFAULT_CODE_NAV_C = [21]" in source
    assert "DEFAULT_CODE_NAV_START = [33]" in source
    assert "DEFAULT_CODE_REACH_HAND = [45]" in source
    assert "DEFAULT_CODE_RETRACT_HAND = [46]" in source
    assert "DEFAULT_CODE_WAVE = [52]" in source
    assert '"code_stop": [1, 2]' in launch
    assert '"code_nav_start": [33]' in launch
    assert '"code_wave": [52]' in launch


def test_voice_control_parses_all_command_frames_and_ignores_wake_frames():
    source = read(NODE)

    assert "def extract_command_codes" in source
    assert "frame[2] == COMMAND_FRAME_TYPE" in source
    assert "self.rx_buffer" in source
    assert "self.pending_codes" in source
    assert "self.speech.ser" in source


def test_voice_control_only_publishes_voice_messages_not_robot_actions():
    source = read(NODE)

    assert "from std_msgs.msg import Int32" in source
    assert "self.voice_pub.publish(message)" in source
    assert "Published unknown voice command code" in source
    assert "from geometry_msgs.msg import Twist" not in source
    assert "from h1_2_nav_msgs.srv import ExecuteGesture" not in source
    assert "Rosmaster" not in source
    assert "LocoClient" not in source


def test_voice_control_handles_speech_lib_and_serial_failures_without_exiting():
    source = read(NODE)

    assert "from Speech_Lib import Speech" in source
    assert "Voice module unavailable" in source
    assert "self.speech = None" in source


def test_voice_control_plays_feedback_only_when_action_bridge_reports_completion():
    source = read(NODE)

    assert "self.feedback_sub = self.create_subscription" in source
    assert "def handle_feedback" in source
    assert "self.acknowledge(code)" in source


def test_voice_action_bridge_maps_voice_messages_to_existing_control_interfaces():
    source = read(BRIDGE)
    launch = read(BRIDGE_LAUNCH)

    assert "from geometry_msgs.msg import Twist" in source
    assert "from h1_2_nav_msgs.srv import ExecuteGesture, ListMemoryPoints" in source
    assert "from nav2_msgs.action import NavigateToPose" in source
    assert 'self.declare_parameter("voice_command_topic", "/h1_2/voice_command")' in source
    assert 'self.declare_parameter("cmd_topic", "/cmd_vel_voice")' in source
    assert 'self.declare_parameter("gesture_service_name", "/h1_2_loco/execute_gesture")' in source
    assert '"low_stand"' in source
    assert '"high_stand"' in source
    assert '"reach_hand"' in source
    assert '"retract_hand"' in source
    assert '"A"' in source
    assert '"B"' in source
    assert '"C"' in source
    assert '"START"' in source
    assert "self.action_client.send_goal_async" in source
    assert "self.feedback_pub.publish(message)" in source
    assert "unknown voice code" in source
    assert 'DeclareLaunchArgument("voice_command_topic", default_value="/h1_2/voice_command")' in launch
    assert 'DeclareLaunchArgument("cmd_topic", default_value="/cmd_vel_voice")' in launch
    assert '"code_nav_a": [19]' in launch
    assert '"code_retract_hand": [46]' in launch
