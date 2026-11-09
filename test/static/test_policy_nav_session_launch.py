from pathlib import Path
import importlib.util

from launch import LaunchContext
from launch.actions import OpaqueFunction


ROOT = Path(__file__).resolve().parents[2]
LAUNCH = ROOT / "src/h1_2_nav_tools/launch/policy_nav_session.launch.py"


def test_policy_session_launch_uses_unique_action_and_velocity_namespaces():
    source = LAUNCH.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("backend_namespace")' in source
    assert 'DeclareLaunchArgument("cmd_topic")' in source
    assert 'SetRemap(src=f"/{name}", dst=[backend_namespace, "/", name])' in source
    assert 'SetRemap(src="/cmd_vel", dst=LaunchConfiguration("raw_cmd_topic"))' in source
    assert '"cmd_topic": cmd_topic' in source
    assert '"raw_cmd_topic": raw_cmd_topic' in source
    assert '"priority_cmd_topic": priority_cmd_topic' in source


def test_policy_session_launch_rejects_non_session_topics_and_composition():
    source = LAUNCH.read_text(encoding="utf-8")

    assert 'r"(/h1_2/policy/session/o_[0-9a-f]{32}/c_[0-9a-f]{32}/e_[0-9]+)"' in source
    assert 'backend != prefix + "/backend"' in source
    assert 'raw_topic != prefix + "/raw_cmd_vel"' in source
    assert 'if LaunchConfiguration("use_composition").perform(context).lower() == "true"' in source


def execute_validation(backend_namespace, cmd_topic, raw_cmd_topic=None, priority_cmd_topic=None):
    spec = importlib.util.spec_from_file_location("policy_nav_session", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    prefix = cmd_topic.rsplit("/", 1)[0]
    context.launch_configurations.update({
        "backend_namespace": backend_namespace,
        "cmd_topic": cmd_topic,
        "raw_cmd_topic": raw_cmd_topic or prefix + "/raw_cmd_vel",
        "priority_cmd_topic": priority_cmd_topic or prefix + "/priority_cmd_vel",
        "use_composition": "false",
    })
    opaque = next(entity for entity in module.generate_launch_description().entities
                  if isinstance(entity, OpaqueFunction))
    return opaque.execute(context)


def test_session_launch_requires_backend_and_topics_from_same_coordinator_session():
    prefix = "/h1_2/policy/session/o_" + "a" * 32 + "/c_" + "b" * 32 + "/e_7"
    assert execute_validation(prefix + "/backend", prefix + "/cmd_vel") == []

    other = "/h1_2/policy/session/o_" + "c" * 32 + "/c_" + "d" * 32 + "/e_8"
    invalid = (
        (other + "/backend", prefix + "/cmd_vel", prefix + "/raw_cmd_vel", prefix + "/priority_cmd_vel"),
        (prefix + "/nested/backend", prefix + "/cmd_vel", prefix + "/raw_cmd_vel", prefix + "/priority_cmd_vel"),
        (prefix + "/backend", prefix + "/nested/cmd_vel", prefix + "/raw_cmd_vel", prefix + "/priority_cmd_vel"),
        (prefix + "/backend", prefix + "/cmd_vel", other + "/raw_cmd_vel", prefix + "/priority_cmd_vel"),
        (prefix + "/backend", prefix + "/cmd_vel", prefix + "/raw_cmd_vel", other + "/priority_cmd_vel"),
    )
    for backend, command, raw, priority in invalid:
        try:
            execute_validation(backend, command, raw, priority)
        except RuntimeError:
            continue
        raise AssertionError(f"accepted mismatched policy session: {backend}, {command}")
