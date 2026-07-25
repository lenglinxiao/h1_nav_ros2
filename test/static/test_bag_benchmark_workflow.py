from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_benchmark_package_has_record_mapping_and_localization_replay_entries():
    setup = read("src/h1_2_bag_benchmark/setup.py")
    record_launch = read("src/h1_2_bag_benchmark/launch/record_slam_bag.launch.py")
    mapping_launch = read("src/h1_2_bag_benchmark/launch/replay_mapping_bag.launch.py")
    localization_launch = read("src/h1_2_bag_benchmark/launch/replay_localization_bag.launch.py")

    assert "slam_bag_evaluator" in setup
    assert '"/livox/lidar"' in record_launch
    assert '"/livox/imu"' in record_launch
    assert '"/slam_odom"' in record_launch
    assert "start_livox_driver" in mapping_launch
    assert '"false"' in mapping_launch
    assert "enable_sport_bridge" not in mapping_launch
    assert "enable_sport_bridge" not in localization_launch
    assert "ros2" in mapping_launch and "bag" in mapping_launch and "play" in mapping_launch
    assert "ros2" in localization_launch and "bag" in localization_launch and "play" in localization_launch


def test_replay_time_is_forwarded_to_fastlio_and_localizers():
    fastlio = read("src/h1_2_slam_bringup/launch/fastlio2_mid360_mapping.launch.py")
    icp = read("src/h1_2_slam_bringup/launch/h1_2_icp_localization.launch.py")
    global_localization = read("src/h1_2_slam_bringup/launch/h1_2_global_relocalization.launch.py")

    for source in (fastlio, icp, global_localization):
        assert 'DeclareLaunchArgument("use_sim_time", default_value="false")' in source
        assert '"use_sim_time": ParameterValue(' in source


def test_replay_launches_only_play_raw_sensor_inputs():
    for path in (
        "src/h1_2_bag_benchmark/launch/replay_mapping_bag.launch.py",
        "src/h1_2_bag_benchmark/launch/replay_localization_bag.launch.py",
    ):
        source = read(path)
        play_topics = source.split('"--topics"', maxsplit=1)[1]
        assert '"/livox/lidar"' in play_topics
        assert '"/livox/imu"' in play_topics
        assert '"/slam_odom"' not in play_topics
        assert '"/tf"' not in play_topics
