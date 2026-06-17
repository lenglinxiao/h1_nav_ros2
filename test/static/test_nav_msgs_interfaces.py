from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
MSG_PKG = ROOT / "src" / "h1_2_nav_msgs"


def read(relative_path: str) -> str:
    return (MSG_PKG / relative_path).read_text(encoding="utf-8")


def test_nav_msgs_package_declares_interface_generation():
    package_xml = read("package.xml")
    cmake = read("CMakeLists.txt")

    assert "<name>h1_2_nav_msgs</name>" in package_xml
    assert "<buildtool_depend>ament_cmake</buildtool_depend>" in package_xml
    assert "rosidl_default_generators" in package_xml
    assert "rosidl_interface_packages" in package_xml
    assert "rosidl_generate_interfaces" in cmake
    assert "DEPENDENCIES geometry_msgs" in cmake


def test_memory_point_services_have_expected_contracts():
    save_srv = read("srv/SaveMemoryPoint.srv")
    delete_srv = read("srv/DeleteMemoryPoint.srv")
    list_srv = read("srv/ListMemoryPoints.srv")

    assert "string name" in save_srv
    assert "bool overwrite" in save_srv
    assert "geometry_msgs/PoseStamped pose" in save_srv
    assert delete_srv.strip().splitlines() == ["string name", "---", "bool success", "string message"]
    assert "string[] names" in list_srv
    assert "geometry_msgs/PoseStamped[] poses" in list_srv


def test_route_control_services_have_expected_contracts():
    start_srv = read("srv/StartRoute.srv")
    stop_srv = read("srv/StopRoute.srv")
    status_srv = read("srv/RouteStatus.srv")

    assert start_srv.strip().splitlines() == ["---", "bool success", "string message"]
    assert stop_srv.strip().splitlines() == ["---", "bool success", "string message"]
    assert "string state" in status_srv
    assert "string current_goal" in status_srv
    assert "uint32 current_index" in status_srv
