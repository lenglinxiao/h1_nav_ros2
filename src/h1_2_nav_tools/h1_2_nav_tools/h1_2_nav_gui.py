import json
import os
import sys
import threading
from pathlib import Path

import rclpy
from h1_2_nav_msgs.srv import ExecuteGesture, RouteStatus, StartRoute, StopRoute
from PyQt5 import QtCore, QtWidgets
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


CONFIG_PATH = Path.home() / ".ros" / "h1_2_nav_gui.json"
GESTURE_OPTIONS = ["", "wave_hand", "shake_hand"]


def find_workspace_root():
    candidates = []
    env_root = os.environ.get("H1_2_NAV_WS")
    if env_root:
        candidates.append(Path(env_root))
    cwd = Path.cwd()
    candidates.extend([cwd, *cwd.parents])
    candidates.extend(
        [
            Path.home() / "h1_nav2_ros2_ws",
            Path("/home/llx/h1_nav2_ros2_ws"),
        ]
    )
    for candidate in candidates:
        if (candidate / "run_ai_slam_nav.sh").exists():
            return candidate
    return cwd


def point_sort_key(name):
    if name == "START":
        return (0, name)
    return (1, name)


class RosServiceBridge(QtCore.QObject):
    status_ready = QtCore.pyqtSignal(str, str, int, str)
    service_done = QtCore.pyqtSignal(str, bool, str)

    def __init__(self):
        super().__init__()
        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = Node("h1_2_nav_gui")
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.start_client = self.node.create_client(StartRoute, "start_memory_route")
        self.stop_client = self.node.create_client(StopRoute, "stop_memory_route")
        self.status_client = self.node.create_client(RouteStatus, "memory_route_status")
        self.gesture_client = self.node.create_client(ExecuteGesture, "/h1_2_loco/execute_gesture")

    def shutdown(self):
        self.executor.shutdown()
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def call_start_route(self):
        self._call_simple("start_route", self.start_client, StartRoute.Request())

    def call_stop_route(self):
        self._call_simple("stop_route", self.stop_client, StopRoute.Request())

    def call_status(self):
        if not self.status_client.service_is_ready():
            self.status_ready.emit("unavailable", "", 0, "memory_route_status is not available")
            return
        future = self.status_client.call_async(RouteStatus.Request())
        future.add_done_callback(self._status_done)

    def call_gesture(self, name):
        if not self.gesture_client.service_is_ready():
            self.service_done.emit("gesture", False, "/h1_2_loco/execute_gesture is not available")
            return
        request = ExecuteGesture.Request()
        request.name = name
        future = self.gesture_client.call_async(request)
        future.add_done_callback(lambda done: self._gesture_done(done, name))

    def _call_simple(self, label, client, request):
        if not client.service_is_ready():
            self.service_done.emit(label, False, f"{label} service is not available")
            return
        future = client.call_async(request)
        future.add_done_callback(lambda done: self._simple_done(done, label))

    def _simple_done(self, future, label):
        try:
            result = future.result()
            self.service_done.emit(label, bool(result.success), result.message)
        except Exception as exc:
            self.service_done.emit(label, False, str(exc))

    def _status_done(self, future):
        try:
            result = future.result()
            self.status_ready.emit(
                result.state,
                result.current_goal,
                int(result.current_index),
                result.message,
            )
        except Exception as exc:
            self.status_ready.emit("error", "", 0, str(exc))

    def _gesture_done(self, future, name):
        try:
            result = future.result()
            message = f"{name}: sdk_ret={result.sdk_ret}, {result.message}"
            self.service_done.emit("gesture", bool(result.success), message)
        except Exception as exc:
            self.service_done.emit("gesture", False, str(exc))


class H12NavGui(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.workspace_root = find_workspace_root()
        self.points_file = self.workspace_root / "maps" / "memory_points.json"
        self.config = self.load_config()
        self.nav_process = None
        self.rviz_process = None
        self.voice_process = None
        self.stop_processes = []
        self.ros = RosServiceBridge()
        self.ros.status_ready.connect(self.on_status_ready)
        self.ros.service_done.connect(self.on_service_done)

        self.setWindowTitle("H1-2 Offline Navigation Console")
        self.resize(1320, 820)
        self.build_ui()
        self.load_memory_points()
        self.restore_config()

        self.status_timer = QtCore.QTimer(self)
        self.status_timer.timeout.connect(self.ros.call_status)
        self.status_timer.start(1000)

    def build_ui(self):
        self.setStyleSheet(
            """
            QMainWindow {
                background: #eef2f6;
                color: #17202a;
                font-size: 14px;
            }
            QWidget#sidebar {
                background: #111827;
                color: #f8fafc;
            }
            QLabel#brandMark {
                background: #2dd4bf;
                color: #06251f;
                border-radius: 8px;
                font-weight: 900;
                font-size: 14px;
            }
            QLabel#brandTitle {
                color: #f8fafc;
                font-weight: 900;
                font-size: 15px;
            }
            QLabel#brandSubtle, QLabel#navItem, QLabel#sideCaption {
                color: #94a3b8;
            }
            QLabel#navItemActive {
                background: rgba(45, 212, 191, 0.14);
                border-left: 3px solid #2dd4bf;
                border-radius: 7px;
                color: #ffffff;
                font-weight: 800;
                padding: 8px 10px;
            }
            QLabel#navItem {
                padding: 8px 13px;
                font-weight: 700;
            }
            QLabel#pageTitle {
                color: #17202a;
                font-size: 22px;
                font-weight: 900;
            }
            QLabel#pageSubtitle {
                color: #687386;
                font-size: 13px;
            }
            QFrame#panel, QFrame#configBand {
                background: #ffffff;
                border: 1px solid #d8e0ea;
                border-radius: 8px;
            }
            QFrame#panelHeader {
                background: #fbfcfe;
                border-bottom: 1px solid #d8e0ea;
            }
            QLabel#panelTitle {
                color: #17202a;
                font-weight: 900;
            }
            QLabel#sectionMark {
                background: #edf2f7;
                color: #475569;
                border-radius: 6px;
                font-size: 11px;
                font-weight: 900;
            }
            QLabel#fieldLabel, QLabel#statusKey {
                color: #687386;
                font-size: 12px;
                font-weight: 900;
            }
            QLabel#statePill {
                background: #eef2f7;
                color: #475569;
                border-radius: 11px;
                padding: 4px 10px;
                font-weight: 900;
            }
            QLineEdit, QComboBox {
                min-height: 34px;
                border: 1px solid #d8e0ea;
                border-radius: 7px;
                padding: 0 10px;
                background: #ffffff;
                color: #17202a;
            }
            QLineEdit:focus, QComboBox:focus {
                border: 1px solid #6ea3ff;
            }
            QPushButton {
                min-height: 34px;
                border: 1px solid #d8e0ea;
                border-radius: 7px;
                padding: 0 12px;
                background: #ffffff;
                color: #17202a;
                font-weight: 800;
            }
            QPushButton:hover {
                background: #f8fafc;
                border-color: #aeb9c8;
            }
            QPushButton[class="primary"] {
                background: #2463eb;
                border-color: #2463eb;
                color: #ffffff;
            }
            QPushButton[class="primary"]:hover {
                background: #194bb8;
                border-color: #194bb8;
            }
            QPushButton[class="danger"] {
                background: #fff1f0;
                border-color: #f3b8b2;
                color: #b42318;
            }
            QPushButton[class="soft"] {
                background: #e9f7f5;
                border-color: #bdd7d4;
                color: #0f766e;
            }
            QCheckBox {
                color: #17202a;
                font-weight: 700;
            }
            QTableWidget {
                background: #ffffff;
                border: 1px solid #d8e0ea;
                border-radius: 8px;
                gridline-color: #e8edf3;
                selection-background-color: #eff6ff;
                selection-color: #17202a;
            }
            QHeaderView::section {
                min-height: 34px;
                padding: 0 10px;
                background: #f3f6fa;
                color: #526173;
                border: 0;
                border-right: 1px solid #e8edf3;
                border-bottom: 1px solid #d8e0ea;
                font-size: 12px;
                font-weight: 900;
            }
            QPlainTextEdit {
                border: 0;
                padding: 10px;
                font-family: monospace;
                font-size: 12px;
            }
            QPlainTextEdit#logText {
                background: #0b1220;
                color: #dbeafe;
            }
            QPlainTextEdit#commandText {
                background: #f8fafc;
                color: #334155;
            }
            """
        )

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root_layout = QtWidgets.QHBoxLayout(central)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)

        sidebar = QtWidgets.QWidget()
        sidebar.setObjectName("sidebar")
        sidebar.setFixedWidth(248)
        sidebar_layout = QtWidgets.QVBoxLayout(sidebar)
        sidebar_layout.setContentsMargins(18, 16, 18, 14)
        sidebar_layout.setSpacing(16)

        brand_layout = QtWidgets.QHBoxLayout()
        brand_mark = QtWidgets.QLabel("H2")
        brand_mark.setObjectName("brandMark")
        brand_mark.setFixedSize(36, 36)
        brand_mark.setAlignment(QtCore.Qt.AlignCenter)
        brand_text_layout = QtWidgets.QVBoxLayout()
        brand_text_layout.setSpacing(2)
        brand_title = QtWidgets.QLabel("H1-2 Console")
        brand_title.setObjectName("brandTitle")
        brand_subtitle = QtWidgets.QLabel("Offline navigation")
        brand_subtitle.setObjectName("brandSubtle")
        brand_text_layout.addWidget(brand_title)
        brand_text_layout.addWidget(brand_subtitle)
        brand_layout.addWidget(brand_mark)
        brand_layout.addLayout(brand_text_layout)
        sidebar_layout.addLayout(brand_layout)

        for text in [
            "RT   Route",
            "MP   Map config",
            "RV   RViz",
            "LG   Logs",
        ]:
            item = QtWidgets.QLabel(text)
            item.setObjectName("navItemActive" if text.startswith("RT") else "navItem")
            sidebar_layout.addWidget(item)
        sidebar_layout.addStretch()
        service_caption = QtWidgets.QLabel("Service status")
        service_caption.setObjectName("sideCaption")
        self.service_status_label = QtWidgets.QLabel("mock/ROS bridge ready")
        self.service_status_label.setObjectName("brandTitle")
        sidebar_layout.addWidget(service_caption)
        sidebar_layout.addWidget(self.service_status_label)
        root_layout.addWidget(sidebar)

        content = QtWidgets.QWidget()
        content_layout = QtWidgets.QVBoxLayout(content)
        content_layout.setContentsMargins(24, 18, 24, 22)
        content_layout.setSpacing(18)
        root_layout.addWidget(content, 1)

        header_layout = QtWidgets.QHBoxLayout()
        title_layout = QtWidgets.QVBoxLayout()
        title_layout.setSpacing(4)
        title = QtWidgets.QLabel("Offline Navigation Control")
        title.setObjectName("pageTitle")
        subtitle = QtWidgets.QLabel("Configure route, bind gestures, manage RViz, then start route execution.")
        subtitle.setObjectName("pageSubtitle")
        title_layout.addWidget(title)
        title_layout.addWidget(subtitle)
        header_layout.addLayout(title_layout)
        header_layout.addStretch()
        self.top_start_nav_button = self.make_button("Start Navigation", "primary")
        self.top_stop_all_button = self.make_button("Stop All", "danger")
        self.top_open_rviz_button = self.make_button("Open RViz", "soft")
        header_layout.addWidget(self.top_start_nav_button)
        header_layout.addWidget(self.top_stop_all_button)
        header_layout.addWidget(self.top_open_rviz_button)
        content_layout.addLayout(header_layout)

        config_band = QtWidgets.QFrame()
        config_band.setObjectName("configBand")
        config_layout = QtWidgets.QGridLayout(config_band)
        config_layout.setContentsMargins(16, 14, 16, 16)
        config_layout.setHorizontalSpacing(12)
        config_layout.setVerticalSpacing(8)
        self.map_edit = QtWidgets.QLineEdit()
        self.pcd_edit = QtWidgets.QLineEdit()
        self.initial_pose_edit = QtWidgets.QLineEdit("START")
        self.localization_combo = QtWidgets.QComboBox()
        self.localization_combo.addItems(["global_gicp", "icp"])
        self.auto_start_check = QtWidgets.QCheckBox("Auto start route")
        self.auto_start_check.setChecked(True)
        self.return_to_start_check = QtWidgets.QCheckBox("Return to START before loop")

        config_layout.addWidget(self.make_field_label("Map YAML"), 0, 0)
        config_layout.addLayout(
            self.make_path_row(self.map_edit, "Map YAML", "YAML (*.yaml *.yml);;All (*)"),
            1,
            0,
        )
        config_layout.addWidget(self.make_field_label("PCD Map"), 0, 1)
        config_layout.addLayout(
            self.make_path_row(self.pcd_edit, "PCD map", "PCD (*.pcd);;All (*)"),
            1,
            1,
        )
        config_layout.addWidget(self.make_field_label("Localization"), 0, 2)
        config_layout.addWidget(self.localization_combo, 1, 2)
        config_layout.addWidget(self.make_field_label("Initial Pose"), 0, 3)
        config_layout.addWidget(self.initial_pose_edit, 1, 3)
        config_layout.setColumnStretch(0, 3)
        config_layout.setColumnStretch(1, 3)
        config_layout.setColumnStretch(2, 1)
        config_layout.setColumnStretch(3, 1)
        content_layout.addWidget(config_band)

        workspace_layout = QtWidgets.QHBoxLayout()
        workspace_layout.setSpacing(18)
        content_layout.addLayout(workspace_layout, 1)

        route_panel, route_body = self.make_panel("1", "Memory Route")
        route_layout = QtWidgets.QVBoxLayout(route_body)
        route_layout.setContentsMargins(16, 16, 16, 16)
        route_layout.setSpacing(10)
        self.route_table = QtWidgets.QTableWidget(0, 4)
        self.route_table.setHorizontalHeaderLabels(["Use", "Point", "Gesture", "Position Summary"])
        self.route_table.verticalHeader().setVisible(False)
        self.route_table.setAlternatingRowColors(False)
        self.route_table.setShowGrid(False)
        self.route_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.route_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.route_table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Fixed)
        self.route_table.setColumnWidth(0, 58)
        self.route_table.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)
        self.route_table.horizontalHeader().setSectionResizeMode(2, QtWidgets.QHeaderView.ResizeToContents)
        self.route_table.horizontalHeader().setSectionResizeMode(3, QtWidgets.QHeaderView.Stretch)
        self.route_table.itemChanged.connect(lambda *_: self.update_command_preview())
        route_layout.addWidget(self.route_table)
        route_buttons = QtWidgets.QHBoxLayout()
        self.reload_points_button = self.make_button("Reload Points")
        self.select_all_button = self.make_button("Select All")
        self.clear_route_button = self.make_button("Clear")
        route_buttons.addWidget(self.reload_points_button)
        route_buttons.addWidget(self.select_all_button)
        route_buttons.addWidget(self.clear_route_button)
        route_buttons.addStretch()
        route_layout.addLayout(route_buttons)
        workspace_layout.addWidget(route_panel, 1)

        side_layout = QtWidgets.QVBoxLayout()
        side_layout.setSpacing(18)
        workspace_layout.addLayout(side_layout)

        process_panel, process_body = self.make_panel("2", "Process")
        process_layout = QtWidgets.QVBoxLayout(process_body)
        process_layout.setContentsMargins(16, 16, 16, 16)
        process_layout.setSpacing(10)
        checks_frame = QtWidgets.QFrame()
        checks_frame.setObjectName("configBand")
        checks_layout = QtWidgets.QVBoxLayout(checks_frame)
        checks_layout.setContentsMargins(10, 10, 10, 10)
        checks_layout.addWidget(self.auto_start_check)
        checks_layout.addWidget(self.return_to_start_check)
        checks_layout.addWidget(self.make_field_label("RViz Mode"))
        self.rviz_mode_combo = QtWidgets.QComboBox()
        self.rviz_mode_combo.addItems(["offline", "online"])
        checks_layout.addWidget(self.rviz_mode_combo)
        process_layout.addWidget(checks_frame)
        self.close_rviz_button = self.make_button("Close RViz")
        self.restart_rviz_button = self.make_button("Restart RViz")
        self.start_nav_button = self.make_button("Start offline navigation", "primary")
        self.stop_all_button = self.make_button("Stop all processes", "danger")
        self.open_rviz_button = self.make_button("Open RViz", "soft")
        self.start_voice_button = self.make_button("Start Voice Control", "soft")
        self.stop_voice_button = self.make_button("Stop Voice Control")
        process_layout.addWidget(self.start_nav_button)
        process_layout.addWidget(self.stop_all_button)
        rviz_row = QtWidgets.QHBoxLayout()
        rviz_row.addWidget(self.open_rviz_button)
        rviz_row.addWidget(self.close_rviz_button)
        process_layout.addLayout(rviz_row)
        process_layout.addWidget(self.restart_rviz_button)
        voice_row = QtWidgets.QHBoxLayout()
        voice_row.addWidget(self.start_voice_button)
        voice_row.addWidget(self.stop_voice_button)
        process_layout.addLayout(voice_row)
        side_layout.addWidget(process_panel)

        control_panel, control_body = self.make_panel("3", "Route Control")
        control_layout = QtWidgets.QVBoxLayout(control_body)
        control_layout.setContentsMargins(16, 16, 16, 16)
        control_layout.setSpacing(10)
        self.start_route_button = self.make_button("Start Route", "primary")
        self.stop_route_button = self.make_button("Stop Route")
        self.refresh_status_button = self.make_button("Refresh Status")
        self.wave_button = self.make_button("Test Wave")
        self.shake_button = self.make_button("Test Shake")
        route_control_row = QtWidgets.QHBoxLayout()
        route_control_row.addWidget(self.start_route_button)
        route_control_row.addWidget(self.stop_route_button)
        gesture_row = QtWidgets.QHBoxLayout()
        gesture_row.addWidget(self.wave_button)
        gesture_row.addWidget(self.shake_button)
        control_layout.addLayout(route_control_row)
        control_layout.addWidget(self.refresh_status_button)
        control_layout.addLayout(gesture_row)
        side_layout.addWidget(control_panel)

        status_panel, status_body = self.make_panel("4", "Runtime Status")
        status_layout = QtWidgets.QGridLayout(status_body)
        status_layout.setContentsMargins(16, 16, 16, 16)
        status_layout.setHorizontalSpacing(12)
        status_layout.setVerticalSpacing(10)
        self.state_label = QtWidgets.QLabel("idle")
        self.state_label.setObjectName("statePill")
        self.goal_label = QtWidgets.QLabel("A / index 0")
        self.message_label = QtWidgets.QLabel("Waiting for offline navigation startup.")
        self.message_label.setWordWrap(True)
        status_layout.addWidget(self.make_field_label("State", "statusKey"), 0, 0)
        status_layout.addWidget(self.state_label, 0, 1)
        status_layout.addWidget(self.make_field_label("Current Goal", "statusKey"), 1, 0)
        status_layout.addWidget(self.goal_label, 1, 1)
        status_layout.addWidget(self.make_field_label("Message", "statusKey"), 2, 0)
        status_layout.addWidget(self.message_label, 2, 1)
        side_layout.addWidget(status_panel)
        side_layout.addStretch()

        bottom_layout = QtWidgets.QHBoxLayout()
        bottom_layout.setSpacing(18)
        content_layout.addLayout(bottom_layout)

        log_panel, log_body = self.make_panel("LG", "Process Log")
        log_layout = QtWidgets.QVBoxLayout(log_body)
        log_layout.setContentsMargins(0, 0, 0, 0)
        self.log_text = QtWidgets.QPlainTextEdit()
        self.log_text.setObjectName("logText")
        self.log_text.setReadOnly(True)
        self.log_text.setMinimumHeight(170)
        log_layout.addWidget(self.log_text)
        bottom_layout.addWidget(log_panel, 1)

        command_panel, command_body = self.make_panel("SH", "Generated Command")
        command_layout = QtWidgets.QVBoxLayout(command_body)
        command_layout.setContentsMargins(0, 0, 0, 0)
        self.command_text = QtWidgets.QPlainTextEdit()
        self.command_text.setObjectName("commandText")
        self.command_text.setReadOnly(True)
        self.command_text.setMinimumWidth(390)
        self.command_text.setMinimumHeight(170)
        command_layout.addWidget(self.command_text)
        bottom_layout.addWidget(command_panel)

        self.top_start_nav_button.clicked.connect(self.start_navigation)
        self.top_stop_all_button.clicked.connect(self.stop_all)
        self.top_open_rviz_button.clicked.connect(self.open_rviz)
        self.start_nav_button.clicked.connect(self.start_navigation)
        self.stop_all_button.clicked.connect(self.stop_all)
        self.open_rviz_button.clicked.connect(self.open_rviz)
        self.close_rviz_button.clicked.connect(self.close_rviz)
        self.restart_rviz_button.clicked.connect(self.restart_rviz)
        self.start_voice_button.clicked.connect(self.start_voice_control)
        self.stop_voice_button.clicked.connect(self.stop_voice_control)
        self.reload_points_button.clicked.connect(self.load_memory_points)
        self.select_all_button.clicked.connect(self.select_all_route_points)
        self.clear_route_button.clicked.connect(self.clear_route_points)
        self.start_route_button.clicked.connect(self.ros.call_start_route)
        self.stop_route_button.clicked.connect(self.ros.call_stop_route)
        self.refresh_status_button.clicked.connect(self.ros.call_status)
        self.wave_button.clicked.connect(lambda: self.ros.call_gesture("wave_hand"))
        self.shake_button.clicked.connect(lambda: self.ros.call_gesture("shake_hand"))
        for widget in [
            self.map_edit,
            self.pcd_edit,
            self.initial_pose_edit,
        ]:
            widget.textChanged.connect(self.update_command_preview)
        self.localization_combo.currentTextChanged.connect(self.update_command_preview)
        self.auto_start_check.toggled.connect(self.update_command_preview)
        self.return_to_start_check.toggled.connect(self.update_command_preview)

    def make_button(self, text, style_class=None):
        button = QtWidgets.QPushButton(text)
        if style_class:
            button.setProperty("class", style_class)
        return button

    def make_field_label(self, text, object_name="fieldLabel"):
        label = QtWidgets.QLabel(text)
        label.setObjectName(object_name)
        return label

    def make_panel(self, mark, title):
        panel = QtWidgets.QFrame()
        panel.setObjectName("panel")
        outer = QtWidgets.QVBoxLayout(panel)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)
        header = QtWidgets.QFrame()
        header.setObjectName("panelHeader")
        header.setFixedHeight(48)
        header_layout = QtWidgets.QHBoxLayout(header)
        header_layout.setContentsMargins(16, 0, 16, 0)
        mark_label = QtWidgets.QLabel(mark)
        mark_label.setObjectName("sectionMark")
        mark_label.setAlignment(QtCore.Qt.AlignCenter)
        mark_label.setFixedSize(24, 24)
        title_label = QtWidgets.QLabel(title)
        title_label.setObjectName("panelTitle")
        header_layout.addWidget(mark_label)
        header_layout.addWidget(title_label)
        header_layout.addStretch()
        body = QtWidgets.QWidget()
        body.setContentsMargins(16, 16, 16, 16)
        outer.addWidget(header)
        outer.addWidget(body, 1)
        return panel, body

    def make_path_row(self, target, title, filter_text):
        row = QtWidgets.QHBoxLayout()
        row.setSpacing(8)
        row.addWidget(target, 1)
        row.addWidget(self.make_browse_button(target, title, filter_text))
        return row

    def make_browse_button(self, target, title, filter_text):
        button = self.make_button("Browse")
        button.clicked.connect(lambda: self.browse_file(target, title, filter_text))
        return button

    def browse_file(self, target, title, filter_text):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, title, str(self.workspace_root), filter_text)
        if path:
            target.setText(path)
            self.update_command_preview()

    def load_config(self):
        try:
            return json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return {}

    def save_config(self):
        data = {
            "map": self.map_edit.text().strip(),
            "pcd": self.pcd_edit.text().strip(),
            "localization": self.localization_combo.currentText(),
            "initial_pose": self.initial_pose_edit.text().strip(),
            "auto_start_route": self.auto_start_check.isChecked(),
            "return_to_start": self.return_to_start_check.isChecked(),
            "memory_route": self.build_memory_route(),
            "gesture_route": self.build_gesture_route(),
        }
        CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
        CONFIG_PATH.write_text(json.dumps(data, indent=2, sort_keys=True), encoding="utf-8")

    def restore_config(self):
        self.map_edit.setText(self.config.get("map", str(self.workspace_root / "maps" / "h1_2_lab.yaml")))
        self.pcd_edit.setText(self.config.get("pcd", str(self.workspace_root / "maps" / "h1_2_lab.pcd")))
        localization = self.config.get("localization", "global_gicp")
        index = self.localization_combo.findText(localization)
        if index >= 0:
            self.localization_combo.setCurrentIndex(index)
        self.initial_pose_edit.setText(self.config.get("initial_pose", "START"))
        self.auto_start_check.setChecked(bool(self.config.get("auto_start_route", True)))
        self.return_to_start_check.setChecked(bool(self.config.get("return_to_start", False)))
        self.apply_saved_route(self.config.get("memory_route", "A,B,C"), self.config.get("gesture_route", ""))
        self.update_command_preview()

    def load_memory_points(self):
        try:
            data = json.loads(self.points_file.read_text(encoding="utf-8"))
            points = data.get("points", {})
            if not isinstance(points, dict):
                raise ValueError("points object is invalid")
        except Exception as exc:
            self.append_log(f"Failed to load memory points: {exc}")
            points = {}

        self.route_table.blockSignals(True)
        try:
            self.route_table.setRowCount(0)
            for name in sorted(points.keys(), key=point_sort_key):
                record = points[name]
                row = self.route_table.rowCount()
                self.route_table.insertRow(row)
                use_item = QtWidgets.QTableWidgetItem()
                use_item.setFlags(QtCore.Qt.ItemIsUserCheckable | QtCore.Qt.ItemIsEnabled)
                use_item.setCheckState(QtCore.Qt.Unchecked if name == "START" else QtCore.Qt.Checked)
                self.route_table.setItem(row, 0, use_item)
                name_item = QtWidgets.QTableWidgetItem(name)
                name_item.setFlags(QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable)
                self.route_table.setItem(row, 1, name_item)
                combo = QtWidgets.QComboBox()
                combo.addItems(GESTURE_OPTIONS)
                combo.currentTextChanged.connect(self.update_command_preview)
                self.route_table.setCellWidget(row, 2, combo)
                position = record.get("position", {})
                summary = "x={:.3f}, y={:.3f}, z={:.3f}".format(
                    float(position.get("x", 0.0)),
                    float(position.get("y", 0.0)),
                    float(position.get("z", 0.0)),
                )
                pos_item = QtWidgets.QTableWidgetItem(summary)
                pos_item.setFlags(QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable)
                self.route_table.setItem(row, 3, pos_item)
        finally:
            self.route_table.blockSignals(False)
        self.apply_saved_route(self.config.get("memory_route", "A,B,C"), self.config.get("gesture_route", ""))
        self.append_log(f"Loaded {self.route_table.rowCount()} memory points from {self.points_file}")
        self.update_command_preview()

    def apply_saved_route(self, memory_route, gesture_route):
        route_names = [name.strip() for name in memory_route.split(",") if name.strip()]
        gestures = self.parse_gesture_route(gesture_route)
        for row in range(self.route_table.rowCount()):
            name = self.route_table.item(row, 1).text()
            use_item = self.route_table.item(row, 0)
            use_item.setCheckState(QtCore.Qt.Checked if name in route_names else QtCore.Qt.Unchecked)
            combo = self.route_table.cellWidget(row, 2)
            gesture = gestures.get(name, "")
            index = combo.findText(gesture)
            combo.setCurrentIndex(index if index >= 0 else 0)
        self.update_command_preview()

    @staticmethod
    def parse_gesture_route(gesture_route):
        gestures = {}
        for item in gesture_route.split(","):
            if ":" not in item:
                continue
            name, gesture = item.split(":", 1)
            name = name.strip()
            gesture = gesture.strip()
            if name and gesture:
                gestures[name] = gesture
        return gestures

    def select_all_route_points(self):
        for row in range(self.route_table.rowCount()):
            name = self.route_table.item(row, 1).text()
            self.route_table.item(row, 0).setCheckState(QtCore.Qt.Unchecked if name == "START" else QtCore.Qt.Checked)
        self.update_command_preview()

    def clear_route_points(self):
        for row in range(self.route_table.rowCount()):
            self.route_table.item(row, 0).setCheckState(QtCore.Qt.Unchecked)
        self.update_command_preview()

    def selected_route_rows(self):
        rows = []
        for row in range(self.route_table.rowCount()):
            if self.route_table.item(row, 0).checkState() == QtCore.Qt.Checked:
                rows.append(row)
        return rows

    def build_memory_route(self):
        names = [self.route_table.item(row, 1).text() for row in self.selected_route_rows()]
        return ",".join(names)

    def build_gesture_route(self):
        entries = []
        for row in self.selected_route_rows():
            name = self.route_table.item(row, 1).text()
            gesture = self.route_table.cellWidget(row, 2).currentText().strip()
            if gesture:
                entries.append(f"{name}:{gesture}")
        return ",".join(entries)

    def validate_navigation_inputs(self):
        map_path = Path(self.map_edit.text().strip()).expanduser()
        pcd_path = Path(self.pcd_edit.text().strip()).expanduser()
        if not map_path.is_file():
            return False, f"Map file does not exist: {map_path}"
        if not pcd_path.is_file():
            return False, f"PCD file does not exist: {pcd_path}"
        if not self.build_memory_route():
            return False, "Select at least one route point."
        return True, ""

    def build_navigation_args(self):
        return [
            "mode=navigation",
            f"map={Path(self.map_edit.text().strip()).expanduser()}",
            f"localization={self.localization_combo.currentText()}",
            f"pcd={Path(self.pcd_edit.text().strip()).expanduser()}",
            f"initial_pose={self.initial_pose_edit.text().strip() or 'START'}",
            f"memory_route={self.build_memory_route()}",
            f"gesture_route={self.build_gesture_route()}",
            f"auto_start_route={str(self.auto_start_check.isChecked()).lower()}",
            f"return_to_start={str(self.return_to_start_check.isChecked()).lower()}",
        ]

    def update_command_preview(self):
        if not hasattr(self, "command_text"):
            return
        script = self.workspace_root / "run_ai_slam_nav.sh"
        command = " \\\n  ".join([str(script), *self.build_navigation_args()])
        self.command_text.setPlainText(command)

    def start_navigation(self):
        ok, message = self.validate_navigation_inputs()
        if not ok:
            self.append_log(message)
            QtWidgets.QMessageBox.warning(self, "Invalid navigation settings", message)
            return
        if self.nav_process and self.nav_process.state() != QtCore.QProcess.NotRunning:
            self.append_log("Navigation is already running.")
            return
        self.save_config()
        script = self.workspace_root / "run_ai_slam_nav.sh"
        args = self.build_navigation_args()
        self.nav_process = self.make_process("nav")
        self.nav_process.start(str(script), args)
        self.append_log("$ " + " ".join([str(script), *args]))
        self.update_command_preview()

    def stop_all(self):
        self.stop_voice_control()
        script = self.workspace_root / "stop_ai_slam_nav.sh"
        process = self.make_process("stop")
        process.finished.connect(lambda *_: self.stop_processes.remove(process) if process in self.stop_processes else None)
        self.stop_processes.append(process)
        process.start(str(script), [])
        self.append_log("$ " + str(script))

    def start_voice_control(self):
        if self.voice_process and self.voice_process.state() != QtCore.QProcess.NotRunning:
            self.append_log("Voice control is already running.")
            return
        self.voice_process = self.make_process("voice")
        self.voice_process.start(
            "ros2",
            ["launch", "h1_2_voice_control", "h1_2_voice_control.launch.py"],
        )
        self.append_log("$ ros2 launch h1_2_voice_control h1_2_voice_control.launch.py")

    def stop_voice_control(self):
        if self.voice_process and self.voice_process.state() != QtCore.QProcess.NotRunning:
            self.voice_process.terminate()
            self.append_log("Stopping voice control...")
        else:
            self.append_log("Voice control is not running.")

    def open_rviz(self):
        if self.rviz_process and self.rviz_process.state() != QtCore.QProcess.NotRunning:
            self.append_log("RViz is already running.")
            return
        rviz_mode = self.rviz_mode_combo.currentText()
        self.rviz_process = self.make_process("rviz")
        self.rviz_process.start(
            "ros2",
            ["launch", "h1_2_slam_bringup", "h1_2_nav2_rviz.launch.py", f"mode:={rviz_mode}"],
        )
        self.append_log(f"$ ros2 launch h1_2_slam_bringup h1_2_nav2_rviz.launch.py mode:={rviz_mode}")

    def close_rviz(self):
        if self.rviz_process and self.rviz_process.state() != QtCore.QProcess.NotRunning:
            self.rviz_process.terminate()
            self.append_log("Closing RViz...")
        else:
            self.append_log("RViz is not running.")

    def restart_rviz(self):
        self.close_rviz()
        QtCore.QTimer.singleShot(1000, self.open_rviz)

    def make_process(self, label):
        process = QtCore.QProcess(self)
        process.setWorkingDirectory(str(self.workspace_root))
        process.setProcessChannelMode(QtCore.QProcess.SeparateChannels)
        process.readyReadStandardOutput.connect(lambda: self.read_process_output(label, process, False))
        process.readyReadStandardError.connect(lambda: self.read_process_output(label, process, True))
        process.finished.connect(lambda code, status: self.append_log(f"[{label}] exited code={code} status={status}"))
        return process

    def read_process_output(self, label, process, is_error):
        data = process.readAllStandardError() if is_error else process.readAllStandardOutput()
        text = bytes(data).decode("utf-8", errors="replace").rstrip()
        if text:
            stream = "stderr" if is_error else "stdout"
            for line in text.splitlines():
                self.append_log(f"[{label} {stream}] {line}")

    def on_status_ready(self, state, goal, index, message):
        self.state_label.setText(state)
        self.goal_label.setText(f"{goal or '-'} / index {index}")
        self.message_label.setText(message or "-")

    def on_service_done(self, label, success, message):
        status = "ok" if success else "failed"
        self.append_log(f"[{label}] {status}: {message}")

    def append_log(self, text):
        self.log_text.appendPlainText(text)
        self.log_text.verticalScrollBar().setValue(self.log_text.verticalScrollBar().maximum())

    def closeEvent(self, event):
        self.save_config()
        running = []
        if self.nav_process and self.nav_process.state() != QtCore.QProcess.NotRunning:
            running.append("navigation")
        if self.rviz_process and self.rviz_process.state() != QtCore.QProcess.NotRunning:
            running.append("RViz")
        if self.voice_process and self.voice_process.state() != QtCore.QProcess.NotRunning:
            running.append("voice control")
        if running:
            reply = QtWidgets.QMessageBox.question(
                self,
                "Close GUI",
                "Close GUI only? Running processes will continue unless you click Stop all.",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.Yes,
            )
            if reply != QtWidgets.QMessageBox.Yes:
                event.ignore()
                return
        self.ros.shutdown()
        event.accept()


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = H12NavGui()
    window.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
