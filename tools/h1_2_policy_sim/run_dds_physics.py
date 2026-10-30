#!/usr/bin/env python3
"""Run one loopback-only ROS node and real-time MuJoCo physics bridge."""
import argparse
import csv
import json
import hashlib
from datetime import datetime, timezone
import math
import os
import signal
from pathlib import Path
import subprocess
import sys
import time
import rclpy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

parser = argparse.ArgumentParser()
parser.add_argument('--output', required=True)
parser.add_argument('--bridge', default='/tmp/h1_2_policy_sim_build/h1_2_dds_physics')
parser.add_argument('--duration', type=float, default=30)
parser.add_argument('--vx', type=float, default=0.0)
parser.add_argument('--wz', type=float, default=0.0)
args = parser.parse_args()
if (not all(map(math.isfinite, (args.duration, args.vx, args.wz))) or
        not 0 < args.duration <= 30 or abs(args.vx) > .2 or abs(args.wz) > .5):
    parser.error('invalid duration or velocity')
os.environ.update(ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='43', RMW_IMPLEMENTATION='rmw_fastrtps_cpp', ROS_LOG_DIR='/tmp/h1_sim_ros_logs')
out = Path(args.output)
out.mkdir(parents=True, exist_ok=True)
stage = out / 'stage.txt'
def set_stage(value):
    temp = out / 'stage.tmp'
    temp.write_text(value + '\n')
    temp.replace(stage)
set_stage('passive')
def terminate_handler(signum, frame):
    raise RuntimeError('interrupted; stopping owned processes')
signal.signal(signal.SIGTERM, terminate_handler)
signal.signal(signal.SIGINT, terminate_handler)
children = []
logs = []
node = None
run_error = None
try:
    for name, command in [('bridge', [args.bridge, str(stage), str(out / 'trace.csv')]),
                          ('node', ['/home/llx/h1_nav2_ros2_ws/install/h1_2_policy_bridge/lib/h1_2_policy_bridge/h1_2_policy_node'])]:
        log = open(out / (name + '.log'), 'w')
        logs.append(log)
        children.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
    rclpy.init()
    node = rclpy.create_node('h1_sim_orchestrator')
    publisher = node.create_publisher(Twist, '/cmd_vel_final', 10)
    def service(name):
        client = node.create_client(Trigger, '/h1_2_policy_node/' + name)
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError('service unavailable: ' + name)
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if not future.done() or not future.result().success:
            raise RuntimeError('transition rejected: ' + name + ': ' + (future.result().message if future.done() else 'timeout'))
        print(name, 'accepted', flush=True)
    def wait(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if any(p.poll() is not None for p in children):
                raise RuntimeError('node or physics bridge exited unexpectedly')
            rclpy.spin_once(node, timeout_sec=.05)
    wait(2)
    service('passive')
    service('fix_stand')
    set_stage('fix')
    wait(3.3)
    service('arm_hold_stand')
    set_stage('stand')
    wait(3.4)
    service('arm_hold')
    set_stage('active')
    deadline = time.monotonic() + args.duration + .4
    while time.monotonic() < deadline:
        if any(p.poll() is not None for p in children):
            raise RuntimeError('node or physics bridge exited during active stage')
        msg = Twist()
        msg.linear.x = args.vx
        msg.angular.z = args.wz
        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=.05)
    wait(.15)
except Exception as exc:
    run_error = str(exc)
finally:
    if node is not None:
        node.destroy_node()
        rclpy.shutdown()
    for process in children:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    for process in children:
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
    for log in logs:
        log.close()

if run_error is not None:
    (out / 'summary.json').write_text(json.dumps({'evaluation_status': 'preparation_or_runtime_failed', 'errors': [run_error]}, indent=2, allow_nan=False) + '\n')
    print(run_error, file=sys.stderr)
    sys.exit(1)

evaluator = Path(__file__).with_name('evaluate_dds.py')
result = subprocess.run([sys.executable, str(evaluator), str(out / 'trace.csv'),
                         str(out / 'summary.json'), str(args.duration), str(args.vx), str(args.wz)])
summary_path = out / 'summary.json'
summary = json.loads(summary_path.read_text())
model = Path('/home/llx/policy.onnx')
scene = Path('/home/llx/unitree_rl_mjlab/src/assets/robots/unitree_h1_2/xmls/scene_h1_2.xml')
summary.update(model_path=str(model), model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
               scene_path=str(scene), scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),
               lowstate_period_seconds=.002, controller_period_seconds=.02,
               recorded_utc=datetime.now(timezone.utc).isoformat(),
               dds_domain=42, network_interface='lo', ros_domain=43)
summary_path.write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
sys.exit(result.returncode)
