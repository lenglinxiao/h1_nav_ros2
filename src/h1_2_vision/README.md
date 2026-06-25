# h1_2_vision

Robot-side CPU YOLO detector for the RealSense RGB stream.

Runtime Python dependencies are intentionally not installed by `colcon`:

```bash
python3 -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
python3 -m pip install ultralytics
```

Install the ROS messages and prepare the model:

```bash
sudo apt install ros-humble-vision-msgs ros-humble-cv-bridge
mkdir -p ~/ros2_ws/models
cp yolov8n.pt ~/ros2_ws/models/yolov8n.pt
```

Build and run:

```bash
cd ~/ros2_ws
colcon build --packages-select h1_2_vision --symlink-install
source install/setup.bash
ros2 launch h1_2_vision yolo_detector.launch.py
```
