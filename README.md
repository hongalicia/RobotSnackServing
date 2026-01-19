# RobotSnackServing

## Steps:
1. TM5S Startup
    1. Press poweron on the controller and wait for booting
    2. Long press M/A, wait till it starts blinking
    3. press **+-++-**
    4. Press M/A twice, to switch to manual mode
    5. File > a_gripper_initial, and press play on controller, and press stop after it finishes.
    6. File > ROS_Control, and press play on controller, it will start to listen commands

2. Connect to TM_Driver:
- Make sure ethernet setting is using fixed **IPv4: 192.168.1.3**, and
```bash
ros2 run tm_driver tm_driver robot_ip:=192.168.1.10
```
- Use Ctrl+Shift+V to paste onto terminal

3. gripper_server:
```bash
cd ~/ITRI-GraspGen/ROS2_server && \
/usr/bin/python3 gripper_server.py
```

4. Isaac Sim(Optional, for GraspGen only):
```bash
cd ~/ITRI-GraspGen/isaac-sim2real && \
omni_python sync_with_ROS2.py
```

5. GraspGen Server(Optional, for GraspGen only):
```bash
cd ~/ITRI-GraspGen && \
uv run scripts/mia_server.py --no-confirm
```

6. Open Main Window:
```bash
cd ~/RobotSnackServing && \
uv run MainWindow_app.py
```

7. modify UI
```bash
cd RobotSnackServing/
conda activate isaac-gr00t-ros2
pyuic5 -x MainWindow_ui.ui -o MainWindow_ui.py
```
- change MainWindow_ui.py "from PySide6 "
