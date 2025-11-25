# RobotSnackServing

## Steps:
1. Connect to TM_Driver:
- Make sure ethernet setting is using fixed **IPv4: 192.168.1.3**, and
```bash
ros2 run tm_driver tm_driver robot_ip:=192.168.1.10
```

2. gripper_server:
```bash
cd ~/ITRI-GraspGen/ROS2_server && \
/usr/bin/python3 gripper_server.py
```

3. Isaac Sim(Optional, for GraspGen only):
```bash
cd ~/ITRI-GraspGen/isaac-sim2real && \
omni_python sync_with_ROS2.py
```

4. GraspGen Server(Optional, for GraspGen only)(Not finished yet):
```bash
cd ~/ITRI-GraspGen && \
uv run <GraspGen Server>
```

5. Open Main Window:
```bash
cd ~/RobotSnackServing && \
conda activate isaac-groot-ros2 && \
python MainWindow_app.py
```