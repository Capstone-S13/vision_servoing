# vision_servoing
Packages used for the AGV to dock towards a AprilTag.

# Launch
```
# camera node to publish to the image topics
roslaunch realsense2_camera rs_camera.launch

# AprilTag detection node
roslaunch apriltag_ros continuous_detection.launch

# ROS action server for vision servoing
rosrun vs_action vs_action_server.py
```

# Calling the Action

```
uint32 tag_id
---
DockState dock_state
bool success
---
DockState dock_state
bool success
```

The Vision Servoing action can be called by sending the action goal, with the AprilTag id (`uint32 tag_id`) to dock to.
