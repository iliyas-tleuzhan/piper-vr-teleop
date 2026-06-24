# ROS Notes

The maintained first path is the direct Python CLI:

```bash
python3 -m piper_vr.movep_teleop --config configs/single_piper.yaml --dry-run
```

The launch files in this folder are lightweight placeholders for users who want to wrap the CLI in a ROS/catkin workspace. If you use ROS Noetic, place this repository inside a catkin package or add a package wrapper that exposes the Python entry points.
