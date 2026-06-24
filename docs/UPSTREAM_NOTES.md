# Upstream Notes

This repository is based on and refactored from ideas in AgileX Robotics `questVR_ws`.

Relevant upstream areas include:

- `oculus_reader`, which reads Quest controller transforms and buttons.
- The Quest teleop APK workflow.
- Piper ROS files.
- Single Piper teleoperation launch and script structure.
- Dual Piper teleoperation launch and script structure.

The root of the upstream repository did not provide a broad repository license during inspection. For that reason, this project uses the upstream repository as a technical reference and provides a clean English implementation instead of copying large VR teleoperation source blocks.

The `Piper_ros` subfolder in the upstream repository has its own license. Review upstream licensing directly before copying any upstream files into this project.
