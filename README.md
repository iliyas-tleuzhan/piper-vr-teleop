# Piper VR Teleop

Piper VR Teleop is a clean English reference project for controlling an AgileX Piper arm with a Meta Quest 3 controller. The Quest controller acts like a 3D target dot: after calibration, moving the controller moves the Piper end effector through Piper endpoint control.

The first working path uses Piper built-in endpoint IK through:

```python
arm.EndPoseCtrl(X, Y, Z, RX, RY, RZ)
```

No custom Jacobian IK solver is used in the main control loop.

## Hardware

- AgileX Piper arm
- CAN adapter
- Meta Quest 3 headset and controller
- USB-C cable for the first wired setup
- Ubuntu 20.04 or 22.04 laptop

## Software

- Python 3.9 recommended
- `android-tools-adb`
- `piper-sdk`
- `pure-python-adb`
- `numpy`
- `pyyaml`
- Optional ROS Noetic and catkin for users who want a ROS launch path
- Optional `pinocchio`, `casadi`, `meshcat`, and `rospkg` if matching parts of the upstream research environment

## Quick Start

```bash
git clone <your-repo-url> piper-vr-teleop
cd piper-vr-teleop
conda env create -f environment.yml
conda activate piper-vr
pip install -r requirements.txt
sudo apt update
sudo apt install android-tools-adb can-utils
```

Install the Quest teleop APK:

```bash
mkdir -p third_party/APK
# Place teleop-debug.apk at third_party/APK/teleop-debug.apk
scripts/install_quest_apk.sh
```

Connect the Quest by USB, put on the headset, and accept USB debugging.

Bring up CAN:

```bash
scripts/setup_can.sh can0 1000000
```

Test VR data:

```bash
python3 scripts/print_vr_data.py
```

Test Piper feedback:

```bash
python3 scripts/print_piper_pose.py --can can0
```

Run dry-run teleop first:

```bash
python3 -m piper_vr.movep_teleop --config configs/single_piper.yaml --dry-run
```

Run real teleop only after dry-run behaves correctly:

```bash
python3 -m piper_vr.movep_teleop --config configs/single_piper.yaml
```

## Controls

- Right controller controls the single Piper by default.
- `A` calibrates the VR home pose to the current Piper end-effector pose.
- Hold `B` as the deadman switch by default.
- Releasing the deadman holds the last command.
- Right trigger can control the gripper when `gripper_enabled: true`.
- `Ctrl+C` exits cleanly and sends a hold command.

All controls are configurable in [configs/single_piper.yaml](configs/single_piper.yaml).

## Safety Rules

- The robot never moves before calibration.
- The robot never moves unless the deadman is held.
- Workspace limits clamp the target position.
- Cartesian speed limiting prevents large endpoint steps.
- Tracking loss or stale Quest data causes a hold.
- Real robot mode prints a warning before connecting.
- Start with dry-run and low speed.

## Command Path

```text
Quest controller pose
-> relative target mapping
-> workspace and speed safety
-> Piper EndPoseCtrl
-> Piper internal IK
-> joint movement
```

Internally, this project uses meters and degrees for readability. Piper endpoint command units are converted at the hardware boundary:

- XYZ: `0.001 mm`
- RX/RY/RZ: `0.001 degrees`

## Axis Mapping

If moving the controller in one direction moves Piper in the wrong direction, edit:

```yaml
axis_mapping:
  piper_x: "+vr_x"
  piper_y: "+vr_y"
  piper_z: "+vr_z"
```

See [docs/AXIS_MAPPING.md](docs/AXIS_MAPPING.md).

## Quest APK

This repository does not commit the upstream APK binary. Place it here:

```text
third_party/APK/teleop-debug.apk
```

See [docs/QUEST_SETUP.md](docs/QUEST_SETUP.md).

## Documentation

- [New laptop setup](docs/NEW_LAPTOP_SETUP.md)
- [Quest setup](docs/QUEST_SETUP.md)
- [CAN setup](docs/CAN_SETUP.md)
- [How it works](docs/HOW_IT_WORKS.md)
- [Axis mapping](docs/AXIS_MAPPING.md)
- [Troubleshooting](docs/TROUBLESHOOTING.md)
- [Upstream notes](docs/UPSTREAM_NOTES.md)

## Upstream Reference

This repository is based on ideas and architecture from AgileX Robotics `questVR_ws`, including the `oculus_reader` approach, Quest APK workflow, Piper ROS files, and single and dual Piper teleoperation examples. The implementation here is written as a clean English project focused on a safe first working endpoint-control path.
