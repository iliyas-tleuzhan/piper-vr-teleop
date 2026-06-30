# Piper VR Teleop

Piper VR Teleop controls an AgileX Piper arm from a Meta Quest 3 controller. The primary mode is now joint-space mimic teleoperation:

```text
Quest controller + inferred human arm model -> six Piper joint targets -> JointCtrl
```

Endpoint control is still preserved as a fallback/debug mode:

```text
Quest controller pose -> safety/clutch mapping -> EndPoseCtrl -> Piper firmware endpoint IK
```

Use `joint_mimic` when the goal is whole-arm teleoperation. It is still approximate because the Quest controller does not directly measure the shoulder or elbow, but the final robot command is six simultaneous joint angles through `JointCtrl`, not firmware endpoint IK. Use `endpoint_firmware` when you only need to move the gripper endpoint and are comfortable letting Piper firmware choose the internal joint posture. `external_ik` is an optional host-side path for endpoint targets plus posture objectives; it sends `JointCtrl` results rather than relying on firmware endpoint IK.

Joint mimic is calibration-relative. Pressing `A` only calibrates; every new `rightGrip` press creates a new clutch anchor. If you move the controller while the deadman is released, the robot should not jump when you grip again.

## Hardware

- AgileX Piper arm
- USB-to-CAN adapter
- Meta Quest 3 headset and controller
- USB-C cable for first Quest setup
- Ubuntu 20.04 or 22.04 laptop

## Software Setup

```bash
git clone <your-repo-url> piper-vr-teleop
cd piper-vr-teleop
conda env create -f environment.yml
conda activate piper-vr
pip install -r requirements.txt
sudo apt update
sudo apt install android-tools-adb can-utils
scripts/install_oculus_reader.sh
```

Install the Quest teleop APK, connect the Quest by USB, put on the headset, and accept USB debugging.

## Required Test Order

```bash
cd ~/Iliyas/piper-vr-teleop
export PYTHONPATH=$PWD:$HOME/Iliyas/questVR_ws/src/oculus_reader/scripts:$PYTHONPATH

scripts/setup_can.sh can0 1000000

python3 scripts/test_piper_endpoint.py --can can0 --speed-percent 5 --dz 0.02
python3 scripts/inspect_piper_sdk_feedback.py --can can0
python3 scripts/print_piper_joints.py --can can0 --debug-feedback
python3 scripts/test_piper_joint.py --can can0 --joint 2 --delta-deg 3 --duration 3 --rate 50

python3 scripts/check_quest_transport.py --seconds 10
python3 scripts/debug_human_arm_model.py --side right
python3 scripts/debug_joint_mimic_mapping.py --side right --calibrate-button A

python3 scripts/run_dry.py
python3 -m piper_vr.vr_teleop
```

`can0` is only an example. Use the CAN interface name your system provides.

## Controls

- Right controller controls the single Piper by default.
- `A` calibrates the current human-arm vector to the measured Piper joint pose.
- After calibration, release and press `rightGrip` before motion starts.
- Each new deadman press creates a clutch anchor.
- Releasing the deadman holds the measured current joint pose in `joint_mimic`.
- Right joystick X adjusts elbow swivel when configured as `rightJS_x`.
- Right trigger controls the gripper only when `gripper_enabled: true`.

## Safety Defaults

- No motion before calibration.
- No motion without the deadman.
- Deadman release holds measured robot pose.
- Stale Quest samples hold measured robot pose and require a deadman re-press.
- Joint targets are clamped to documented Piper joint limits.
- Joint speeds are rate-limited per joint.
- Ctrl+C commands a clean hold.
- Real joint mimic refuses calibration when joint feedback is unavailable.
- If feedback later drops out, hold can fall back only to a real joint command already sent during this process.

## Useful Commands

```bash
python3 -m piper_vr.vr_teleop
python3 scripts/run_real.py
python3 scripts/run_dry.py
scripts/run_real.sh
scripts/run_dry.sh
python3 -m piper_vr.vr_teleop --can can1
python3 -m piper_vr.vr_teleop --max-joint-speed 10
python3 -m piper_vr.vr_teleop --quiet --no-log
python3 scripts/print_piper_joints.py --can can0
python3 scripts/debug_human_arm_model.py --side right --dry-run
python3 scripts/debug_joint_mimic_mapping.py --side right --calibrate-button A --dry-run
python3 scripts/tune_joint_mapping_vr.py --can can0 --dry-run
python3 -m piper_vr.movep_teleop --config configs/single_piper.yaml --control-mode endpoint_firmware --dry-run --verbose
python3 -m piper_vr.dual_movep_teleop --config configs/dual_piper.yaml --dry-run
```

## Documentation

- [Joint mimic teleop](docs/JOINT_MIMIC_TELEOP.md)
- [How it works](docs/HOW_IT_WORKS.md)
- [Axis mapping](docs/AXIS_MAPPING.md)
- [Troubleshooting](docs/TROUBLESHOOTING.md)
- [Quest setup](docs/QUEST_SETUP.md)
- [CAN setup](docs/CAN_SETUP.md)
