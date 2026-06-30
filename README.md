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

`quest_endpoint_ik` is the more natural Cartesian mode. It is closest to AgileX's hand-gesture demo: calibrate a controller home frame, move a tracked hand/controller relative to that frame, map that relative motion to an end-effector target, then solve IK for all six Piper joints. This project uses Quest 3 controller 6DoF tracking instead of camera, MediaPipe, and depth alignment.

Joint mimic is calibration-relative. Pressing `A` only calibrates; every new `rightGrip` press creates a new clutch anchor. If you move the controller while the deadman is released, the robot should not jump when you grip again.

The default `joint_mimic` mapping is now six-joint `relative_delta`: each frame uses small controller motion in a calibrated HMD-yaw/control frame to increment the robot joint target. Controller translation drives joints 1-3, and controller rotation drives wrist joints 4-6 while `rightGrip` is held. `rightTrig` is not required for wrist motion by default.

## Why the old movement felt wrong

- Raw controller Euler angles caused unexpected wrist twisting.
- Absolute pose-delta mapping made horizontal/forward directions depend too much on poorly calibrated frames.
- Rate limiting could leave a target backlog, so Piper kept moving after the controller stopped.
- The default path now uses relative controller deltas, deadband, filtering, small wrist gains, backlog cancellation on stop, and main-grip clutching.

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
git pull origin main
export PYTHONPATH=$PWD:$HOME/Iliyas/questVR_ws/src/oculus_reader/scripts:$PYTHONPATH

python3 scripts/check_quest_transport.py --seconds 10
python3 scripts/record_quest_axis_movements.py --side right
python3 scripts/generate_relative_gain_config.py
python3 scripts/predict_piper_motion_from_controller.py --config configs/generated_relative_mapping.yaml

scripts/setup_can.sh can0 1000000
python3 scripts/print_piper_joints.py --can can0 --debug-feedback
python3 scripts/test_piper_joint.py --can can0 --joint 2 --delta-deg 5 --duration 2 --rate 50

python3 -m piper_vr.vr_teleop --config configs/generated_relative_mapping.yaml --debug-motion
```

`can0` is only an example. Use the CAN interface name your system provides.

## Quest Endpoint IK Mode

Endpoint IK mode is selected with `control_mode: "quest_endpoint_ik"` or `--endpoint-ik`. In this mode:

- `A` calibrates the current Quest controller pose and current Piper FK end-effector pose.
- Holding `rightGrip` maps controller translation to target end-effector XYZ.
- Holding `rightGrip` maps controller rotation to target end-effector orientation.
- The host solves IK from the Piper URDF and sends all six joints with `JointCtrl`.
- Releasing `rightGrip` holds/stops the whole arm.

Real endpoint IK test sequence:

```bash
cd ~/Iliyas/piper-vr-teleop
git pull origin main
git submodule update --init --recursive || true
export PYTHONPATH=$PWD:$HOME/Iliyas/questVR_ws/src/oculus_reader/scripts:$PYTHONPATH

python3 -m compileall piper_vr scripts tests
pytest -q

python3 scripts/check_quest_transport.py --seconds 10
python3 scripts/calibrate_quest_endpoint_frame.py --side right
python3 scripts/predict_endpoint_ik_from_controller.py --config configs/generated_endpoint_ik_mapping.yaml

scripts/setup_can.sh can0 1000000
python3 scripts/print_piper_joints.py --can can0 --debug-feedback

# First, firmware endpoint dry-run / target test:
python3 scripts/test_firmware_endpoint_from_quest.py --config configs/generated_endpoint_ik_mapping.yaml

# Then with robot, no send:
python3 scripts/test_firmware_endpoint_from_quest.py --robot --can can0 --config configs/generated_endpoint_ik_mapping.yaml

# Then real low-scale firmware endpoint:
python3 scripts/test_firmware_endpoint_from_quest.py --robot --send --can can0 --config configs/generated_endpoint_ik_mapping.yaml --scale 0.3

# Real teleop, safest first:
python3 -m piper_vr.vr_teleop \
  --config configs/single_piper.yaml \
  --mapping-config configs/generated_endpoint_ik_mapping.yaml \
  --endpoint-ik \
  --ik-backend firmware_endpoint \
  --profile safe \
  --ik-scale 0.3 \
  --position-only \
  --debug-ik \
  --no-log
```

### Endpoint IK Direction Tuning

Use these commands whenever endpoint directions feel inverted, weak, or clamped:

```bash
python3 scripts/calibrate_quest_endpoint_frame.py --side right
python3 scripts/verify_endpoint_directions.py --config configs/generated_endpoint_ik_mapping.yaml
python3 scripts/test_firmware_endpoint_from_quest.py --config configs/generated_endpoint_ik_mapping.yaml
```

FK backend notes:

- `firmware_endpoint` is recommended for first real robot testing because Piper firmware handles endpoint IK.
- `host_ik_sdk_fk` uses official Piper SDK FK when available.
- If official SDK FK is unavailable, the local fallback is approximate and mainly for dry-run/testing.
- `host_ik_urdf` requires the AgileX URDF submodule.

- If forward/back is inverted, rerun calibration and check the generated `quest_endpoint_ik.axis_mapping.robot_x` sign.
- If left/right is slow, increase `quest_endpoint_ik.scale_xyz[1]`.
- If forward/back barely moves, check `quest_endpoint_ik.scale_xyz[0]`, `max_delta_from_home_m`, and the `--debug-ik` clamped axes.
- If only some axes move, run teleop with `--debug-ik` and inspect `target_before_home_clamp`, `target_after_home_clamp`, `target_after_workspace_clamp`, and `clamped_axes`.

After direction fixes, a faster lateral firmware endpoint test can use:

```bash
python3 -m piper_vr.vr_teleop \
  --config configs/single_piper.yaml \
  --mapping-config configs/generated_endpoint_ik_mapping.yaml \
  --endpoint-ik \
  --ik-backend firmware_endpoint \
  --profile safe \
  --endpoint-speed-percent 25 \
  --ik-scale 0.5 \
  --position-only \
  --debug-ik \
  --no-log
```

## Controls

- Right controller controls the single Piper by default.
- `A` calibrates the current human-arm vector to the measured Piper joint pose.
- After calibration, release and press `rightGrip` before motion starts.
- Hold `rightGrip` to control all six joints: translation controls joints 1-3, rotation controls joints 4-6.
- Each new deadman press creates a clutch anchor.
- Releasing the deadman holds the measured current joint pose in `joint_mimic`.
- Right joystick X adjusts elbow swivel when configured as `rightJS_x`.
- Right trigger is not required for wrist rotation. It controls the gripper only when `gripper_enabled: true`.

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
python3 scripts/record_quest_axis_movements.py --side right
python3 scripts/generate_relative_gain_config.py
python3 scripts/predict_piper_motion_from_controller.py --config configs/generated_relative_mapping.yaml
python3 scripts/test_controller_rotation_to_wrist.py
python3 -m piper_vr.vr_teleop --profile safe
python3 -m piper_vr.vr_teleop --profile normal
python3 -m piper_vr.vr_teleop --profile fast
python3 -m piper_vr.vr_teleop --config configs/single_piper.yaml --mapping-config configs/generated_relative_mapping.yaml --debug-motion
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

## Quest 3 Piper Visualization App

`quest_viz_app/` is a companion Unity/OpenXR project for Meta Quest 3. It visualizes a Piper arm from the official Piper URDF/meshes and animates commanded and measured six-joint state sent by the Python teleop loop over UDP. It is separate from robot control and never commands the real arm.

Quickstart:

```bash
python3 scripts/prepare_piper_urdf_assets.py
python3 scripts/convert_piper_urdf_to_unity_json.py
python3 scripts/debug_viz_receiver.py --port 5055
python3 -m piper_vr.vr_teleop --dry-run --viz --viz-host 127.0.0.1
```

For Quest:

```bash
adb shell ip addr show wlan0
python3 -m piper_vr.vr_teleop --viz --viz-host <QUEST_IP>
```

See [Quest Piper visualization app](docs/QUEST_PIPER_VIZ_APP.md) for Unity setup, APK build steps, and UDP troubleshooting.

## Documentation

- [Joint mimic teleop](docs/JOINT_MIMIC_TELEOP.md)
- [Quest Piper visualization app](docs/QUEST_PIPER_VIZ_APP.md)
- [How it works](docs/HOW_IT_WORKS.md)
- [Axis mapping](docs/AXIS_MAPPING.md)
- [Troubleshooting](docs/TROUBLESHOOTING.md)
- [Quest setup](docs/QUEST_SETUP.md)
- [CAN setup](docs/CAN_SETUP.md)
