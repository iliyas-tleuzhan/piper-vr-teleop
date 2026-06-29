# Piper VR Teleop

Piper VR Teleop controls an AgileX Piper arm from a Meta Quest 3 controller. The maintained first-working path is intentionally direct:

```text
Quest controller pose -> safety/clutch mapping -> Piper EndPoseCtrl -> Piper firmware endpoint IK
```

The robot side preserves the Piper SDK endpoint sequence:

```python
C_PiperInterface_V2(can)
arm.ConnectPort()
arm.EnableArm(7, 0x02)
arm.ModeCtrl(0x01, 0x00, speed_percent, 0x00)
arm.EndPoseCtrl(X, Y, Z, RX, RY, RZ)
```

No host-side Jacobian IK solver is used in the default teleop path. The optional URDF guard is off by default.

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
```

Install the Quest reader:

```bash
scripts/install_oculus_reader.sh
```

If direct install fails, use the legacy fallback:

```bash
cd ~
git clone https://github.com/agilexrobotics/questVR_ws.git
export PYTHONPATH=~/questVR_ws/src/oculus_reader/scripts:$PYTHONPATH
python3 -c "import oculus_reader; print('oculus_reader ok')"
```

Install the Quest teleop APK:

```bash
mkdir -p third_party/APK
# Place teleop-debug.apk at third_party/APK/teleop-debug.apk
scripts/install_quest_apk.sh
```

Connect the Quest by USB, put on the headset, and accept USB debugging.

## Required Test Order

```bash
scripts/setup_can.sh can0 1000000
python3 scripts/test_piper_endpoint.py --can can0 --speed-percent 5 --dz 0.02
python3 scripts/check_quest_transport.py --transport adb_logcat --seconds 10
python3 -m piper_vr.movep_teleop --config configs/single_piper.yaml --dry-run --verbose
python3 -m piper_vr.movep_teleop --config configs/single_piper.yaml --can can0 --speed-percent 5 --scale 0.40 --max-speed 0.05 --verbose
```

`can0` is only an example. Use any interface name your system provides, including `can1` or `can_piper`.

## Controls

- Right controller controls the single Piper by default.
- `A` calibrates the current controller pose to the measured Piper endpoint pose.
- After calibration, release and press `rightGrip` before motion starts.
- Each new deadman press creates a clutch anchor from the measured robot pose.
- Releasing the deadman holds at the measured current endpoint pose.
- Right trigger controls the gripper only when `gripper_enabled: true`.

## Safety Defaults

- No motion before calibration.
- No motion without the deadman.
- Stale Quest samples hold position and require a deadman re-press.
- Workspace clamp, speed limit, max jump limit, deadband, and smoothing are enabled.
- Orientation control is off by default.
- URDF guarding is off by default; enable it only with `--urdf-guard`.

## Quest Transport

ADB/logcat plus `oculus_reader` is the primary transport because it matches the existing Quest APK workflow and keeps first motion simple. Wireless ADB is supported by passing `--quest-ip <ip>` after pairing the headset. ROS topics and WebRTC are documented as future integration paths, not the default route for first Piper motion.

## Useful Commands

```bash
python3 scripts/check_quest_transport.py --transport adb_logcat --seconds 10
scripts/check_adb_logcat.sh
python3 scripts/print_vr_xyz.py --seconds 10
python3 scripts/print_piper_pose.py --can can0
python3 -m piper_vr.dual_movep_teleop --config configs/dual_piper.yaml --dry-run
```

## Documentation

- [New laptop setup](docs/NEW_LAPTOP_SETUP.md)
- [Quest setup](docs/QUEST_SETUP.md)
- [CAN setup](docs/CAN_SETUP.md)
- [How it works](docs/HOW_IT_WORKS.md)
- [Axis mapping](docs/AXIS_MAPPING.md)
- [Troubleshooting](docs/TROUBLESHOOTING.md)
- [Upstream notes](docs/UPSTREAM_NOTES.md)
