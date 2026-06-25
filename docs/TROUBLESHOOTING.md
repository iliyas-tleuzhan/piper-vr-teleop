# Troubleshooting

## `No module named piper_sdk`

Install the SDK in the active environment:

```bash
pip install piper-sdk
```

## `No module named oculus_reader`

This repo expects:

```python
import oculus_reader
```

Clone AgileX's upstream workspace:

```bash
cd ~
git clone https://github.com/agilexrobotics/questVR_ws.git
```

Find the module:

```bash
find ~/questVR_ws -type f | grep -i oculus
```

Add the likely scripts folder to `PYTHONPATH`:

```bash
export PYTHONPATH=~/questVR_ws/src/oculus_reader/scripts:$PYTHONPATH
```

Then test:

```bash
python3 -c "import oculus_reader; print('oculus_reader ok')"
```

You can also run `scripts/setup_upstream_oculus_reader.sh` to print the expected `PYTHONPATH` export.

## `adb devices` shows no device

Check the cable, use a data-capable USB-C port, unlock the headset, and accept USB debugging inside the headset.

## Quest asks for USB debugging repeatedly

Use a stable USB cable, avoid USB hubs, and check `adb kill-server && adb start-server`.

## APK not installed

Place the APK at:

```text
third_party/APK/teleop-debug.apk
```

Then run:

```bash
scripts/install_quest_apk.sh
```

## Quest controller data not changing

Wake the headset, keep the controller visible to tracking, restart the APK, and run:

```bash
python3 scripts/print_vr_data.py
```

## Piper CAN not found

Check adapter names:

```bash
ip link
dmesg | tail
```

Then bring up the correct interface:

```bash
scripts/setup_can.sh can0 1000000
```

## Piper does not move

Confirm that real mode is running, the robot is enabled, emergency stop is released, calibration has been completed, and the deadman is held.

Before real robot mode, run dry-run first and verify Piper feedback:

```bash
scripts/setup_can.sh can0 1000000
python3 scripts/print_piper_pose.py --can can0
python3 scripts/test_piper_endpoint.py --can can0
python3 -m piper_vr.movep_teleop --config configs/single_piper.yaml --dry-run --verbose
```

Start real mode with slow values:

```bash
python3 -m piper_vr.movep_teleop \
  --config configs/single_piper.yaml \
  --can can0 \
  --speed-percent 5 \
  --scale 0.40 \
  --max-speed 0.05 \
  --verbose
```

Required real-robot test order:

```bash
scripts/setup_can.sh can0 1000000
python3 scripts/test_piper_endpoint.py --can can0
python3 -m piper_vr.movep_teleop --config configs/single_piper.yaml --dry-run --verbose
python3 -m piper_vr.movep_teleop --config configs/single_piper.yaml --can can0 --speed-percent 5 --scale 0.40 --max-speed 0.05 --verbose
```

## Piper connects but does not move

Check that `piper_vr/piper_driver.py` uses:

```python
arm.ConnectPort()
arm.EnableArm(7, 0x02)
arm.ModeCtrl(0x01, 0x00, speed_percent, 0x00)
arm.EndPoseCtrl(...)
```

`ModeCtrl` must use `move_mode=0x00` for MOVE P endpoint control. If `move_mode=0x01`, the arm may be in joint mode and endpoint commands may not behave as expected.

`ConnectPort()` must not receive the CAN name. Pass the CAN name to `C_PiperInterface_V2(can_name)` instead.

If raw `piper_sdk` movement works but `PiperDriver` does not, the wrapper must repeat enable and MOVE P mode setup with delays:

```python
for _ in range(5):
    arm.EnableArm(7, 0x02)
    time.sleep(0.2)

for _ in range(5):
    arm.ModeCtrl(0x01, 0x00, speed_percent, 0x00)
    time.sleep(0.2)
```

If `scripts/test_piper_endpoint.py --can can0` does not move, do not debug VR yet. Fix Piper/CAN/driver setup first.

If `scripts/test_piper_endpoint.py --can can0` moves but VR teleop does not, run with `--verbose` and check the deadman state, raw target, and safe target:

```bash
python3 -m piper_vr.movep_teleop --config configs/single_piper.yaml --dry-run --verbose
python3 -m piper_vr.movep_teleop --config configs/single_piper.yaml --can can0 --speed-percent 5 --scale 0.40 --max-speed 0.05 --verbose
```

## Piper moves in wrong direction

Edit `axis_mapping` in the config. See `docs/AXIS_MAPPING.md`.

## Piper moves too fast

Lower these config values:

```yaml
scale: 0.20
max_speed_m_s: 0.04
speed_percent: 5
```

## Piper jumps after calibration

Do not hold the deadman during calibration. Calibrate while the controller is steady. Lower `max_speed_m_s`.

## Piper internal IK refuses target

The endpoint may be outside the reachable workspace or orientation limits. Reduce workspace bounds and use `hold_orientation: true`.

## Gripper reversed

Invert the trigger mapping in `piper_vr/movep_teleop.py` or tune the gripper command range for your hardware.

## Dual CAN adapter names change after reboot

Use stable udev rules based on adapter serial numbers, or check `ip link` before each run and update `configs/dual_piper.yaml`.
