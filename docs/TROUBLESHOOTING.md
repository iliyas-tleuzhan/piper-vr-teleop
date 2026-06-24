# Troubleshooting

## `No module named piper_sdk`

Install the SDK in the active environment:

```bash
pip install piper-sdk
```

## `No module named oculus_reader`

Install or place the `oculus_reader` Python module on `PYTHONPATH`. This project expects a compatible module exposing `OculusReader` and `get_transformations_and_buttons()`.

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
