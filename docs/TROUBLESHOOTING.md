# Troubleshooting

## No module named oculus_reader

Run:

```bash
scripts/install_oculus_reader.sh
```

Fallback:

```bash
export PYTHONPATH=~/questVR_ws/src/oculus_reader/scripts:$PYTHONPATH
```

## adb devices shows unauthorized

Put on the headset and accept USB debugging. Unplug and reconnect USB if the prompt is missing.

## Quest app installed but no samples received

Run:

```bash
scripts/check_adb_logcat.sh
python3 scripts/check_quest_transport.py --transport adb_logcat --seconds 10
```

The diagnostic tag is `wE9ryARX`; the package is `com.rail.oculus.teleop`.

## Stale tracking or headset asleep

Wake the headset, keep the controller visible, and re-press the deadman after tracking returns. Stale samples intentionally hold position.

## Endpoint test moves but teleop does not

Check that you pressed `A` to calibrate, released `rightGrip`, then pressed it again. Run teleop with `--verbose` and inspect state, sample age, and command action.

## test_piper_endpoint.py does not move

Check CAN bitrate, power, emergency stop, `piper-sdk`, and the CAN interface name.

## Wrong CAN interface

Use the actual interface name:

```bash
ip link
python3 scripts/test_piper_endpoint.py --can can1 --speed-percent 5
```

## Motion too fast or wrong axis direction

Lower `scale`, `max_speed_m_s`, or `speed_percent`. Edit `axis_mapping` in `configs/single_piper.yaml` for direction changes.

## URDF guard blocks motion

The URDF guard is off by default. If enabled with `--urdf-guard`, disable it with:

```bash
python3 -m piper_vr.movep_teleop --config configs/single_piper.yaml --no-urdf-guard --verbose
```
