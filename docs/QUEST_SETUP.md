# Quest Setup

ADB/logcat plus `oculus_reader` is the primary Quest transport.

1. Enable developer mode for the Quest.
2. Connect the headset over USB.
3. Put on the headset and accept USB debugging.
4. Install the APK:

```bash
mkdir -p third_party/APK
# Place teleop-debug.apk at third_party/APK/teleop-debug.apk
scripts/install_quest_apk.sh
```

Install `oculus_reader`:

```bash
scripts/install_oculus_reader.sh
```

Legacy fallback:

```bash
cd ~
git clone https://github.com/agilexrobotics/questVR_ws.git
export PYTHONPATH=~/questVR_ws/src/oculus_reader/scripts:$PYTHONPATH
```

Check transport:

```bash
adb devices
scripts/check_adb_logcat.sh
python3 scripts/check_quest_transport.py --transport adb_logcat --seconds 10
```

For wireless ADB, first verify USB mode, then use the existing wireless helper and pass `--quest-ip <ip>` to diagnostics or teleop.
