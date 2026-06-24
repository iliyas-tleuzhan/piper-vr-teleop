# Quest Setup

## Developer Mode

1. Create or use a Meta developer organization.
2. Enable developer mode for the Quest 3 in the Meta mobile app.
3. Reboot the headset.
4. Connect the headset to the laptop with USB-C.
5. Put on the headset and accept USB debugging.

## Unknown Sources

After installing the teleop APK, open it from the headset app library under unknown sources if needed.

## Display Timeout

Increase the headset sleep and display timeout while testing. If the headset sleeps, controller tracking and log streaming can stop.

## Install APK

Place the APK here:

```text
third_party/APK/teleop-debug.apk
```

Then run:

```bash
scripts/install_quest_apk.sh
```

## Verify USB ADB

```bash
adb devices
```

Expected state is `device`. If the state is `unauthorized`, accept the prompt inside the headset.

## Optional Wireless ADB

Start over USB:

```bash
scripts/enable_quest_wireless_adb.sh
```

Find the Quest IP address from Wi-Fi settings or:

```bash
adb shell ip route
```

Connect:

```bash
scripts/enable_quest_wireless_adb.sh <Quest_IP>
```

Then run teleop with:

```bash
python3 -m piper_vr.movep_teleop --config configs/single_piper.yaml --quest-ip <Quest_IP> --dry-run
```
