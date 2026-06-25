# New Laptop Setup

These steps target Ubuntu 20.04 or 22.04.

## 1. Install system packages

```bash
sudo apt update
sudo apt install git android-tools-adb can-utils net-tools
```

Optional ROS Noetic setup is only needed if you want to use a catkin or ROS launch path.

## 2. Install Miniconda

Download Miniconda for Linux from the official Anaconda site, then install it:

```bash
bash Miniconda3-latest-Linux-x86_64.sh
```

Restart the terminal after installation.

## 3. Create the environment

```bash
git clone <your-repo-url> piper-vr-teleop
cd piper-vr-teleop
conda env create -f environment.yml
conda activate piper-vr
pip install -r requirements.txt
```

If you prefer `venv`:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## 4. Install Piper SDK

The main dependency is:

```bash
pip install piper-sdk
```

Verify import:

```bash
python3 -c "import piper_sdk; print('piper-sdk import ok')"
```

## 5. Set up Quest developer mode

Use the Meta mobile app and Meta developer account flow to enable developer mode for the headset. Reboot the headset after enabling it.

## 6. Install upstream oculus_reader

This project expects:

```python
import oculus_reader
```

Clone AgileX's upstream Quest workspace:

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

Verify import:

```bash
python3 -c "import oculus_reader; print('oculus_reader ok')"
```

You can also run:

```bash
scripts/setup_upstream_oculus_reader.sh
```

and copy the printed `PYTHONPATH` export into your shell config.

## 7. Install the Quest APK

Place the APK at:

```text
third_party/APK/teleop-debug.apk
```

Connect the Quest with USB-C, put on the headset, and accept USB debugging. Then run:

```bash
scripts/install_quest_apk.sh
```

## 8. Verify ADB

```bash
adb devices
```

Expected result:

```text
List of devices attached
<device_serial>    device
```

If it says `unauthorized`, put on the headset and accept USB debugging.

## 9. Set up CAN

Connect the CAN adapter and bring it up:

```bash
scripts/setup_can.sh can0 1000000
ip -details link show can0
```

## 10. Verify Piper driver setup

Before real robot mode, patch or verify [../piper_vr/piper_driver.py](../piper_vr/piper_driver.py) uses Piper SDK V2 endpoint control:

```python
arm.ConnectPort()
arm.EnableArm(7, 0x02)
arm.ModeCtrl(0x01, 0x00, speed_percent, 0x00)
arm.EndPoseCtrl(...)
```

`ConnectPort()` must not receive the CAN name. The CAN name belongs in `C_PiperInterface_V2(can_name)`.

## 11. Verify Piper feedback

Power the Piper, check emergency stop status, and run:

```bash
python3 scripts/print_piper_pose.py --can can0
```

## 12. Run dry-run

```bash
python3 -m piper_vr.movep_teleop --config configs/single_piper.yaml --dry-run
```

Press `A` to calibrate and hold `B` to allow target updates. Dry-run prints endpoint commands instead of moving the robot.

## 13. Run real teleop

Only continue after dry-run looks correct. Start with slow values:

```bash
python3 -m piper_vr.movep_teleop \
  --config configs/single_piper.yaml \
  --speed-percent 5 \
  --scale 0.20 \
  --max-speed 0.04
```

Keep the robot workspace clear. Release the deadman to hold position.
