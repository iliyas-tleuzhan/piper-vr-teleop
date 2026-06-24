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

## 6. Install the Quest APK

Place the APK at:

```text
third_party/APK/teleop-debug.apk
```

Connect the Quest with USB-C, put on the headset, and accept USB debugging. Then run:

```bash
scripts/install_quest_apk.sh
```

## 7. Verify ADB

```bash
adb devices
```

Expected result:

```text
List of devices attached
<device_serial>    device
```

If it says `unauthorized`, put on the headset and accept USB debugging.

## 8. Set up CAN

Connect the CAN adapter and bring it up:

```bash
scripts/setup_can.sh can0 1000000
ip -details link show can0
```

## 9. Verify Piper feedback

Power the Piper, check emergency stop status, and run:

```bash
python3 scripts/print_piper_pose.py --can can0
```

## 10. Run dry-run

```bash
python3 -m piper_vr.movep_teleop --config configs/single_piper.yaml --dry-run
```

Press `A` to calibrate and hold `B` to allow target updates. Dry-run prints endpoint commands instead of moving the robot.

## 11. Run real teleop

Only continue after dry-run looks correct:

```bash
python3 -m piper_vr.movep_teleop --config configs/single_piper.yaml
```

Keep the robot workspace clear. Release the deadman to hold position.
