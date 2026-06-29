# New Laptop Setup

Tested target platforms are Ubuntu 20.04 and 22.04.

```bash
sudo apt update
sudo apt install android-tools-adb can-utils git python3-pip
git clone <your-repo-url> piper-vr-teleop
cd piper-vr-teleop
python3 -m pip install -r requirements.txt
scripts/install_oculus_reader.sh
```

Then follow the required order:

```bash
scripts/setup_can.sh can0 1000000
python3 scripts/test_piper_endpoint.py --can can0 --speed-percent 5 --dz 0.02
python3 scripts/check_quest_transport.py --transport adb_logcat --seconds 10
python3 -m piper_vr.movep_teleop --config configs/single_piper.yaml --dry-run --verbose
python3 -m piper_vr.movep_teleop --config configs/single_piper.yaml --can can0 --speed-percent 5 --scale 0.40 --max-speed 0.05 --verbose
```
