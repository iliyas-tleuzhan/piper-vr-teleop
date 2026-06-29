# CAN Setup

Bring up any SocketCAN interface name your system uses:

```bash
scripts/setup_can.sh can0 1000000
```

The interface does not have to be `can0`; `can1`, `can_piper`, and other names are valid. Piper ROS examples expose `can_port` as a string for the same reason. Multi-adapter systems may rename interfaces using USB bus information.

For two adapters:

```bash
scripts/setup_dual_can.sh can0 can1 1000000
```

Robot-side first test:

```bash
python3 scripts/test_piper_endpoint.py --can can0 --speed-percent 5 --dz 0.02
```

If the endpoint test does not move, fix CAN, power, SDK installation, or Piper enable state before debugging VR.
