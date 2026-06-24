# CAN Setup

Piper commonly uses a 1 Mbps CAN bus.

Bring up a single adapter:

```bash
scripts/setup_can.sh can0 1000000
```

Bring up two adapters:

```bash
scripts/setup_dual_can.sh can0 can1 1000000
```

Check interface details:

```bash
ip -details link show can0
```

Monitor traffic:

```bash
candump can0
```

If adapter names change after reboot, use `ip link` and `dmesg` to identify the device, then update the config.
