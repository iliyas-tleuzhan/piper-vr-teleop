# Quest Piper Visualization App

This companion app visualizes the AgileX Piper arm on Meta Quest 3. It receives the same commanded joint targets that Python sends through `JointCtrl`, plus measured joint feedback when available. The Quest app never commands the robot.

## Prepare URDF Assets

```bash
cd ~/Iliyas/piper-vr-teleop
python3 scripts/prepare_piper_urdf_assets.py
python3 scripts/convert_piper_urdf_to_unity_json.py
```

The scripts expect the official Piper URDF at:

```text
third_party/agx_arm_urdf/piper/urdf/piper_description.urdf
```

Meshes are copied into `quest_viz_app/Assets/Models/Piper/`. The generated kinematic model is written to `quest_viz_app/Assets/StreamingAssets/piper_kinematic_model.json`.

## Open Unity

1. Install Unity 2022.3 LTS or newer LTS with Android Build Support.
2. Open `quest_viz_app/` as the Unity project.
3. Install/enable XR Plug-in Management and OpenXR for Android if Unity prompts.
4. Enable Meta Quest/OpenXR support for Android.
5. Run `Piper VR/Create Piper Viz Scene` from the Unity menu if the scene needs to be regenerated.
6. Open `Assets/Scenes/PiperVizScene.unity`.

For Quest 3 builds, switch the build target to Android, use ARM64, and build an APK from Unity Build Settings.

## Test UDP Locally

Start the desktop receiver:

```bash
python3 scripts/debug_viz_receiver.py --port 5055
```

Run teleop in dry-run mode and broadcast to localhost:

```bash
python3 -m piper_vr.vr_teleop --dry-run --viz --viz-host 127.0.0.1
```

The receiver should print JSON packets with `commanded_joints_deg`, `measured_joints_deg`, `controller_xyz`, `state`, and `reason`.

## Run With Quest

Find the Quest IP address:

```bash
adb shell ip addr show wlan0
```

Build and launch the Unity APK on the Quest, then start teleop:

```bash
python3 -m piper_vr.vr_teleop --viz --viz-host <QUEST_IP>
```

The default UDP port is `5055`. Use `--viz-port 5055` or config if you need a different port.

## Config

```yaml
viz:
  enabled: false
  host: "127.0.0.1"
  port: 5055
```

CLI flags override config:

```bash
python3 -m piper_vr.vr_teleop --dry-run --viz --viz-host 127.0.0.1
```

## In-App Status

The VR HUD shows:

- `WAITING FOR TELEOP DATA`
- `CALIBRATE: Press A`
- `HOLD RIGHTGRIP TO MOVE`
- `ACTIVE`
- `FAULT: <reason>`
- stale data warning after 0.5 seconds without packets
- commanded joints, measured joints, and tracking error

## Troubleshooting

- No packets locally: run `python3 scripts/debug_viz_receiver.py --port 5055` before teleop and confirm `--viz` is set.
- No packets on Quest: confirm the Quest and host are on the same Wi-Fi network, check `<QUEST_IP>`, and make sure firewalls allow UDP 5055.
- Model appears as simple blocks: Unity did not import or assign Piper mesh assets. The app intentionally falls back to labeled simple geometry; rerun asset preparation and confirm files exist under `quest_viz_app/Assets/Models/Piper/`.
- Robot moves but visualization does not: visualization is passive and best-effort; teleop continues even if the Quest app is not listening.
