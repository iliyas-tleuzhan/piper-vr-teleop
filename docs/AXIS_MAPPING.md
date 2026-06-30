# Axis Mapping

Endpoint axis mapping lives under `endpoint_firmware` in `configs/single_piper.yaml`:

```yaml
endpoint_firmware:
  axis_mapping:
    translation_frame: "controller_home"
    piper_x: "-vr_z"
    piper_y: "-vr_x"
    piper_z: "+vr_y"
```

Each output axis can read one Quest axis with a sign:

- `+vr_x`
- `-vr_x`
- `+vr_y`
- `-vr_y`
- `+vr_z`
- `-vr_z`

`translation_frame: controller_home` is the recommended default. It converts Quest room coordinates into the controller orientation present at calibration, so controller forward/right/up have stable meanings. Use `quest_world` only when you intentionally want fixed room-coordinate mapping.

## Controller forward makes Piper go backward

Flip the sign of the affected axis:

```yaml
piper_x: "-vr_x"
```

## Controller up makes Piper go down

Flip the vertical axis:

```yaml
piper_z: "-vr_z"
```

## Controller left makes Piper move sideways in the wrong axis

Swap axis assignments:

```yaml
axis_mapping:
  piper_x: "+vr_y"
  piper_y: "+vr_x"
  piper_z: "+vr_z"
```

Tune in dry-run first, then test at low speed with the robot workspace clear.

## Joint Mimic Axes

`joint_mimic` does not use endpoint XYZ axis mapping. It maps inferred human arm angles to Piper joints:

- joint 1: shoulder yaw/horizontal sweep
- joint 2: shoulder lift
- joint 3: elbow flexion into Piper's negative elbow range
- joint 4: wrist/forearm roll
- joint 5: wrist pitch
- joint 6: wrist yaw/roll depending on real Piper axis behavior

Tune these with:

```yaml
joint_mimic:
  neutral_deg: [0.0, 90.0, -90.0, 0.0, 0.0, 0.0]
  offsets_deg: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  signs: [1, 1, 1, 1, 1, 1]
  gains: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
```
