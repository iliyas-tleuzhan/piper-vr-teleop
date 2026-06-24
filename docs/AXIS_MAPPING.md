# Axis Mapping

Axis mapping lives in `configs/single_piper.yaml`:

```yaml
axis_mapping:
  piper_x: "+vr_x"
  piper_y: "+vr_y"
  piper_z: "+vr_z"
```

Each output axis can read one Quest axis with a sign:

- `+vr_x`
- `-vr_x`
- `+vr_y`
- `-vr_y`
- `+vr_z`
- `-vr_z`

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
