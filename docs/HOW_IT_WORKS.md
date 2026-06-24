# How It Works

The Quest APK streams controller transforms and button states through Android logs. The Python reader follows the `oculus_reader` style and calls:

```python
transformations, buttons = oculus_reader.get_transformations_and_buttons()
```

The teleop loop uses that data in four stages.

## 1. Read Quest State

`QuestReader` returns a 4x4 transform for the selected controller and a normalized button dictionary.

## 2. Calibrate Home

When the calibrate button is pressed, the software stores:

- Current Quest controller transform
- Current Piper endpoint position
- Current Piper endpoint orientation, unless a fixed orientation is configured

This makes future controller motion relative to a known robot pose.

## 3. Map Motion

Controller translation delta is scaled and mapped into Piper XYZ target motion. Axis signs and assignments are configured in YAML.

## 4. Apply Safety

The target is clamped to the configured workspace and limited by maximum Cartesian speed. If tracking is stale or the deadman is released, the command is held.

## 5. Command Piper Endpoint Control

The final target is converted from meters and degrees into Piper command units, then sent through:

```python
arm.EndPoseCtrl(X, Y, Z, RX, RY, RZ)
```

Piper internal IK converts the endpoint command to joint movement.

```text
Quest controller pose
-> relative target mapping
-> workspace and speed safety
-> Piper EndPoseCtrl
-> Piper internal IK
-> joint movement
```
