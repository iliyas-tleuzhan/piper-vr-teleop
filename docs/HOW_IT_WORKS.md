# How It Works

The default path is:

```text
Quest APK -> ADB/logcat -> oculus_reader -> QuestSample -> TeleopSession -> SafetyLimiter -> PiperDriver -> EndPoseCtrl
```

`QuestSample` keeps source-native OpenXR-like 4x4 controller matrices and button maps. Axis mapping happens later through `piper_vr/vr_mapping.py`, using the YAML `axis_mapping` rules.

`TeleopSession` owns the state machine:

- `WAITING_FOR_DEVICE`
- `WAITING_FOR_CALIBRATION`
- `READY_IDLE`
- `ACTIVE`
- `HOLDING`
- `FAULT`

Calibration snapshots the current controller transform and the measured Piper endpoint pose. Motion is disarmed immediately after calibration, so the operator must release and re-press the deadman. Every new deadman press creates a fresh clutch anchor.

The Piper driver uses meters and degrees internally. It converts only at the SDK boundary:

- XYZ: `0.001 mm`
- RX/RY/RZ: `0.001 degrees`

The optional ROS transport is a placeholder for future ROS-topic Quest apps. WebRTC/browser XR stacks are better suited to richer telepresence systems and are not the primary first-working path here.
