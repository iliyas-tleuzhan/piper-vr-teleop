# How It Works

The mode is selected by `control_mode`:

- `joint_mimic`: default. Infer a human arm posture and command all six Piper joints with `JointCtrl`.
- `endpoint_firmware`: fallback/debug. Move the endpoint with `EndPoseCtrl`; Piper firmware chooses the joint posture.
- `external_ik`: optional host-side IK. Solve joint angles from an endpoint target plus posture objective, then send `JointCtrl`.

## Joint Mimic Path

```text
QuestSample -> JointMimicSession -> HumanArmState -> human_arm_to_piper_joints -> JointCtrl
```

The Quest controller gives hand pose. A Quest controller alone does not directly measure the operator shoulder or elbow, so `piper_vr/human_arm_model.py` estimates them:

- HMD/head pose can estimate a torso/shoulder frame when available.
- The default fallback stores a fixed shoulder position from calibration.
- Elbow position is solved from shoulder-to-wrist geometry, arm lengths, previous elbow state, and elbow swivel.
- Wrist orientation comes from controller orientation.

The resulting `HumanArmState` is mapped to six Piper joint angles through configurable neutral positions, signs, gains, offsets, smoothing, and per-joint speed limits.

## Endpoint Firmware Path

```text
QuestSample -> TeleopSession -> SafetyLimiter -> EndPoseCtrl -> Piper firmware IK
```

Endpoint control is useful for simple pick/place and debugging. It is not true whole-arm mimicry because the host only asks for a gripper pose and Piper firmware can choose any valid internal joint configuration.

## State Machine

Both runtime paths use the same state names:

- `WAITING_FOR_DEVICE`
- `WAITING_FOR_CALIBRATION`
- `READY_IDLE`
- `ACTIVE`
- `HOLDING`
- `FAULT`

Calibration records the controller home, shoulder estimate, and measured robot pose. Motion is disarmed immediately after calibration, so the operator must release and re-press the deadman.

## Units

The driver uses meters and degrees internally. It converts only at the SDK boundary:

- Endpoint XYZ: `0.001 mm`
- Endpoint RPY: `0.001 degrees`
- Joint angles: `0.001 degrees`
