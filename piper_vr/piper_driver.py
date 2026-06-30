"""Piper hardware driver wrapper for endpoint and joint-space control."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any

import numpy as np

from .joint_limits import clamp_joints_deg, degrees_to_piper_joint_units, piper_joint_units_to_degrees
from .units import degrees_to_piper_rpy, meters_to_gripper_units, meters_to_piper_xyz, piper_rpy_to_degrees, piper_xyz_to_meters


@dataclass
class EndPose:
    xyz_m: np.ndarray
    rpy_deg: np.ndarray


@dataclass
class JointPose:
    joints_deg: np.ndarray
    gripper_m: float | None = None


class PiperDriver:
    def __init__(self, can: str = "can0", speed_percent: int = 10, dry_run: bool = False) -> None:
        self.can = can
        self.speed_percent = int(speed_percent)
        self.dry_run = dry_run
        self.arm: Any | None = None
        self.last_pose = EndPose(np.array([0.35, 0.0, 0.25], dtype=float), np.zeros(3, dtype=float))
        self.last_command = EndPose(self.last_pose.xyz_m.copy(), self.last_pose.rpy_deg.copy())
        self.last_joint_command = JointPose(np.array([0.0, 90.0, -90.0, 0.0, 0.0, 0.0], dtype=float))
        self.has_sent_joint_command = False
        self.has_measured_joint_feedback = False
        self._joint_feedback_warnings: set[str] = set()

    def connect(self, initial_mode: str | None = None) -> None:
        if self.dry_run:
            print("[DRY-RUN] Piper connection skipped.")
            return

        self.arm = self._build_arm()

        # CAN name is already passed to C_PiperInterface_V2(self.can).
        if hasattr(self.arm, "ConnectPort"):
            self.arm.ConnectPort()
        elif hasattr(self.arm, "connect"):
            self.arm.connect()
        else:
            raise RuntimeError("No ConnectPort/connect method found in piper_sdk interface.")

        time.sleep(1.0)

        self.enable()
        if initial_mode == "endpoint":
            self.set_move_p_mode()
        elif initial_mode == "joint":
            self.set_move_j_mode()
        elif initial_mode is not None:
            raise ValueError("initial_mode must be 'endpoint', 'joint', or None")

        pose = self.read_end_pose()
        if pose is not None:
            self.last_pose = pose

    def _build_arm(self) -> Any:
        try:
            from piper_sdk import C_PiperInterface_V2
        except ImportError as exc:
            raise RuntimeError("piper-sdk is not installed. Install it with `pip install piper-sdk`.") from exc

        return C_PiperInterface_V2(self.can)

    def _call_first_available(self, names: tuple[str, ...], *args: Any) -> Any:
        if self.arm is None:
            return None
        for name in names:
            method = getattr(self.arm, name, None)
            if method is not None:
                return method(*args)
        return None

    def enable(self) -> None:
        if self.dry_run:
            return

        if self.arm is None:
            raise RuntimeError("Piper interface is not initialized.")

        if not hasattr(self.arm, "EnableArm"):
            raise RuntimeError("No EnableArm method found in piper_sdk interface.")

        print("[Piper] Enabling all motors...")
        for _ in range(5):
            # EnableArm(7, 0x02): 7 selects all motors, 0x02 enables them.
            self.arm.EnableArm(7, 0x02)
            time.sleep(0.2)

    def set_move_p_mode(self) -> None:
        if self.dry_run:
            return

        if self.arm is None:
            raise RuntimeError("Piper interface is not initialized.")

        print("[Piper] Setting MOVE P endpoint mode...")
        for _ in range(5):
            if hasattr(self.arm, "ModeCtrl"):
                # ModeCtrl(0x01, 0x00, speed, 0x00): CAN command control
                # plus MOVE P endpoint mode at the requested speed percentage.
                self.arm.ModeCtrl(0x01, 0x00, self.speed_percent, 0x00)
            elif hasattr(self.arm, "MotionCtrl_2"):
                self.arm.MotionCtrl_2(0x01, 0x00, self.speed_percent, 0x00)
            else:
                raise RuntimeError("No ModeCtrl/MotionCtrl_2 method found in piper_sdk interface.")
            time.sleep(0.2)

    def set_move_j_mode(self) -> None:
        if self.dry_run:
            return

        if self.arm is None:
            raise RuntimeError("Piper interface is not initialized.")

        print("[Piper] Setting MOVE J joint mode...")
        for _ in range(5):
            if hasattr(self.arm, "ModeCtrl"):
                # ModeCtrl(0x01, 0x01, speed, 0x00): CAN command control
                # plus MOVE J joint mode at the requested speed percentage.
                self.arm.ModeCtrl(0x01, 0x01, self.speed_percent, 0x00)
            elif hasattr(self.arm, "MotionCtrl_2"):
                self.arm.MotionCtrl_2(0x01, 0x01, self.speed_percent, 0x00)
            else:
                raise RuntimeError("No ModeCtrl/MotionCtrl_2 method found in piper_sdk interface.")
            time.sleep(0.2)

    def read_end_pose(self) -> EndPose | None:
        if self.dry_run:
            return self.last_pose
        if self.arm is None:
            return None

        feedback_getters = ("GetArmEndPoseMsgs", "GetArmEndPose", "get_end_pose")
        feedback = None
        for getter in feedback_getters:
            method = getattr(self.arm, getter, None)
            if method is not None:
                feedback = method()
                break
        if feedback is None:
            return self.last_pose

        values = self._extract_pose_values(feedback)
        if values is None:
            return self.last_pose
        xyz = np.array([piper_xyz_to_meters(v) for v in values[:3]], dtype=float)
        rpy = np.array([piper_rpy_to_degrees(v) for v in values[3:6]], dtype=float)
        self.last_pose = EndPose(xyz, rpy)
        return self.last_pose

    def _extract_pose_values(self, feedback: Any) -> list[float] | None:
        obj = feedback
        for attr in ("end_pose", "arm_end_pose", "pose"):
            obj = getattr(obj, attr, obj)
        names = ("X_axis", "Y_axis", "Z_axis", "RX_axis", "RY_axis", "RZ_axis")
        if all(hasattr(obj, name) for name in names):
            return [float(getattr(obj, name)) for name in names]
        lower_names = ("x", "y", "z", "rx", "ry", "rz")
        if all(hasattr(obj, name) for name in lower_names):
            return [float(getattr(obj, name)) for name in lower_names]
        if isinstance(obj, (tuple, list)) and len(obj) >= 6:
            return [float(v) for v in obj[:6]]
        return None

    def read_joint_pose(self, *, debug_feedback: bool = False) -> JointPose | None:
        if self.dry_run:
            return self.last_joint_command
        if self.arm is None:
            return None

        feedback_getters = ("GetArmJointMsgs", "GetArmJointCtrl", "GetArmLowSpdInfoMsgs", "get_joint_pose", "get_joint_state")
        found_getter = False
        for getter in feedback_getters:
            method = getattr(self.arm, getter, None)
            if method is None:
                continue
            found_getter = True
            try:
                feedback = method()
            except Exception as exc:
                self._warn_joint_feedback_once(getter, f"[Piper] Joint feedback getter {getter} failed: {exc!r}")
                continue
            if feedback is None:
                self._warn_joint_feedback_once(getter, f"[Piper] Joint feedback getter {getter} returned None.")
                continue
            if debug_feedback:
                print(f"[Piper] {getter} -> {feedback!r}")
                print(f"[Piper] dir({getter}) = {dir(feedback)}")
            values = self._extract_joint_values(feedback)
            if values is None:
                self._warn_joint_feedback_once(getter, f"[Piper] Could not parse joint feedback from {getter}: {feedback!r}")
                continue
            pose = JointPose(clamp_joints_deg([piper_joint_units_to_degrees(v) for v in values[:6]]))
            self.has_measured_joint_feedback = True
            self.last_joint_command = pose
            return pose
        if not found_getter:
            self._warn_joint_feedback_once("missing_getter", "[Piper] No joint feedback getter found.")
        return None

    def _warn_joint_feedback_once(self, key: str, message: str) -> None:
        if key not in self._joint_feedback_warnings:
            self._joint_feedback_warnings.add(key)
            print(message)

    def _extract_joint_values(self, feedback: Any) -> list[float] | None:
        candidates = [feedback]
        for attr in ("joint_state", "joint_ctrl", "arm_joint_msgs", "arm_joint_ctrl", "joint"):
            obj = getattr(feedback, attr, None)
            if obj is not None:
                candidates.append(obj)
        containers: list[Any] = []
        for obj in candidates:
            containers.append(obj)
            for attr in ("joint_state", "joint_ctrl", "joint"):
                nested = getattr(obj, attr, None)
                if nested is not None:
                    containers.append(nested)

        name_sets = (
            ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"),
            ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6"),
            ("j1", "j2", "j3", "j4", "j5", "j6"),
        )
        for obj in containers:
            for names in name_sets:
                if all(hasattr(obj, name) for name in names):
                    return [float(getattr(obj, name)) for name in names]
            if isinstance(obj, (tuple, list, np.ndarray)) and len(obj) >= 6:
                return [float(v) for v in obj[:6]]
            for attr in ("joints", "joint", "joint_values"):
                values = getattr(obj, attr, None)
                if isinstance(values, (tuple, list, np.ndarray)) and len(values) >= 6:
                    return [float(v) for v in values[:6]]
        return None

    def send_end_pose(self, xyz_m: np.ndarray, rpy_deg: np.ndarray) -> None:
        xyz_m = np.asarray(xyz_m, dtype=float)
        rpy_deg = np.asarray(rpy_deg, dtype=float)
        command = [
            meters_to_piper_xyz(float(xyz_m[0])),
            meters_to_piper_xyz(float(xyz_m[1])),
            meters_to_piper_xyz(float(xyz_m[2])),
            degrees_to_piper_rpy(float(rpy_deg[0])),
            degrees_to_piper_rpy(float(rpy_deg[1])),
            degrees_to_piper_rpy(float(rpy_deg[2])),
        ]
        # Host code uses meters and degrees internally. EndPoseCtrl expects
        # XYZ in 0.001 mm and RX/RY/RZ in 0.001 degrees.
        # Keep commanded and measured poses separate.  Treating a requested pose as
        # feedback makes a deadman hold command chase an old target while the arm is
        # still moving.
        self.last_command = EndPose(xyz_m.copy(), rpy_deg.copy())
        if self.dry_run:
            self.last_pose = EndPose(xyz_m.copy(), rpy_deg.copy())
            print(
                "[DRY-RUN] EndPoseCtrl "
                f"xyz_m={xyz_m.round(4).tolist()} rpy_deg={rpy_deg.round(2).tolist()} raw={command}"
            )
            return
        if self.arm is None:
            raise RuntimeError("Piper is not connected")
        # EndPoseCtrl sends an endpoint pose; Piper firmware performs internal IK.
        self.arm.EndPoseCtrl(*command)

    def send_joint_pose(self, joints_deg: np.ndarray) -> None:
        joints_deg = clamp_joints_deg(joints_deg)
        command = [degrees_to_piper_joint_units(float(value)) for value in joints_deg]
        self.last_joint_command = JointPose(joints_deg.copy())
        self.has_sent_joint_command = True
        if self.dry_run:
            print(f"[DRY-RUN] JointCtrl joints_deg={joints_deg.round(2).tolist()} raw={command}")
            return
        if self.arm is None:
            raise RuntimeError("Piper is not connected")
        if not hasattr(self.arm, "JointCtrl"):
            raise RuntimeError("No JointCtrl method found in piper_sdk interface.")
        self.arm.JointCtrl(*command)

    def send_gripper(self, opening_m: float) -> None:
        opening_m = max(0.0, min(float(opening_m), 0.08))
        raw = meters_to_gripper_units(opening_m)
        if self.dry_run:
            print(f"[DRY-RUN] Gripper opening_m={opening_m:.4f} raw={raw}")
            return
        self._call_first_available(("GripperCtrl", "gripper_ctrl"), raw, 1000, 0x01, 0)

    def hold(self) -> None:
        """Command the measured pose, rather than the previous target, to stop motion."""
        pose = self.read_end_pose()
        if pose is None:
            pose = self.last_pose
        self.send_end_pose(pose.xyz_m, pose.rpy_deg)

    def hold_joints(self, *, allow_last_command_fallback: bool = False) -> None:
        """Hold measured joint pose.

        In real mode, fallback to the last command is allowed only when a real
        JointCtrl command was already sent in this process and the caller
        explicitly opts in.
        """
        pose = self.read_joint_pose()
        if pose is None:
            if not allow_last_command_fallback or not self.has_sent_joint_command:
                raise RuntimeError("Cannot hold joints safely: no measured joint feedback and no allowed previous joint command fallback.")
            print("[Piper] WARNING: joint feedback unavailable; holding last sent joint command.")
            pose = self.last_joint_command
        self.send_joint_pose(pose.joints_deg)
