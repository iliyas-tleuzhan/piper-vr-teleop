"""Piper hardware driver wrapper for endpoint control."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any

import numpy as np

from .units import degrees_to_piper_rpy, meters_to_gripper_units, meters_to_piper_xyz, piper_rpy_to_degrees, piper_xyz_to_meters


@dataclass
class EndPose:
    xyz_m: np.ndarray
    rpy_deg: np.ndarray


class PiperDriver:
    def __init__(self, can: str = "can0", speed_percent: int = 10, dry_run: bool = False) -> None:
        self.can = can
        self.speed_percent = int(speed_percent)
        self.dry_run = dry_run
        self.arm: Any | None = None
        self.last_pose = EndPose(np.array([0.35, 0.0, 0.25], dtype=float), np.zeros(3, dtype=float))

    def connect(self) -> None:
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
        self.set_move_p_mode()

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
            self.arm.EnableArm(7, 0x02)  # 7 = all motors, 0x02 = enable
            time.sleep(0.2)

    def set_move_p_mode(self) -> None:
        if self.dry_run:
            return

        if self.arm is None:
            raise RuntimeError("Piper interface is not initialized.")

        print("[Piper] Setting MOVE P endpoint mode...")
        for _ in range(5):
            if hasattr(self.arm, "ModeCtrl"):
                # ctrl_mode=0x01 CAN control, move_mode=0x00 MOVE P endpoint mode
                self.arm.ModeCtrl(0x01, 0x00, self.speed_percent, 0x00)
            elif hasattr(self.arm, "MotionCtrl_2"):
                self.arm.MotionCtrl_2(0x01, 0x00, self.speed_percent, 0x00)
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
        self.last_pose = EndPose(xyz_m.copy(), rpy_deg.copy())
        if self.dry_run:
            print(
                "[DRY-RUN] EndPoseCtrl "
                f"xyz_m={xyz_m.round(4).tolist()} rpy_deg={rpy_deg.round(2).tolist()} raw={command}"
            )
            return
        if self.arm is None:
            raise RuntimeError("Piper is not connected")
        # EndPoseCtrl sends an endpoint pose; Piper firmware performs internal IK.
        self.arm.EndPoseCtrl(*command)

    def send_gripper(self, opening_m: float) -> None:
        opening_m = max(0.0, min(float(opening_m), 0.08))
        raw = meters_to_gripper_units(opening_m)
        if self.dry_run:
            print(f"[DRY-RUN] Gripper opening_m={opening_m:.4f} raw={raw}")
            return
        self._call_first_available(("GripperCtrl", "gripper_ctrl"), raw, 1000, 0x01, 0)

    def hold(self) -> None:
        self.send_end_pose(self.last_pose.xyz_m, self.last_pose.rpy_deg)
