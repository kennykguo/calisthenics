# ABOUTME: Provides a typed local control layer on top of the arm service with calibrated-limit checks.
# ABOUTME: Validates absolute and relative motion requests before they leave the host and exposes typed poses.

from __future__ import annotations

from dataclasses import dataclass

from arm_service import ArmLimits, ArmServiceClient, ArmStatus


class ArmControllerError(RuntimeError):
    pass


@dataclass(frozen=True)
class ArmPose:
    base: int
    shoulder: int
    elbow: int
    gripper: int

    @classmethod
    def from_status(cls, status: ArmStatus) -> "ArmPose":
        return cls(
            base=status.snapshot.current_deg[0],
            shoulder=status.snapshot.current_deg[1],
            elbow=status.snapshot.current_deg[2],
            gripper=status.snapshot.current_deg[3],
        )

    @classmethod
    def from_limits_home(cls, limits: ArmLimits) -> "ArmPose":
        return cls(
            base=limits.base.home_deg,
            shoulder=limits.shoulder.home_deg,
            elbow=limits.elbow.home_deg,
            gripper=limits.gripper.home_deg,
        )


class ArmController:
    def __init__(self, client: ArmServiceClient, limits: ArmLimits) -> None:
        self._client = client
        self._limits = limits

    @classmethod
    def connect(cls, address: str, timeout_s: float = 2.0) -> "ArmController":
        client = ArmServiceClient.connect(address, timeout_s=timeout_s)
        try:
            limits = client.limits()
        except Exception:
            client.close()
            raise

        return cls(client, limits)

    @property
    def limits(self) -> ArmLimits:
        return self._limits

    def close(self) -> None:
        self._client.close()

    def __enter__(self) -> "ArmController":
        return self

    def __exit__(self, exc_type: object, exc: object, exc_tb: object) -> None:
        self.close()

    def refresh_limits(self) -> ArmLimits:
        self._limits = self._client.limits()
        return self._limits

    def status(self) -> ArmStatus:
        return self._client.status()

    def current_pose(self) -> ArmPose:
        return ArmPose.from_status(self.status())

    def home_pose(self) -> ArmPose:
        return ArmPose.from_limits_home(self._limits)

    def clear_fault(self) -> None:
        self._client.clear_fault()

    def arm(self) -> None:
        self._client.arm()

    def disarm(self) -> None:
        self._client.disarm()

    def home(self) -> ArmStatus:
        return self._client.home()

    def open_gripper(self) -> ArmStatus:
        return self._client.open_gripper()

    def close_gripper(self) -> ArmStatus:
        return self._client.close_gripper()

    def wait(self, duration_ms: int) -> ArmStatus:
        return self._client.wait(duration_ms)

    def move_joint(self, joint: str, angle_deg: int) -> ArmStatus:
        self._validate_joint_angle(joint, angle_deg)
        return self._client.move_joint(joint, angle_deg)

    def move_pose(self, pose: ArmPose) -> ArmStatus:
        self._validate_pose(pose)
        return self._client.move_pose(
            base=pose.base,
            shoulder=pose.shoulder,
            elbow=pose.elbow,
            gripper=pose.gripper,
        )

    def offset_pose(
        self,
        *,
        base_delta: int = 0,
        shoulder_delta: int = 0,
        elbow_delta: int = 0,
        gripper_delta: int = 0,
    ) -> ArmStatus:
        current_pose = self.current_pose()
        return self.move_pose(
            ArmPose(
                base=current_pose.base + base_delta,
                shoulder=current_pose.shoulder + shoulder_delta,
                elbow=current_pose.elbow + elbow_delta,
                gripper=current_pose.gripper + gripper_delta,
            )
        )

    def step_joint(self, joint: str, delta_deg: int) -> ArmStatus:
        current_pose = self.current_pose()
        target = {
            "base": current_pose.base,
            "shoulder": current_pose.shoulder,
            "elbow": current_pose.elbow,
            "gripper": current_pose.gripper,
        }
        normalized_joint = self._normalize_joint_name(joint)
        target[normalized_joint] += delta_deg
        return self.move_joint(normalized_joint, target[normalized_joint])

    def _validate_pose(self, pose: ArmPose) -> None:
        self._validate_joint_angle("base", pose.base)
        self._validate_joint_angle("shoulder", pose.shoulder)
        self._validate_joint_angle("elbow", pose.elbow)
        self._validate_joint_angle("gripper", pose.gripper)

    def _validate_joint_angle(self, joint: str, angle_deg: int) -> None:
        normalized_joint = self._normalize_joint_name(joint)
        limits = {
            "base": self._limits.base,
            "shoulder": self._limits.shoulder,
            "elbow": self._limits.elbow,
            "gripper": self._limits.gripper,
        }[normalized_joint]

        if angle_deg < limits.min_deg or angle_deg > limits.max_deg:
            raise ArmControllerError(
                f"{normalized_joint} angle {angle_deg} is outside the calibrated range {limits.min_deg}..{limits.max_deg}"
            )

    def _normalize_joint_name(self, joint: str) -> str:
        normalized_joint = joint.strip().lower()
        if normalized_joint not in {"base", "shoulder", "elbow", "gripper"}:
            raise ArmControllerError(
                "joint must be one of: base, shoulder, elbow, gripper"
            )
        return normalized_joint
