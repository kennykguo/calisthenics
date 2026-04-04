# ABOUTME: Verifies the local Python arm controller validates moves against calibrated limits.
# ABOUTME: Confirms home/current poses and relative joint steps without requiring real arm hardware.

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from arm_controller import ArmController, ArmControllerError, ArmPose
from arm_service import ArmStatus
from test_support import FakeArmService


def make_limits_response() -> dict:
    return {
        "id": "req-1",
        "kind": "limits",
        "base": {"min_deg": 0, "max_deg": 180, "home_deg": 90},
        "shoulder": {"min_deg": 50, "max_deg": 120, "home_deg": 90},
        "elbow": {"min_deg": 50, "max_deg": 125, "home_deg": 100},
        "gripper": {
            "min_deg": 18,
            "max_deg": 120,
            "home_deg": 120,
            "closed_deg": 38,
        },
        "heartbeat_timeout_ms": 2000,
        "servo_voltage_min_mv": 4700,
    }


def make_status_response(
    request_id: str,
    command: str,
    current: list[int],
    target: list[int] | None = None,
    seq: int = 7,
) -> dict:
    return {
        "id": request_id,
        "kind": "status",
        "command": command,
        "seq": seq,
        "armed": True,
        "fault": "NONE",
        "moving": False,
        "active": "NONE",
        "voltage_mv": 5130,
        "current": current,
        "target": target or current,
    }


class ArmControllerTests(unittest.TestCase):
    def test_home_pose_comes_from_limits(self) -> None:
        service = FakeArmService([make_limits_response()])
        service.start()
        try:
            with ArmController.connect(
                f"{service.address[0]}:{service.address[1]}"
            ) as controller:
                self.assertEqual(
                    controller.home_pose(),
                    ArmPose(base=90, shoulder=90, elbow=100, gripper=120),
                )
        finally:
            service.stop()

    def test_move_joint_rejects_out_of_range_angles_locally(self) -> None:
        service = FakeArmService([make_limits_response()])
        service.start()
        try:
            with ArmController.connect(
                f"{service.address[0]}:{service.address[1]}"
            ) as controller:
                with self.assertRaises(ArmControllerError) as context:
                    controller.move_joint("shoulder", 40)
        finally:
            service.stop()

        self.assertIn("shoulder angle 40", str(context.exception))
        self.assertEqual(service.requests, [{"id": "req-1", "command": "limits"}])

    def test_move_pose_rejects_out_of_range_pose_locally(self) -> None:
        service = FakeArmService([make_limits_response()])
        service.start()
        try:
            with ArmController.connect(
                f"{service.address[0]}:{service.address[1]}"
            ) as controller:
                with self.assertRaises(ArmControllerError) as context:
                    controller.move_pose(
                        ArmPose(base=90, shoulder=90, elbow=130, gripper=120)
                    )
        finally:
            service.stop()

        self.assertIn("elbow angle 130", str(context.exception))
        self.assertEqual(service.requests, [{"id": "req-1", "command": "limits"}])

    def test_step_joint_uses_current_pose_then_sends_a_valid_move(self) -> None:
        service = FakeArmService(
            [
                make_limits_response(),
                make_status_response("req-2", "status", [90, 90, 100, 120], seq=10),
                make_status_response(
                    "req-3", "joint", [90, 95, 100, 120], [90, 95, 100, 120], seq=11
                ),
            ]
        )
        service.start()
        try:
            with ArmController.connect(
                f"{service.address[0]}:{service.address[1]}"
            ) as controller:
                status = controller.step_joint("shoulder", 5)
        finally:
            service.stop()

        self.assertIsInstance(status, ArmStatus)
        self.assertEqual(status.snapshot.current_deg, [90, 95, 100, 120])
        self.assertEqual(
            service.requests,
            [
                {"id": "req-1", "command": "limits"},
                {"id": "req-2", "command": "status"},
                {"id": "req-3", "command": "joint", "joint": "shoulder", "angle_deg": 95},
            ],
        )

    def test_current_pose_reads_live_status(self) -> None:
        service = FakeArmService(
            [
                make_limits_response(),
                make_status_response("req-2", "status", [120, 90, 110, 60], seq=14),
            ]
        )
        service.start()
        try:
            with ArmController.connect(
                f"{service.address[0]}:{service.address[1]}"
            ) as controller:
                self.assertEqual(
                    controller.current_pose(),
                    ArmPose(base=120, shoulder=90, elbow=110, gripper=60),
                )
        finally:
            service.stop()

    def test_wait_passes_through_to_the_service(self) -> None:
        service = FakeArmService(
            [
                make_limits_response(),
                make_status_response("req-2", "wait", [90, 90, 100, 120], seq=18),
            ]
        )
        service.start()
        try:
            with ArmController.connect(
                f"{service.address[0]}:{service.address[1]}"
            ) as controller:
                status = controller.wait(750)
        finally:
            service.stop()

        self.assertEqual(status.seq, 18)
        self.assertEqual(
            service.requests,
            [
                {"id": "req-1", "command": "limits"},
                {"id": "req-2", "command": "wait", "duration_ms": 750},
            ],
        )

    def test_offset_pose_uses_live_status_then_sends_a_pose_move(self) -> None:
        service = FakeArmService(
            [
                make_limits_response(),
                make_status_response("req-2", "status", [90, 90, 100, 120], seq=20),
                make_status_response(
                    "req-3", "pose", [95, 85, 100, 120], [95, 85, 100, 120], seq=21
                ),
            ]
        )
        service.start()
        try:
            with ArmController.connect(
                f"{service.address[0]}:{service.address[1]}"
            ) as controller:
                status = controller.offset_pose(base_delta=5, shoulder_delta=-5)
        finally:
            service.stop()

        self.assertEqual(status.snapshot.current_deg, [95, 85, 100, 120])
        self.assertEqual(
            service.requests,
            [
                {"id": "req-1", "command": "limits"},
                {"id": "req-2", "command": "status"},
                {
                    "id": "req-3",
                    "command": "pose",
                    "base": 95,
                    "shoulder": 85,
                    "elbow": 100,
                    "gripper": 120,
                },
            ],
        )

    def test_offset_pose_rejects_out_of_range_targets_locally(self) -> None:
        service = FakeArmService(
            [
                make_limits_response(),
                make_status_response("req-2", "status", [90, 90, 100, 120], seq=22),
            ]
        )
        service.start()
        try:
            with ArmController.connect(
                f"{service.address[0]}:{service.address[1]}"
            ) as controller:
                with self.assertRaises(ArmControllerError) as context:
                    controller.offset_pose(shoulder_delta=-50)
        finally:
            service.stop()

        self.assertIn("shoulder angle 40", str(context.exception))
        self.assertEqual(
            service.requests,
            [
                {"id": "req-1", "command": "limits"},
                {"id": "req-2", "command": "status"},
            ],
        )


if __name__ == "__main__":
    unittest.main()
