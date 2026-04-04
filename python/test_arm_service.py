# ABOUTME: Verifies the local Python arm service client against a fake newline-delimited JSON socket.
# ABOUTME: Confirms typed parsing, request ids, and error handling without requiring real arm hardware.

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from arm_service import (
    ArmLimits,
    ArmServiceClient,
    ArmServiceError,
    ArmStatus,
    GripperLimits,
    JointLimits,
)
from test_support import FakeArmService


class ArmServiceClientTests(unittest.TestCase):
    def test_limits_request_is_typed(self) -> None:
        service = FakeArmService(
            [
                {
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
            ]
        )
        service.start()
        try:
            client = ArmServiceClient.connect(
                f"{service.address[0]}:{service.address[1]}"
            )
            try:
                self.assertEqual(
                    client.limits(),
                    ArmLimits(
                        base=JointLimits(min_deg=0, max_deg=180, home_deg=90),
                        shoulder=JointLimits(min_deg=50, max_deg=120, home_deg=90),
                        elbow=JointLimits(min_deg=50, max_deg=125, home_deg=100),
                        gripper=GripperLimits(
                            min_deg=18,
                            max_deg=120,
                            home_deg=120,
                            closed_deg=38,
                        ),
                        heartbeat_timeout_ms=2000,
                        servo_voltage_min_mv=4700,
                    ),
                )
            finally:
                client.close()
        finally:
            service.stop()

        self.assertEqual(service.requests, [{"id": "req-1", "command": "limits"}])

    def test_status_request_is_typed(self) -> None:
        service = FakeArmService(
            [
                {
                    "id": "req-1",
                    "kind": "status",
                    "command": "status",
                    "seq": 41,
                    "armed": True,
                    "fault": "NONE",
                    "moving": False,
                    "active": "NONE",
                    "voltage_mv": 5130,
                    "current": [90, 90, 100, 120],
                    "target": [90, 90, 100, 120],
                }
            ]
        )
        service.start()
        try:
            client = ArmServiceClient.connect(
                f"{service.address[0]}:{service.address[1]}"
            )
            try:
                status = client.status()
            finally:
                client.close()
        finally:
            service.stop()

        self.assertIsInstance(status, ArmStatus)
        self.assertEqual(status.seq, 41)
        self.assertTrue(status.snapshot.armed)
        self.assertEqual(status.snapshot.servo_voltage_mv, 5130)
        self.assertEqual(status.snapshot.current_deg, [90, 90, 100, 120])

    def test_status_timeout_raises_arm_service_error(self) -> None:
        service = FakeArmService(
            [
                (
                    0.1,
                    {
                        "id": "req-1",
                        "kind": "status",
                        "command": "status",
                        "seq": 41,
                        "armed": True,
                        "fault": "NONE",
                        "moving": False,
                        "active": "NONE",
                        "voltage_mv": 5130,
                        "current": [90, 90, 100, 120],
                        "target": [90, 90, 100, 120],
                    },
                )
            ]
        )
        service.start()
        try:
            client = ArmServiceClient.connect(
                f"{service.address[0]}:{service.address[1]}",
                timeout_s=0.05,
            )
            try:
                with self.assertRaises(ArmServiceError) as context:
                    client.status()
            finally:
                client.close()
        finally:
            service.stop()

        self.assertIn("timed out", str(context.exception))

    def test_home_uses_a_longer_reply_timeout_than_fast_commands(self) -> None:
        service = FakeArmService(
            [
                (
                    0.1,
                    {
                        "id": "req-1",
                        "kind": "status",
                        "command": "home",
                        "seq": 42,
                        "armed": True,
                        "fault": "NONE",
                        "moving": False,
                        "active": "NONE",
                        "voltage_mv": 5130,
                        "current": [90, 90, 100, 120],
                        "target": [90, 90, 100, 120],
                    },
                )
            ]
        )
        service.start()
        try:
            client = ArmServiceClient.connect(
                f"{service.address[0]}:{service.address[1]}",
                timeout_s=0.05,
            )
            try:
                status = client.home()
            finally:
                client.close()
        finally:
            service.stop()

        self.assertEqual(status.seq, 42)

    def test_error_replies_raise(self) -> None:
        service = FakeArmService(
            [
                {
                    "id": "req-1",
                    "kind": "error",
                    "command": "joint",
                    "message": "shoulder angle 40 is outside the calibrated range 50..120",
                }
            ]
        )
        service.start()
        try:
            client = ArmServiceClient.connect(
                f"{service.address[0]}:{service.address[1]}"
            )
            try:
                with self.assertRaises(ArmServiceError) as context:
                    client.move_joint("shoulder", 40)
            finally:
                client.close()
        finally:
            service.stop()

        self.assertIn("shoulder angle 40", str(context.exception))

    def test_mismatched_request_ids_raise(self) -> None:
        service = FakeArmService(
            [
                {
                    "id": "req-99",
                    "kind": "ok",
                    "command": "clear-fault",
                }
            ]
        )
        service.start()
        try:
            client = ArmServiceClient.connect(
                f"{service.address[0]}:{service.address[1]}"
            )
            try:
                with self.assertRaises(ArmServiceError) as context:
                    client.clear_fault()
            finally:
                client.close()
        finally:
            service.stop()

        self.assertIn("mismatched id", str(context.exception))


if __name__ == "__main__":
    unittest.main()
