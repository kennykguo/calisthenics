# ABOUTME: Verifies the keyboard teleop loop maps single-key inputs into safe arm controller calls.
# ABOUTME: Exercises teleop behavior with a fake controller so no real arm hardware is required.

from __future__ import annotations

import io
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from arm_controller import ArmPose
from arm_service import ArmStatus, Snapshot
from arm_teleop import KeyEvent, StepSizes, handle_keypress, run_teleop


def make_status(seq: int = 7) -> ArmStatus:
    return ArmStatus(
        seq=seq,
        snapshot=Snapshot(
            armed=True,
            fault=None,
            moving=False,
            active_joint=None,
            servo_voltage_mv=5132,
            current_deg=[90, 90, 100, 120],
            target_deg=[90, 90, 100, 120],
        ),
    )


class FakeController:
    def __init__(self) -> None:
        self.calls: list[tuple] = []

    def clear_fault(self) -> None:
        self.calls.append(("clear_fault",))

    def arm(self) -> None:
        self.calls.append(("arm",))

    def disarm(self) -> None:
        self.calls.append(("disarm",))

    def home(self) -> ArmStatus:
        self.calls.append(("home",))
        return make_status(seq=10)

    def open_gripper(self) -> ArmStatus:
        self.calls.append(("open_gripper",))
        return make_status(seq=11)

    def close_gripper(self) -> ArmStatus:
        self.calls.append(("close_gripper",))
        return ArmStatus(
            seq=12,
            snapshot=Snapshot(
                armed=True,
                fault=None,
                moving=False,
                active_joint=None,
                servo_voltage_mv=5132,
                current_deg=[90, 90, 100, 28],
                target_deg=[90, 90, 100, 28],
            ),
        )

    def status(self) -> ArmStatus:
        self.calls.append(("status",))
        return make_status(seq=13)

    def wait(self, duration_ms: int) -> ArmStatus:
        self.calls.append(("wait", duration_ms))
        return make_status(seq=14)

    def step_joint(self, joint: str, delta_deg: int) -> ArmStatus:
        self.calls.append(("step_joint", joint, delta_deg))
        return make_status(seq=15)


class ArmTeleopTests(unittest.TestCase):
    def test_maps_shoulder_forward_key_to_positive_step(self) -> None:
        controller = FakeController()

        step_sizes, response, should_exit = handle_keypress(
            "w",
            controller=controller,
            step_sizes=StepSizes(joint_deg=5, gripper_deg=10),
        )

        self.assertEqual(step_sizes, StepSizes(joint_deg=5, gripper_deg=10))
        self.assertFalse(should_exit)
        self.assertEqual(response["kind"], "status")
        self.assertEqual(controller.calls, [("step_joint", "shoulder", 5)])

    def test_maps_gripper_open_key_to_positive_gripper_step(self) -> None:
        controller = FakeController()

        _, response, _ = handle_keypress(
            "l",
            controller=controller,
            step_sizes=StepSizes(joint_deg=5, gripper_deg=10),
        )

        self.assertEqual(response["kind"], "status")
        self.assertEqual(controller.calls, [("step_joint", "gripper", 10)])

    def test_plus_key_increases_step_size(self) -> None:
        controller = FakeController()

        step_sizes, response, should_exit = handle_keypress(
            "+",
            controller=controller,
            step_sizes=StepSizes(joint_deg=5, gripper_deg=10),
        )

        self.assertEqual(step_sizes, StepSizes(joint_deg=10, gripper_deg=15))
        self.assertFalse(should_exit)
        self.assertEqual(response, {"kind": "teleop", "joint_step_deg": 10, "gripper_step_deg": 15})
        self.assertEqual(controller.calls, [])

    def test_minus_key_decreases_step_size_without_going_below_one(self) -> None:
        controller = FakeController()

        step_sizes, response, should_exit = handle_keypress(
            "-",
            controller=controller,
            step_sizes=StepSizes(joint_deg=3, gripper_deg=4),
        )

        self.assertEqual(step_sizes, StepSizes(joint_deg=1, gripper_deg=1))
        self.assertFalse(should_exit)
        self.assertEqual(response, {"kind": "teleop", "joint_step_deg": 1, "gripper_step_deg": 1})
        self.assertEqual(controller.calls, [])

    def test_q_key_exits_without_touching_the_controller(self) -> None:
        controller = FakeController()

        step_sizes, response, should_exit = handle_keypress(
            "q",
            controller=controller,
            step_sizes=StepSizes(joint_deg=5, gripper_deg=10),
        )

        self.assertEqual(step_sizes, StepSizes(joint_deg=5, gripper_deg=10))
        self.assertIsNone(response)
        self.assertTrue(should_exit)
        self.assertEqual(controller.calls, [])

    def test_idle_events_keep_the_watchdog_alive(self) -> None:
        controller = FakeController()
        output = io.StringIO()
        events = iter(
            [
                KeyEvent(kind="idle"),
                KeyEvent(kind="key", key="t"),
                KeyEvent(kind="eof"),
            ]
        )

        run_teleop(
            controller=controller,
            output=output,
            next_event=lambda: next(events),
        )

        self.assertEqual(
            controller.calls,
            [("wait", 0), ("status",)],
        )
        rendered_lines = output.getvalue().splitlines()
        self.assertEqual(rendered_lines[1], '{"kind":"status","seq":13,"armed":true,"fault":"NONE","moving":false,"active":"NONE","voltage_mv":5132,"current":[90,90,100,120],"target":[90,90,100,120]}')


if __name__ == "__main__":
    unittest.main()
