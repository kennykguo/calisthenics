# ABOUTME: Verifies the Python arm operator parses commands and renders stable JSON replies.
# ABOUTME: Exercises the operator against a fake controller so no real arm hardware is required.

from __future__ import annotations

import io
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from arm_controller import ArmPose
from arm_operator import (
    Invocation,
    Offset,
    SessionEvent,
    execute_invocation,
    parse_invocation,
    parse_session_line,
    render_response,
    run_script,
    run_session,
)
from arm_service import ArmLimits, ArmStatus, GripperLimits, JointLimits, Snapshot


def make_limits() -> ArmLimits:
    return ArmLimits(
        base=JointLimits(min_deg=0, max_deg=180, home_deg=90),
        shoulder=JointLimits(min_deg=50, max_deg=120, home_deg=90),
        elbow=JointLimits(min_deg=50, max_deg=125, home_deg=100),
        gripper=GripperLimits(min_deg=18, max_deg=120, home_deg=120, closed_deg=38),
        heartbeat_timeout_ms=2000,
        servo_voltage_min_mv=4700,
    )


def make_status(
    *,
    seq: int = 7,
    current: list[int] | None = None,
    target: list[int] | None = None,
) -> ArmStatus:
    current = current or [90, 90, 100, 120]
    return ArmStatus(
        seq=seq,
        snapshot=Snapshot(
            armed=True,
            fault=None,
            moving=False,
            active_joint=None,
            servo_voltage_mv=5132,
            current_deg=current,
            target_deg=target or current,
        ),
    )


class FakeController:
    def __init__(self) -> None:
        self.limits = make_limits()
        self.calls: list[tuple] = []

    def status(self) -> ArmStatus:
        self.calls.append(("status",))
        return make_status(seq=9)

    def current_pose(self) -> ArmPose:
        self.calls.append(("current_pose",))
        return ArmPose(base=90, shoulder=90, elbow=100, gripper=120)

    def home_pose(self) -> ArmPose:
        self.calls.append(("home_pose",))
        return ArmPose(base=90, shoulder=90, elbow=100, gripper=120)

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
        return make_status(seq=12, current=[90, 90, 100, 38], target=[90, 90, 100, 38])

    def wait(self, duration_ms: int) -> ArmStatus:
        self.calls.append(("wait", duration_ms))
        return make_status(seq=13)

    def move_joint(self, joint: str, angle_deg: int) -> ArmStatus:
        self.calls.append(("move_joint", joint, angle_deg))
        return make_status(seq=14)

    def step_joint(self, joint: str, delta_deg: int) -> ArmStatus:
        self.calls.append(("step_joint", joint, delta_deg))
        return make_status(seq=15)

    def move_pose(self, pose: ArmPose) -> ArmStatus:
        self.calls.append(("move_pose", pose))
        return make_status(seq=16)

    def offset_pose(
        self,
        *,
        base_delta: int = 0,
        shoulder_delta: int = 0,
        elbow_delta: int = 0,
        gripper_delta: int = 0,
    ) -> ArmStatus:
        self.calls.append(
            (
                "offset_pose",
                base_delta,
                shoulder_delta,
                elbow_delta,
                gripper_delta,
            )
        )
        return make_status(seq=17)

    def note(self, text: str) -> dict:
        self.calls.append(("note", text))
        return {"kind": "note", "text": text}


class ArmOperatorTests(unittest.TestCase):
    def test_parses_step_commands(self) -> None:
        self.assertEqual(
            parse_invocation(["127.0.0.1:7878", "step", "shoulder", "5"]),
            Invocation(
                address="127.0.0.1:7878",
                command="step",
                joint="shoulder",
                delta_deg=5,
            ),
        )

    def test_parses_offset_commands(self) -> None:
        self.assertEqual(
            parse_invocation(["127.0.0.1:7878", "offset", "5", "-5", "0", "10"]),
            Invocation(
                address="127.0.0.1:7878",
                command="offset",
                offset=Offset(base=5, shoulder=-5, elbow=0, gripper=10),
            ),
        )

    def test_executes_limits_commands(self) -> None:
        controller = FakeController()
        response = execute_invocation(
            Invocation(address="127.0.0.1:7878", command="limits"),
            controller,
        )

        self.assertEqual(response["kind"], "limits")
        self.assertEqual(response["base"]["home_deg"], 90)
        self.assertEqual(controller.calls, [])

    def test_executes_step_commands(self) -> None:
        controller = FakeController()
        response = execute_invocation(
            Invocation(
                address="127.0.0.1:7878",
                command="step",
                joint="elbow",
                delta_deg=-5,
            ),
            controller,
        )

        self.assertEqual(response["kind"], "status")
        self.assertEqual(response["seq"], 15)
        self.assertEqual(controller.calls, [("step_joint", "elbow", -5)])

    def test_executes_offset_commands(self) -> None:
        controller = FakeController()
        response = execute_invocation(
            Invocation(
                address="127.0.0.1:7878",
                command="offset",
                offset=Offset(base=0, shoulder=-5, elbow=5, gripper=0),
            ),
            controller,
        )

        self.assertEqual(response["kind"], "status")
        self.assertEqual(response["seq"], 17)
        self.assertEqual(controller.calls, [("offset_pose", 0, -5, 5, 0)])

    def test_renders_pose_output_as_stable_json(self) -> None:
        self.assertEqual(
            render_response(
                {
                    "kind": "pose",
                    "label": "home",
                    "base": 90,
                    "shoulder": 90,
                    "elbow": 100,
                    "gripper": 120,
                }
            ),
            '{"kind":"pose","label":"home","base":90,"shoulder":90,"elbow":100,"gripper":120}',
        )

    def test_parses_serve_commands(self) -> None:
        self.assertEqual(
            parse_invocation(["127.0.0.1:7878", "serve"]),
            Invocation(address="127.0.0.1:7878", command="serve"),
        )

    def test_parses_note_commands(self) -> None:
        self.assertEqual(
            parse_invocation(["127.0.0.1:7878", "note", "close", "gripper"]),
            Invocation(
                address="127.0.0.1:7878",
                command="note",
                note_text="close gripper",
            ),
        )

    def test_parses_run_commands(self) -> None:
        self.assertEqual(
            parse_invocation(["127.0.0.1:7878", "run", "demo.arm"]),
            Invocation(
                address="127.0.0.1:7878",
                command="run",
                script_path="demo.arm",
            ),
        )

    def test_parses_record_commands(self) -> None:
        self.assertEqual(
            parse_invocation(["127.0.0.1:7878", "record", "demo.arm", "demo.jsonl"]),
            Invocation(
                address="127.0.0.1:7878",
                command="record",
                script_path="demo.arm",
                log_path="demo.jsonl",
            ),
        )

    def test_parses_session_lines(self) -> None:
        self.assertEqual(
            parse_session_line("step shoulder 5", "127.0.0.1:7878"),
            Invocation(
                address="127.0.0.1:7878",
                command="step",
                joint="shoulder",
                delta_deg=5,
            ),
        )

    def test_ignores_blank_and_comment_session_lines(self) -> None:
        self.assertIsNone(parse_session_line("   ", "127.0.0.1:7878"))
        self.assertIsNone(parse_session_line("# arm later", "127.0.0.1:7878"))

    def test_runs_session_commands_over_one_controller(self) -> None:
        controller = FakeController()
        stdout = io.StringIO()
        run_session(
            address="127.0.0.1:7878",
            controller=controller,
            lines=io.StringIO("# comment\nstatus\nstep shoulder 5\n"),
            output=stdout,
        )

        self.assertEqual(
            stdout.getvalue().splitlines(),
            [
                '{"kind":"status","seq":9,"armed":true,"fault":"NONE","moving":false,"active":"NONE","voltage_mv":5132,"current":[90,90,100,120],"target":[90,90,100,120]}',
                '{"kind":"status","seq":15,"armed":true,"fault":"NONE","moving":false,"active":"NONE","voltage_mv":5132,"current":[90,90,100,120],"target":[90,90,100,120]}',
            ],
        )
        self.assertEqual(controller.calls, [("status",), ("step_joint", "shoulder", 5)])

    def test_runs_note_commands_over_one_controller(self) -> None:
        controller = FakeController()
        stdout = io.StringIO()
        run_session(
            address="127.0.0.1:7878",
            controller=controller,
            lines=io.StringIO("note close gripper\n"),
            output=stdout,
        )

        self.assertEqual(
            stdout.getvalue().splitlines(),
            ['{"kind":"note","text":"close gripper"}'],
        )
        self.assertEqual(controller.calls, [])

    def test_reports_session_errors_without_stopping(self) -> None:
        controller = FakeController()
        stdout = io.StringIO()
        run_session(
            address="127.0.0.1:7878",
            controller=controller,
            lines=io.StringIO("bogus\nstatus\n"),
            output=stdout,
        )

        self.assertEqual(
            stdout.getvalue().splitlines(),
            [
                '{"kind":"error","message":"usage: python3 python/arm_operator.py <address> <serve|limits|status|current-pose|home-pose|clear-fault|arm|disarm|home|open|close|note <text...>|run <script>|record <script> <log.jsonl>|wait <ms>|joint <joint> <deg>|step <joint> <delta>|pose <base> <shoulder> <elbow> <gripper>|offset <base_delta> <shoulder_delta> <elbow_delta> <gripper_delta>>"}',
                '{"kind":"status","seq":9,"armed":true,"fault":"NONE","moving":false,"active":"NONE","voltage_mv":5132,"current":[90,90,100,120],"target":[90,90,100,120]}',
            ],
        )
        self.assertEqual(controller.calls, [("status",)])

    def test_keeps_the_watchdog_alive_while_waiting_for_input(self) -> None:
        controller = FakeController()
        stdout = io.StringIO()
        events = iter(
            [
                SessionEvent(kind="idle"),
                SessionEvent(kind="line", line="status\n"),
                SessionEvent(kind="eof"),
            ]
        )

        run_session(
            address="127.0.0.1:7878",
            controller=controller,
            lines=io.StringIO(),
            output=stdout,
            next_event=lambda: next(events),
        )

        self.assertEqual(
            controller.calls,
            [("wait", 0), ("status",)],
        )
        self.assertEqual(
            stdout.getvalue().splitlines(),
            [
                '{"kind":"status","seq":9,"armed":true,"fault":"NONE","moving":false,"active":"NONE","voltage_mv":5132,"current":[90,90,100,120],"target":[90,90,100,120]}',
            ],
        )

    def test_runs_scripts_from_disk(self) -> None:
        controller = FakeController()
        stdout = io.StringIO()
        script_path = Path(tempfile.gettempdir()) / "arm-operator-run.arm"
        script_path.write_text("# comment\nnote close gripper\nstatus\n", encoding="utf-8")
        try:
            run_script(
                address="127.0.0.1:7878",
                controller=controller,
                script_path=str(script_path),
                output=stdout,
            )
        finally:
            script_path.unlink(missing_ok=True)

        self.assertEqual(
            stdout.getvalue().splitlines(),
            [
                '{"kind":"note","text":"close gripper"}',
                '{"kind":"status","seq":9,"armed":true,"fault":"NONE","moving":false,"active":"NONE","voltage_mv":5132,"current":[90,90,100,120],"target":[90,90,100,120]}',
            ],
        )
        self.assertEqual(controller.calls, [("status",)])

    def test_records_script_runs_as_jsonl(self) -> None:
        controller = FakeController()
        stdout = io.StringIO()
        with tempfile.TemporaryDirectory(prefix="arm-operator-record-") as temp_dir_name:
            temp_dir = Path(temp_dir_name)
            script_path = temp_dir / "nested" / "demo.arm"
            log_path = temp_dir / "logs" / "demo.jsonl"
            script_path.parent.mkdir(parents=True, exist_ok=True)
            script_path.write_text("status\nstep shoulder 5\n", encoding="utf-8")

            run_script(
                address="127.0.0.1:7878",
                controller=controller,
                script_path=str(script_path),
                output=stdout,
                log_path=str(log_path),
            )
            log_lines = log_path.read_text(encoding="utf-8").splitlines()

        self.assertEqual(len(log_lines), 2)
        first_event = json.loads(log_lines[0])
        second_event = json.loads(log_lines[1])
        self.assertEqual(first_event["line"], "status")
        self.assertEqual(first_event["response"]["kind"], "status")
        self.assertEqual(second_event["line"], "step shoulder 5")
        self.assertEqual(second_event["response"]["seq"], 15)


if __name__ == "__main__":
    unittest.main()
