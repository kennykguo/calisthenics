# ABOUTME: Provides a local Python CLI on top of the arm controller for typed manual and programmatic control.
# ABOUTME: Exposes calibrated limits, poses, waits, absolute moves, and relative offsets as stable JSON replies.

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import select
import shlex
import sys
import time
from typing import Any, TextIO

from arm_controller import ArmController, ArmControllerError, ArmPose
from arm_service import ArmLimits, ArmServiceError, ArmStatus


@dataclass(frozen=True)
class Offset:
    base: int = 0
    shoulder: int = 0
    elbow: int = 0
    gripper: int = 0


@dataclass(frozen=True)
class Invocation:
    address: str
    command: str
    joint: str | None = None
    angle_deg: int | None = None
    delta_deg: int | None = None
    duration_ms: int | None = None
    pose: ArmPose | None = None
    offset: Offset | None = None
    note_text: str | None = None
    script_path: str | None = None
    log_path: str | None = None


@dataclass(frozen=True)
class SessionEvent:
    kind: str
    line: str = ""


def main() -> None:
    try:
        invocation = parse_invocation(sys.argv[1:])
        with ArmController.connect(invocation.address) as controller:
            if invocation.command == "serve":
                run_session(
                    address=invocation.address,
                    controller=controller,
                    lines=sys.stdin,
                    output=sys.stdout,
                )
            elif invocation.command == "run":
                run_script(
                    address=invocation.address,
                    controller=controller,
                    script_path=invocation.script_path or "",
                    output=sys.stdout,
                )
            elif invocation.command == "record":
                run_script(
                    address=invocation.address,
                    controller=controller,
                    script_path=invocation.script_path or "",
                    output=sys.stdout,
                    log_path=invocation.log_path,
                )
            else:
                response = execute_invocation(invocation, controller)
                print(render_response(response))
    except (ArmControllerError, ArmServiceError, ValueError) as error:
        print(render_response({"kind": "error", "message": str(error)}), file=sys.stderr)
        raise SystemExit(1)


def parse_invocation(args: list[str]) -> Invocation:
    if len(args) < 2:
        raise ValueError(usage())

    address = args[0]
    return parse_operator_command(address, args[1:])


def parse_operator_command(address: str, args: list[str]) -> Invocation:
    command = args[0]

    if command in {
        "serve",
        "limits",
        "status",
        "current-pose",
        "home-pose",
        "clear-fault",
        "arm",
        "disarm",
        "home",
        "open",
        "close",
    }:
        if len(args) != 1:
            raise ValueError(usage())
        return Invocation(address=address, command=command)

    if command == "note":
        if len(args) < 2:
            raise ValueError(usage())
        return Invocation(
            address=address,
            command=command,
            note_text=" ".join(args[1:]),
        )

    if command == "run":
        if len(args) != 2:
            raise ValueError(usage())
        return Invocation(
            address=address,
            command=command,
            script_path=args[1],
        )

    if command == "record":
        if len(args) != 3:
            raise ValueError(usage())
        return Invocation(
            address=address,
            command=command,
            script_path=args[1],
            log_path=args[2],
        )

    if command == "wait":
        if len(args) != 2:
            raise ValueError(usage())
        return Invocation(
            address=address,
            command=command,
            duration_ms=parse_int(args[1], "wait duration"),
        )

    if command == "joint":
        if len(args) != 3:
            raise ValueError(usage())
        return Invocation(
            address=address,
            command=command,
            joint=args[1],
            angle_deg=parse_int(args[2], "joint angle"),
        )

    if command == "step":
        if len(args) != 3:
            raise ValueError(usage())
        return Invocation(
            address=address,
            command=command,
            joint=args[1],
            delta_deg=parse_int(args[2], "joint delta"),
        )

    if command == "pose":
        if len(args) != 5:
            raise ValueError(usage())
        return Invocation(
            address=address,
            command=command,
            pose=ArmPose(
                base=parse_int(args[1], "base angle"),
                shoulder=parse_int(args[2], "shoulder angle"),
                elbow=parse_int(args[3], "elbow angle"),
                gripper=parse_int(args[4], "gripper angle"),
            ),
        )

    if command == "offset":
        if len(args) != 5:
            raise ValueError(usage())
        return Invocation(
            address=address,
            command=command,
            offset=Offset(
                base=parse_int(args[1], "base delta"),
                shoulder=parse_int(args[2], "shoulder delta"),
                elbow=parse_int(args[3], "elbow delta"),
                gripper=parse_int(args[4], "gripper delta"),
            ),
        )

    raise ValueError(usage())


def usage() -> str:
    return (
        "usage: python3 python/arm_operator.py <address> "
        "<serve|limits|status|current-pose|home-pose|clear-fault|arm|disarm|home|open|close|note <text...>|run <script>|record <script> <log.jsonl>|wait <ms>|"
        "joint <joint> <deg>|step <joint> <delta>|pose <base> <shoulder> <elbow> <gripper>|"
        "offset <base_delta> <shoulder_delta> <elbow_delta> <gripper_delta>>"
    )


def parse_int(value: str, label: str) -> int:
    try:
        return int(value)
    except ValueError as error:
        raise ValueError(f"{label} must be an integer") from error


def execute_invocation(
    invocation: Invocation,
    controller: ArmController,
) -> dict[str, Any]:
    command = invocation.command
    if command == "limits":
        return render_limits(controller.limits)
    if command == "status":
        return render_status(controller.status())
    if command == "current-pose":
        return render_pose("current", controller.current_pose())
    if command == "home-pose":
        return render_pose("home", controller.home_pose())
    if command == "serve":
        raise ValueError("`serve` must be run through the session loop")
    if command == "run":
        raise ValueError("`run` must be executed through the script runner")
    if command == "record":
        raise ValueError("`record` must be executed through the script runner")
    if command == "note":
        return {"kind": "note", "text": invocation.note_text or ""}
    if command == "clear-fault":
        controller.clear_fault()
        return {"kind": "ok", "command": "clear-fault"}
    if command == "arm":
        controller.arm()
        return {"kind": "ok", "command": "arm"}
    if command == "disarm":
        controller.disarm()
        return {"kind": "ok", "command": "disarm"}
    if command == "home":
        return render_status(controller.home())
    if command == "open":
        return render_status(controller.open_gripper())
    if command == "close":
        return render_status(controller.close_gripper())
    if command == "wait":
        return render_status(controller.wait(invocation.duration_ms or 0))
    if command == "joint":
        return render_status(controller.move_joint(invocation.joint or "", invocation.angle_deg or 0))
    if command == "step":
        return render_status(controller.step_joint(invocation.joint or "", invocation.delta_deg or 0))
    if command == "pose":
        if invocation.pose is None:
            raise ValueError("pose command is missing a target pose")
        return render_status(controller.move_pose(invocation.pose))
    if command == "offset":
        if invocation.offset is None:
            raise ValueError("offset command is missing pose deltas")
        return render_status(
            controller.offset_pose(
                base_delta=invocation.offset.base,
                shoulder_delta=invocation.offset.shoulder,
                elbow_delta=invocation.offset.elbow,
                gripper_delta=invocation.offset.gripper,
            )
        )
    raise ValueError(f"unsupported command `{command}`")


def render_limits(limits: ArmLimits) -> dict[str, Any]:
    return {
        "kind": "limits",
        "base": render_joint_limits(limits.base),
        "shoulder": render_joint_limits(limits.shoulder),
        "elbow": render_joint_limits(limits.elbow),
        "gripper": {
            "min_deg": limits.gripper.min_deg,
            "max_deg": limits.gripper.max_deg,
            "home_deg": limits.gripper.home_deg,
            "closed_deg": limits.gripper.closed_deg,
        },
        "heartbeat_timeout_ms": limits.heartbeat_timeout_ms,
        "servo_voltage_min_mv": limits.servo_voltage_min_mv,
    }


def render_joint_limits(limits: Any) -> dict[str, int]:
    return {
        "min_deg": limits.min_deg,
        "max_deg": limits.max_deg,
        "home_deg": limits.home_deg,
    }


def render_pose(label: str, pose: ArmPose) -> dict[str, Any]:
    return {
        "kind": "pose",
        "label": label,
        "base": pose.base,
        "shoulder": pose.shoulder,
        "elbow": pose.elbow,
        "gripper": pose.gripper,
    }


def render_status(status: ArmStatus) -> dict[str, Any]:
    return {
        "kind": "status",
        "seq": status.seq,
        "armed": status.snapshot.armed,
        "fault": status.snapshot.fault or "NONE",
        "moving": status.snapshot.moving,
        "active": status.snapshot.active_joint or "NONE",
        "voltage_mv": status.snapshot.servo_voltage_mv or 0,
        "current": status.snapshot.current_deg,
        "target": status.snapshot.target_deg,
    }


def render_response(response: dict[str, Any]) -> str:
    return json.dumps(response, separators=(",", ":"))


def parse_session_line(line: str, address: str) -> Invocation | None:
    stripped = line.strip()
    if stripped == "" or stripped.startswith("#"):
        return None
    return parse_operator_command(address, shlex.split(stripped))


def run_session(
    *,
    address: str,
    controller: ArmController,
    lines: TextIO,
    output: TextIO,
    next_event: Any | None = None,
) -> None:
    if next_event is None:
        next_event = lambda: read_session_event(lines)

    while True:
        event = next_event()
        if event.kind == "eof":
            return

        if event.kind == "idle":
            try:
                controller.wait(0)
            except (ArmControllerError, ArmServiceError, ValueError):
                pass
            continue

        try:
            invocation = parse_session_line(event.line, address)
            if invocation is None:
                continue

            response = execute_invocation(invocation, controller)
        except (ArmControllerError, ArmServiceError, ValueError) as error:
            response = {"kind": "error", "message": str(error)}

        output.write(f"{render_response(response)}\n")
        output.flush()


def run_script(
    *,
    address: str,
    controller: ArmController,
    script_path: str,
    output: TextIO,
    log_path: str | None = None,
) -> None:
    try:
        script_file = open(script_path, "r", encoding="utf-8")
    except OSError as error:
        raise ValueError(f"failed to read {script_path}: {error}") from error

    try:
        log_file = open_log_file(log_path)
        start_time = time.monotonic()
        try:
            for raw_line in script_file:
                invocation = parse_session_line(raw_line, address)
                if invocation is None:
                    continue

                response = execute_invocation(invocation, controller)
                rendered = render_response(response)
                output.write(f"{rendered}\n")
                output.flush()

                if log_file is not None:
                    event = {
                        "t_ms": int((time.monotonic() - start_time) * 1000),
                        "line": raw_line.strip(),
                        "response": response,
                    }
                    log_file.write(f"{render_response(event)}\n")
                    log_file.flush()
        finally:
            if log_file is not None:
                log_file.close()
    finally:
        script_file.close()


def open_log_file(log_path: str | None) -> TextIO | None:
    if log_path is None:
        return None

    log_file_path = Path(log_path)
    try:
        log_file_path.parent.mkdir(parents=True, exist_ok=True)
        return log_file_path.open("w", encoding="utf-8")
    except OSError as error:
        raise ValueError(f"failed to open {log_path}: {error}") from error


def read_session_event(lines: TextIO, timeout_s: float = 0.5) -> SessionEvent:
    try:
        ready, _, _ = select.select([lines], [], [], timeout_s)
        if not ready:
            return SessionEvent(kind="idle")
    except (OSError, ValueError):
        line = lines.readline()
        if line == "":
            return SessionEvent(kind="eof")
        return SessionEvent(kind="line", line=line)

    line = lines.readline()
    if line == "":
        return SessionEvent(kind="eof")

    return SessionEvent(kind="line", line=line)


if __name__ == "__main__":
    main()
