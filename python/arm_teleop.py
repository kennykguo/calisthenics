# ABOUTME: Provides a keyboard teleop loop on top of the typed arm controller for quick manual positioning.
# ABOUTME: Maps single-key inputs into safe host-side arm commands while keeping the firmware watchdog alive.

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
import select
import sys
import termios
import tty
from typing import Any, Iterator, TextIO

from arm_controller import ArmController, ArmControllerError
from arm_operator import render_response, render_status
from arm_service import ArmServiceError


@dataclass(frozen=True)
class KeyEvent:
    kind: str
    key: str = ""


@dataclass(frozen=True)
class StepSizes:
    joint_deg: int = 5
    gripper_deg: int = 10


def main() -> None:
    if len(sys.argv) != 2:
        print(usage(), file=sys.stderr)
        raise SystemExit(1)

    address = sys.argv[1]
    try:
        with ArmController.connect(address) as controller:
            with raw_terminal(sys.stdin):
                run_teleop(controller=controller, output=sys.stdout, input_stream=sys.stdin)
    except (ArmControllerError, ArmServiceError, ValueError) as error:
        print(render_response({"kind": "error", "message": str(error)}), file=sys.stderr)
        raise SystemExit(1)


def usage() -> str:
    return "usage: python3 python/arm_teleop.py <address>"


def run_teleop(
    *,
    controller: ArmController,
    output: TextIO,
    input_stream: TextIO | None = None,
    next_event: Any | None = None,
) -> None:
    if next_event is None:
        if input_stream is None:
            raise ValueError("input_stream is required when next_event is not provided")
        next_event = lambda: read_key_event(input_stream)

    step_sizes = StepSizes()
    output.write(f"{help_text()}\n")
    output.flush()

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
            step_sizes, response, should_exit = handle_keypress(
                event.key,
                controller=controller,
                step_sizes=step_sizes,
            )
        except (ArmControllerError, ArmServiceError, ValueError) as error:
            response = {"kind": "error", "message": str(error)}
            should_exit = False

        if response is not None:
            output.write(f"{render_response(response)}\n")
            output.flush()

        if should_exit:
            return


def handle_keypress(
    key: str,
    *,
    controller: ArmController,
    step_sizes: StepSizes,
) -> tuple[StepSizes, dict[str, Any] | None, bool]:
    if key in {"\n", "\r", ""}:
        return step_sizes, None, False

    if key == "q":
        return step_sizes, None, True

    if key == "?":
        return step_sizes, {"kind": "teleop-help", "text": help_text()}, False

    if key == "+":
        next_sizes = StepSizes(
            joint_deg=step_sizes.joint_deg + 5,
            gripper_deg=step_sizes.gripper_deg + 5,
        )
        return next_sizes, render_step_sizes(next_sizes), False

    if key == "-":
        next_sizes = StepSizes(
            joint_deg=max(1, step_sizes.joint_deg - 5),
            gripper_deg=max(1, step_sizes.gripper_deg - 5),
        )
        return next_sizes, render_step_sizes(next_sizes), False

    immediate_commands = {
        "c": lambda: {"kind": "ok", "command": "clear-fault"} if not controller.clear_fault() else None,
        "r": lambda: {"kind": "ok", "command": "arm"} if not controller.arm() else None,
        "x": lambda: {"kind": "ok", "command": "disarm"} if not controller.disarm() else None,
        "t": lambda: render_status(controller.status()),
        "h": lambda: render_status(controller.home()),
        "o": lambda: render_status(controller.open_gripper()),
        "p": lambda: render_status(controller.close_gripper()),
    }
    if key in immediate_commands:
        return step_sizes, immediate_commands[key](), False

    joint_steps = {
        "a": ("base", -step_sizes.joint_deg),
        "d": ("base", step_sizes.joint_deg),
        "w": ("shoulder", step_sizes.joint_deg),
        "s": ("shoulder", -step_sizes.joint_deg),
        "i": ("elbow", step_sizes.joint_deg),
        "k": ("elbow", -step_sizes.joint_deg),
        "j": ("gripper", -step_sizes.gripper_deg),
        "l": ("gripper", step_sizes.gripper_deg),
    }
    if key in joint_steps:
        joint, delta_deg = joint_steps[key]
        return step_sizes, render_status(controller.step_joint(joint, delta_deg)), False

    raise ValueError(f"unsupported teleop key `{key}`")


def render_step_sizes(step_sizes: StepSizes) -> dict[str, Any]:
    return {
        "kind": "teleop",
        "joint_step_deg": step_sizes.joint_deg,
        "gripper_step_deg": step_sizes.gripper_deg,
    }


def help_text() -> str:
    return (
        "keys: c clear-fault, r arm, x disarm, h home, o open, p close, "
        "a/d base, w/s shoulder, i/k elbow, j/l gripper, +/- step, t status, q quit, ? help"
    )


def read_key_event(input_stream: TextIO, timeout_s: float = 0.5) -> KeyEvent:
    try:
        ready, _, _ = select.select([input_stream], [], [], timeout_s)
        if not ready:
            return KeyEvent(kind="idle")
    except (OSError, ValueError):
        key = input_stream.read(1)
        if key == "":
            return KeyEvent(kind="eof")
        return KeyEvent(kind="key", key=key)

    key = input_stream.read(1)
    if key == "":
        return KeyEvent(kind="eof")
    return KeyEvent(kind="key", key=key)


@contextmanager
def raw_terminal(stream: TextIO) -> Iterator[None]:
    if not hasattr(stream, "isatty") or not stream.isatty():
        yield
        return

    file_descriptor = stream.fileno()
    original_attributes = termios.tcgetattr(file_descriptor)
    try:
        tty.setcbreak(file_descriptor)
        yield
    finally:
        termios.tcsetattr(file_descriptor, termios.TCSADRAIN, original_attributes)


if __name__ == "__main__":
    main()
