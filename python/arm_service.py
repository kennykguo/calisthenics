# ABOUTME: Talks to the local arm daemon over newline-delimited JSON with typed Python helpers.
# ABOUTME: Parses limits, status, ok, and error replies so host-side control code can stay small.

from __future__ import annotations

from dataclasses import dataclass
import json
import socket
from typing import Any

FAST_REPLY_TIMEOUT_S = 2.0
MOTION_REPLY_TIMEOUT_S = 35.0


class ArmServiceError(RuntimeError):
    pass


@dataclass(frozen=True)
class JointLimits:
    min_deg: int
    max_deg: int
    home_deg: int


@dataclass(frozen=True)
class GripperLimits:
    min_deg: int
    max_deg: int
    home_deg: int
    closed_deg: int


@dataclass(frozen=True)
class ArmLimits:
    base: JointLimits
    shoulder: JointLimits
    elbow: JointLimits
    gripper: GripperLimits
    heartbeat_timeout_ms: int
    servo_voltage_min_mv: int


@dataclass(frozen=True)
class Snapshot:
    armed: bool
    fault: str | None
    moving: bool
    active_joint: str | None
    servo_voltage_mv: int | None
    current_deg: list[int]
    target_deg: list[int]


@dataclass(frozen=True)
class ArmStatus:
    seq: int
    snapshot: Snapshot


class ArmServiceClient:
    def __init__(self, sock: socket.socket, timeout_s: float) -> None:
        self._socket = sock
        self._reader = sock.makefile("r", encoding="utf-8", newline="\n")
        self._writer = sock.makefile("w", encoding="utf-8", newline="\n")
        self._fast_timeout_s = timeout_s
        self._next_request_id = 1

    @classmethod
    def connect(cls, address: str, timeout_s: float = FAST_REPLY_TIMEOUT_S) -> "ArmServiceClient":
        host, port = _parse_address(address)
        sock = socket.create_connection((host, port), timeout=timeout_s)
        sock.settimeout(timeout_s)
        return cls(sock, timeout_s)

    def close(self) -> None:
        self._reader.close()
        self._writer.close()
        self._socket.close()

    def __enter__(self) -> "ArmServiceClient":
        return self

    def __exit__(self, exc_type: object, exc: object, exc_tb: object) -> None:
        self.close()

    def limits(self) -> ArmLimits:
        payload = self._request({"command": "limits"})
        _expect_kind(payload, "limits")
        return _parse_limits(payload)

    def status(self) -> ArmStatus:
        payload = self._request({"command": "status"})
        _expect_kind(payload, "status")
        return _parse_status(payload)

    def clear_fault(self) -> None:
        payload = self._request({"command": "clear-fault"})
        _expect_kind(payload, "ok")

    def arm(self) -> None:
        payload = self._request({"command": "arm"})
        _expect_kind(payload, "ok")

    def disarm(self) -> None:
        payload = self._request({"command": "disarm"})
        _expect_kind(payload, "ok")

    def home(self) -> ArmStatus:
        payload = self._request({"command": "home"})
        _expect_kind(payload, "status")
        return _parse_status(payload)

    def open_gripper(self) -> ArmStatus:
        payload = self._request({"command": "open"})
        _expect_kind(payload, "status")
        return _parse_status(payload)

    def close_gripper(self) -> ArmStatus:
        payload = self._request({"command": "close"})
        _expect_kind(payload, "status")
        return _parse_status(payload)

    def wait(self, duration_ms: int) -> ArmStatus:
        payload = self._request({"command": "wait", "duration_ms": duration_ms})
        _expect_kind(payload, "status")
        return _parse_status(payload)

    def move_joint(self, joint: str, angle_deg: int) -> ArmStatus:
        payload = self._request(
            {
                "command": "joint",
                "joint": joint.lower(),
                "angle_deg": angle_deg,
            }
        )
        _expect_kind(payload, "status")
        return _parse_status(payload)

    def move_pose(
        self, base: int, shoulder: int, elbow: int, gripper: int
    ) -> ArmStatus:
        payload = self._request(
            {
                "command": "pose",
                "base": base,
                "shoulder": shoulder,
                "elbow": elbow,
                "gripper": gripper,
            }
        )
        _expect_kind(payload, "status")
        return _parse_status(payload)

    def _request(self, payload: dict[str, Any]) -> dict[str, Any]:
        request_id = f"req-{self._next_request_id}"
        self._next_request_id += 1
        line = json.dumps({"id": request_id, **payload})
        command = str(payload.get("command", "unknown"))
        original_timeout_s = self._socket.gettimeout()
        self._socket.settimeout(self._response_timeout_s(command))
        try:
            self._writer.write(f"{line}\n")
            self._writer.flush()

            response_line = self._reader.readline()
            if not response_line:
                raise ArmServiceError("arm service closed the connection")

            response = json.loads(response_line)
            if response.get("id") != request_id:
                raise ArmServiceError(
                    f"arm service replied with mismatched id: expected `{request_id}`, got `{response.get('id', 'none')}`"
                )
            if response.get("kind") == "error":
                raise ArmServiceError(str(response.get("message", "unknown arm service error")))
            return response
        except TimeoutError as error:
            raise ArmServiceError(f"timed out waiting for `{command}` reply") from error
        finally:
            self._socket.settimeout(original_timeout_s)

    def _response_timeout_s(self, command: str) -> float:
        if command in {"home", "open", "close", "wait", "joint", "pose"}:
            return max(self._fast_timeout_s, MOTION_REPLY_TIMEOUT_S)
        return self._fast_timeout_s


def _parse_address(address: str) -> tuple[str, int]:
    host, separator, port_text = address.rpartition(":")
    if separator == "" or host == "" or port_text == "":
        raise ArmServiceError(
            f"arm service address must be HOST:PORT, got `{address}`"
        )

    try:
        return host, int(port_text)
    except ValueError as error:
        raise ArmServiceError(
            f"arm service port must be an integer, got `{port_text}`"
        ) from error


def _expect_kind(payload: dict[str, Any], expected_kind: str) -> None:
    actual_kind = payload.get("kind")
    if actual_kind != expected_kind:
        raise ArmServiceError(
            f"unexpected arm service reply kind: expected `{expected_kind}`, got `{actual_kind}`"
        )


def _parse_limits(payload: dict[str, Any]) -> ArmLimits:
    return ArmLimits(
        base=_parse_joint_limits(payload["base"]),
        shoulder=_parse_joint_limits(payload["shoulder"]),
        elbow=_parse_joint_limits(payload["elbow"]),
        gripper=_parse_gripper_limits(payload["gripper"]),
        heartbeat_timeout_ms=int(payload["heartbeat_timeout_ms"]),
        servo_voltage_min_mv=int(payload["servo_voltage_min_mv"]),
    )


def _parse_joint_limits(payload: dict[str, Any]) -> JointLimits:
    return JointLimits(
        min_deg=int(payload["min_deg"]),
        max_deg=int(payload["max_deg"]),
        home_deg=int(payload["home_deg"]),
    )


def _parse_gripper_limits(payload: dict[str, Any]) -> GripperLimits:
    return GripperLimits(
        min_deg=int(payload["min_deg"]),
        max_deg=int(payload["max_deg"]),
        home_deg=int(payload["home_deg"]),
        closed_deg=int(payload["closed_deg"]),
    )


def _parse_status(payload: dict[str, Any]) -> ArmStatus:
    fault = payload["fault"]
    active_joint = payload["active"]
    voltage_mv = int(payload["voltage_mv"])

    return ArmStatus(
        seq=int(payload["seq"]),
        snapshot=Snapshot(
            armed=bool(payload["armed"]),
            fault=None if fault == "NONE" else str(fault),
            moving=bool(payload["moving"]),
            active_joint=None if active_joint == "NONE" else str(active_joint),
            servo_voltage_mv=None if voltage_mv == 0 else voltage_mv,
            current_deg=[int(value) for value in payload["current"]],
            target_deg=[int(value) for value in payload["target"]],
        ),
    )
