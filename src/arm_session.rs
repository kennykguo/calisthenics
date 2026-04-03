// ABOUTME: Runs typed host-side sessions against the STM32 arm controller over a serial link.
// ABOUTME: Renders requests, parses status snapshots, and keeps motion commands alive with heartbeats.

use std::{
    fs::{File, OpenOptions},
    io::{Read, Write},
    process::Command as ProcessCommand,
    thread,
    time::{Duration, Instant},
};

use crate::{Fault, Joint, Snapshot, StatusFrame};

const COMMAND_ACK_TIMEOUT_MS: u64 = 2_000;
const COMMAND_SETTLE_TIMEOUT_MS: u64 = 30_000;
const HEARTBEAT_PERIOD_MS: u64 = 500;
const INITIAL_DRAIN_MS: u64 = 150;
const POLL_DELAY_MS: u64 = 10;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum CommandKind {
    Immediate,
    WaitForStatus,
    WaitForIdle,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ArmStatus {
    pub seq: u32,
    pub snapshot: Snapshot,
}

impl ArmStatus {
    pub fn to_line(&self) -> String {
        let mut line = String::new();
        StatusFrame {
            seq: self.seq,
            snapshot: self.snapshot,
        }
        .write_to(&mut line)
        .expect("status formatting should fit in String");
        line
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ArmRequest {
    Arm,
    Disarm,
    Heartbeat,
    Status,
    ClearFault,
    Home,
    OpenGripper,
    CloseGripper,
    MoveJoint { joint: Joint, angle_deg: u16 },
}

impl ArmRequest {
    pub fn to_line(&self) -> String {
        match self {
            Self::Arm => String::from("ARM"),
            Self::Disarm => String::from("DISARM"),
            Self::Heartbeat => String::from("HEARTBEAT"),
            Self::Status => String::from("STATUS"),
            Self::ClearFault => String::from("CLEAR_FAULT"),
            Self::Home => String::from("HOME"),
            Self::OpenGripper => String::from("OPEN_GRIPPER"),
            Self::CloseGripper => String::from("CLOSE_GRIPPER"),
            Self::MoveJoint { joint, angle_deg } => {
                format!("MOVE {} {angle_deg}", format_joint(*joint))
            }
        }
    }

    fn kind(&self) -> CommandKind {
        match self {
            Self::Status => CommandKind::WaitForStatus,
            Self::Home | Self::OpenGripper | Self::CloseGripper | Self::MoveJoint { .. } => {
                CommandKind::WaitForIdle
            }
            _ => CommandKind::Immediate,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ArmPose {
    pub base: u16,
    pub shoulder: u16,
    pub elbow: u16,
    pub gripper: u16,
}

impl ArmPose {
    pub fn requests(self) -> Vec<ArmRequest> {
        vec![
            ArmRequest::MoveJoint {
                joint: Joint::Base,
                angle_deg: self.base,
            },
            ArmRequest::MoveJoint {
                joint: Joint::Shoulder,
                angle_deg: self.shoulder,
            },
            ArmRequest::MoveJoint {
                joint: Joint::Elbow,
                angle_deg: self.elbow,
            },
            ArmRequest::MoveJoint {
                joint: Joint::Gripper,
                angle_deg: self.gripper,
            },
        ]
    }
}

pub struct ArmSession {
    serial: File,
    line_buffer: SerialLineBuffer,
    read_buffer: [u8; 64],
}

impl ArmSession {
    pub fn connect(device_path: &str) -> Result<Self, String> {
        configure_serial_device(device_path)?;

        let mut serial = OpenOptions::new()
            .read(true)
            .write(true)
            .open(device_path)
            .map_err(|error| format!("failed to open {device_path}: {error}"))?;
        let mut line_buffer = SerialLineBuffer::new();
        let mut read_buffer = [0_u8; 64];

        drain_for_window(
            &mut serial,
            &mut line_buffer,
            &mut read_buffer,
            Duration::from_millis(INITIAL_DRAIN_MS),
        )?;
        line_buffer.reset_for_quiet_link();

        Ok(Self {
            serial,
            line_buffer,
            read_buffer,
        })
    }

    pub fn run_request(&mut self, request: &ArmRequest) -> Result<Option<ArmStatus>, String> {
        let line = request.to_line();
        self.run_command(&line, request.kind())
    }

    pub fn run_text_command(&mut self, command: &str) -> Result<Option<ArmStatus>, String> {
        self.run_command(command, classify_command(command))
    }

    pub fn clear_fault(&mut self) -> Result<(), String> {
        self.run_request(&ArmRequest::ClearFault).map(|_| ())
    }

    pub fn arm(&mut self) -> Result<(), String> {
        self.run_request(&ArmRequest::Arm).map(|_| ())
    }

    pub fn disarm(&mut self) -> Result<(), String> {
        self.run_request(&ArmRequest::Disarm).map(|_| ())
    }

    pub fn home(&mut self) -> Result<ArmStatus, String> {
        self.run_request(&ArmRequest::Home)?
            .ok_or_else(|| String::from("HOME did not return a final status"))
    }

    pub fn open_gripper(&mut self) -> Result<ArmStatus, String> {
        self.run_request(&ArmRequest::OpenGripper)?
            .ok_or_else(|| String::from("OPEN_GRIPPER did not return a final status"))
    }

    pub fn close_gripper(&mut self) -> Result<ArmStatus, String> {
        self.run_request(&ArmRequest::CloseGripper)?
            .ok_or_else(|| String::from("CLOSE_GRIPPER did not return a final status"))
    }

    pub fn move_joint(&mut self, joint: Joint, angle_deg: u16) -> Result<ArmStatus, String> {
        self.run_request(&ArmRequest::MoveJoint { joint, angle_deg })?
            .ok_or_else(|| {
                format!(
                    "MOVE {} {angle_deg} did not return a final status",
                    format_joint(joint)
                )
            })
    }

    pub fn status(&mut self) -> Result<ArmStatus, String> {
        self.run_request(&ArmRequest::Status)?
            .ok_or_else(|| String::from("STATUS did not return a status snapshot"))
    }

    pub fn move_pose(&mut self, pose: ArmPose) -> Result<ArmStatus, String> {
        let mut last_status = None;

        for request in pose.requests() {
            last_status = self.run_request(&request)?;
        }

        last_status.ok_or_else(|| String::from("pose did not return a final status"))
    }

    pub fn wait(&mut self, duration_ms: u64) -> Result<ArmStatus, String> {
        let deadline = Instant::now() + Duration::from_millis(duration_ms);
        let mut last_status = None;
        let mut next_poll_at = Instant::now();

        loop {
            let now = Instant::now();
            if last_status.is_none() || now >= next_poll_at {
                self.run_request(&ArmRequest::Heartbeat)?;
                let status = self
                    .run_request(&ArmRequest::Status)?
                    .ok_or_else(|| String::from("STATUS did not return a status snapshot"))?;

                if status.snapshot.fault.is_some() {
                    return Err(format!("board faulted while waiting: {}", status.to_line()));
                }

                last_status = Some(status);
                next_poll_at = Instant::now() + Duration::from_millis(HEARTBEAT_PERIOD_MS);
            }

            if Instant::now() >= deadline {
                break;
            }

            let sleep_until = core::cmp::min(deadline, next_poll_at);
            let remaining = sleep_until.saturating_duration_since(Instant::now());
            if !remaining.is_zero() {
                thread::sleep(remaining);
            }
        }

        last_status.ok_or_else(|| String::from("wait did not produce a status snapshot"))
    }

    fn run_command(
        &mut self,
        command: &str,
        command_kind: CommandKind,
    ) -> Result<Option<ArmStatus>, String> {
        let command_deadline = Instant::now() + Duration::from_millis(COMMAND_ACK_TIMEOUT_MS);
        let settle_deadline = Instant::now() + Duration::from_millis(COMMAND_SETTLE_TIMEOUT_MS);
        let mut command_acknowledged = false;
        let mut next_status_poll_at = Instant::now();

        write_line(&mut self.serial, command)?;

        loop {
            if command_acknowledged
                && command_kind == CommandKind::WaitForIdle
                && Instant::now() >= next_status_poll_at
            {
                write_line(&mut self.serial, "HEARTBEAT")?;
                write_line(&mut self.serial, "STATUS")?;
                next_status_poll_at = Instant::now() + Duration::from_millis(HEARTBEAT_PERIOD_MS);
            }

            let mut read_any_bytes = false;

            match self.serial.read(&mut self.read_buffer) {
                Ok(0) => {}
                Ok(read_len) => {
                    read_any_bytes = true;

                    for byte in &self.read_buffer[..read_len] {
                        if let Some(line) = self.line_buffer.push_byte(*byte) {
                            if line == "ACK" {
                                if command_kind == CommandKind::WaitForStatus {
                                    continue;
                                }

                                if !command_acknowledged {
                                    command_acknowledged = true;

                                    if command_kind == CommandKind::Immediate {
                                        return Ok(None);
                                    }
                                }
                                continue;
                            }

                            if line.starts_with("ERR ") {
                                return Err(format!("board rejected `{command}`: {line}"));
                            }

                            if let Some(status) = parse_status_line(&line) {
                                if command_kind == CommandKind::WaitForStatus {
                                    return Ok(Some(status));
                                }

                                if status.snapshot.fault.is_some() {
                                    return Err(format!(
                                        "board faulted while running `{command}`: {line}"
                                    ));
                                }

                                if command_acknowledged
                                    && command_kind == CommandKind::WaitForIdle
                                    && !status.snapshot.moving
                                {
                                    return Ok(Some(status));
                                }
                            }
                        }
                    }
                }
                Err(error) if error.kind() == std::io::ErrorKind::Interrupted => {}
                Err(error) => {
                    return Err(format!("serial read error: {error}"));
                }
            }

            if command_kind == CommandKind::WaitForStatus && Instant::now() >= command_deadline {
                return Err(format!("timed out waiting for `{command}` to respond"));
            }

            if command_kind != CommandKind::WaitForStatus
                && !command_acknowledged
                && Instant::now() >= command_deadline
            {
                return Err(format!("timed out waiting for `{command}` to acknowledge"));
            }

            if command_acknowledged
                && command_kind == CommandKind::WaitForIdle
                && Instant::now() >= settle_deadline
            {
                return Err(format!("timed out waiting for `{command}` to settle"));
            }

            if !read_any_bytes {
                thread::sleep(Duration::from_millis(POLL_DELAY_MS));
            }
        }
    }
}

pub fn parse_status_line(line: &str) -> Option<ArmStatus> {
    if !line.starts_with("STATUS ") {
        return None;
    }

    let mut seq = None;
    let mut armed = None;
    let mut fault = None;
    let mut moving = None;
    let mut active_joint = None;
    let mut servo_voltage_mv = None;
    let mut current_deg = None;
    let mut target_deg = None;

    for part in line.split_ascii_whitespace().skip(1) {
        if let Some(value) = part.strip_prefix("seq=") {
            seq = value.parse::<u32>().ok();
        } else if let Some(value) = part.strip_prefix("armed=") {
            armed = parse_bool_flag(value);
        } else if let Some(value) = part.strip_prefix("fault=") {
            fault = parse_fault(value);
        } else if let Some(value) = part.strip_prefix("moving=") {
            moving = parse_bool_flag(value);
        } else if let Some(value) = part.strip_prefix("active=") {
            active_joint = parse_active_joint(value);
        } else if let Some(value) = part.strip_prefix("voltage_mv=") {
            servo_voltage_mv = parse_voltage_mv(value);
        } else if let Some(value) = part.strip_prefix("current=") {
            current_deg = parse_joint_array(value);
        } else if let Some(value) = part.strip_prefix("target=") {
            target_deg = parse_joint_array(value);
        }
    }

    Some(ArmStatus {
        seq: seq?,
        snapshot: Snapshot {
            armed: armed?,
            fault: fault?,
            moving: moving?,
            active_joint: active_joint?,
            servo_voltage_mv: servo_voltage_mv?,
            current_deg: current_deg?,
            target_deg: target_deg?,
        },
    })
}

fn classify_command(command: &str) -> CommandKind {
    match command.split_ascii_whitespace().next() {
        Some("MOVE" | "HOME" | "OPEN_GRIPPER" | "CLOSE_GRIPPER") => CommandKind::WaitForIdle,
        Some("STATUS") => CommandKind::WaitForStatus,
        _ => CommandKind::Immediate,
    }
}

fn configure_serial_device(device_path: &str) -> Result<(), String> {
    let status = ProcessCommand::new("stty")
        .args([
            "-F",
            device_path,
            "115200",
            "raw",
            "-echo",
            "min",
            "0",
            "time",
            "1",
        ])
        .status()
        .map_err(|error| format!("failed to run stty for {device_path}: {error}"))?;

    if status.success() {
        Ok(())
    } else {
        Err(format!(
            "stty failed for {device_path} with status {status}"
        ))
    }
}

fn write_line(serial: &mut File, line: &str) -> Result<(), String> {
    serial
        .write_all(line.as_bytes())
        .and_then(|_| serial.write_all(b"\r"))
        .and_then(|_| serial.flush())
        .map_err(|error| format!("failed to write to serial device: {error}"))
}

fn drain_for_window(
    serial: &mut File,
    line_buffer: &mut SerialLineBuffer,
    read_buffer: &mut [u8; 64],
    drain_for: Duration,
) -> Result<(), String> {
    let drain_started_at = Instant::now();

    loop {
        match serial.read(read_buffer) {
            Ok(0) => {
                if Instant::now().duration_since(drain_started_at) >= drain_for {
                    return Ok(());
                }

                thread::sleep(Duration::from_millis(POLL_DELAY_MS));
            }
            Ok(read_len) => {
                for byte in &read_buffer[..read_len] {
                    let _ = line_buffer.push_byte(*byte);
                }

                if Instant::now().duration_since(drain_started_at) >= drain_for {
                    return Ok(());
                }
            }
            Err(error) if error.kind() == std::io::ErrorKind::Interrupted => {}
            Err(error) => {
                return Err(format!("serial read error: {error}"));
            }
        }
    }
}

fn parse_bool_flag(value: &str) -> Option<bool> {
    match value {
        "0" => Some(false),
        "1" => Some(true),
        _ => None,
    }
}

fn parse_fault(value: &str) -> Option<Option<Fault>> {
    match value {
        "NONE" => Some(None),
        "HEARTBEAT_TIMEOUT" => Some(Some(Fault::HeartbeatTimedOut)),
        "BROWNOUT" => Some(Some(Fault::Brownout)),
        _ => None,
    }
}

fn parse_active_joint(value: &str) -> Option<Option<Joint>> {
    match value {
        "NONE" => Some(None),
        "BASE" => Some(Some(Joint::Base)),
        "SHOULDER" => Some(Some(Joint::Shoulder)),
        "ELBOW" => Some(Some(Joint::Elbow)),
        "GRIPPER" => Some(Some(Joint::Gripper)),
        _ => None,
    }
}

fn parse_voltage_mv(value: &str) -> Option<Option<u16>> {
    let voltage_mv = value.parse::<u16>().ok()?;
    if voltage_mv == 0 {
        Some(None)
    } else {
        Some(Some(voltage_mv))
    }
}

fn parse_joint_array(value: &str) -> Option<[u16; 4]> {
    let mut parts = value.split(',');
    Some([
        parts.next()?.parse().ok()?,
        parts.next()?.parse().ok()?,
        parts.next()?.parse().ok()?,
        parts.next()?.parse().ok()?,
    ])
}

fn format_joint(joint: Joint) -> &'static str {
    match joint {
        Joint::Base => "BASE",
        Joint::Shoulder => "SHOULDER",
        Joint::Elbow => "ELBOW",
        Joint::Gripper => "GRIPPER",
    }
}

struct SerialLineBuffer {
    bytes: Vec<u8>,
    aligned: bool,
    ignore_next_lf: bool,
}

impl SerialLineBuffer {
    fn new() -> Self {
        Self {
            bytes: Vec::new(),
            aligned: false,
            ignore_next_lf: false,
        }
    }

    fn push_byte(&mut self, byte: u8) -> Option<String> {
        if !self.aligned {
            if byte == b'\r' {
                self.aligned = true;
                self.ignore_next_lf = true;
            } else if byte == b'\n' {
                self.aligned = true;
            }
            return None;
        }

        if self.ignore_next_lf {
            self.ignore_next_lf = false;
            if byte == b'\n' {
                return None;
            }
        }

        if byte == b'\r' {
            self.ignore_next_lf = true;
            return self.finish_line();
        }

        if byte == b'\n' {
            return self.finish_line();
        }

        self.bytes.push(byte);
        None
    }

    fn finish_line(&mut self) -> Option<String> {
        if self.bytes.is_empty() {
            return None;
        }

        let line = String::from_utf8_lossy(&self.bytes).into_owned();
        self.bytes.clear();
        Some(line)
    }

    fn reset_for_quiet_link(&mut self) {
        self.bytes.clear();
        self.aligned = true;
        self.ignore_next_lf = false;
    }
}

#[cfg(test)]
mod tests {
    use super::{ArmStatus, CommandKind, SerialLineBuffer, classify_command, parse_status_line};
    use crate::{Fault, Joint, Snapshot};

    #[test]
    fn classifies_motion_commands_as_wait_for_idle() {
        assert_eq!(classify_command("MOVE BASE 150"), CommandKind::WaitForIdle);
        assert_eq!(classify_command("HOME"), CommandKind::WaitForIdle);
        assert_eq!(classify_command("OPEN_GRIPPER"), CommandKind::WaitForIdle);
        assert_eq!(classify_command("CLOSE_GRIPPER"), CommandKind::WaitForIdle);
    }

    #[test]
    fn classifies_status_commands_as_wait_for_status() {
        assert_eq!(classify_command("STATUS"), CommandKind::WaitForStatus);
    }

    #[test]
    fn classifies_control_commands_as_immediate() {
        assert_eq!(classify_command("ARM"), CommandKind::Immediate);
        assert_eq!(classify_command("DISARM"), CommandKind::Immediate);
        assert_eq!(classify_command("CLEAR_FAULT"), CommandKind::Immediate);
        assert_eq!(classify_command("HEARTBEAT"), CommandKind::Immediate);
    }

    #[test]
    fn discards_the_initial_partial_line_until_the_first_terminator() {
        let mut line_buffer = SerialLineBuffer::new();
        let mut lines = Vec::new();

        for byte in b"ALF STATUS\rSTATUS seq=1\r" {
            if let Some(line) = line_buffer.push_byte(*byte) {
                lines.push(line);
            }
        }

        assert_eq!(lines, vec![String::from("STATUS seq=1")]);
    }

    #[test]
    fn assembles_crlf_terminated_lines_after_alignment() {
        let mut line_buffer = SerialLineBuffer::new();
        let mut lines = Vec::new();

        for byte in b"\rACK\r\nSTATUS seq=1\r\n" {
            if let Some(line) = line_buffer.push_byte(*byte) {
                lines.push(line);
            }
        }

        assert_eq!(
            lines,
            vec![String::from("ACK"), String::from("STATUS seq=1")]
        );
    }

    #[test]
    fn can_capture_the_first_line_after_a_quiet_link_reset() {
        let mut line_buffer = SerialLineBuffer::new();
        let mut lines = Vec::new();

        line_buffer.reset_for_quiet_link();

        for byte in b"ACK\r" {
            if let Some(line) = line_buffer.push_byte(*byte) {
                lines.push(line);
            }
        }

        assert_eq!(lines, vec![String::from("ACK")]);
    }

    #[test]
    fn parses_status_lines_into_arm_status() {
        assert_eq!(
            parse_status_line(
                "STATUS seq=8 armed=1 fault=BROWNOUT moving=0 active=NONE voltage_mv=4600 current=90,90,100,120 target=90,90,100,120"
            ),
            Some(ArmStatus {
                seq: 8,
                snapshot: Snapshot {
                    armed: true,
                    fault: Some(Fault::Brownout),
                    moving: false,
                    active_joint: None,
                    servo_voltage_mv: Some(4600),
                    current_deg: [90, 90, 100, 120],
                    target_deg: [90, 90, 100, 120],
                }
            })
        );
    }

    #[test]
    fn ignores_non_status_lines_when_parsing_status() {
        assert_eq!(parse_status_line("ACK"), None);
        assert_eq!(parse_status_line("ERR kind=COMMAND code=DISARMED"), None);
    }

    #[test]
    fn parses_active_joint_and_heartbeat_fault() {
        assert_eq!(
            parse_status_line(
                "STATUS seq=7 armed=1 fault=HEARTBEAT_TIMEOUT moving=1 active=ELBOW voltage_mv=5000 current=91,90,100,120 target=100,90,100,120"
            ),
            Some(ArmStatus {
                seq: 7,
                snapshot: Snapshot {
                    armed: true,
                    fault: Some(Fault::HeartbeatTimedOut),
                    moving: true,
                    active_joint: Some(Joint::Elbow),
                    servo_voltage_mv: Some(5000),
                    current_deg: [91, 90, 100, 120],
                    target_deg: [100, 90, 100, 120],
                }
            })
        );
    }
}
