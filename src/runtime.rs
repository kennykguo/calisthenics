// ABOUTME: Connects incoming ASCII command lines to the arm controller and status reporting.
// ABOUTME: Provides transport-neutral ACK, ERR, and STATUS responses for firmware integration.

use core::fmt::{self, Write};

use crate::{
    ArmConfig, ArmController, CommandError, ParseError, Snapshot, StatusFrame, parse_command,
};

pub struct Runtime {
    controller: ArmController,
    next_status_seq: u32,
}

impl Runtime {
    pub fn new(config: ArmConfig) -> Self {
        Self {
            controller: ArmController::new(config),
            next_status_seq: 0,
        }
    }

    pub fn process_line(&mut self, now_ms: u32, line: &str) -> Response {
        match parse_command(line) {
            Ok(crate::Command::Status) => self.status(),
            Ok(command) => match self.controller.handle_command(now_ms, command) {
                Ok(()) => Response::Ack,
                Err(error) => Response::Error(ResponseError::Command(error)),
            },
            Err(error) => Response::Error(ResponseError::Parse(error)),
        }
    }

    pub fn tick(&mut self, now_ms: u32) {
        self.controller.tick(now_ms);
    }

    pub fn observe_servo_voltage_mv(&mut self, servo_voltage_mv: u16) {
        self.controller.observe_servo_voltage_mv(servo_voltage_mv);
    }

    pub fn status(&mut self) -> Response {
        let frame = self.controller.status_frame(self.next_status_seq);
        self.next_status_seq = self.next_status_seq.wrapping_add(1);
        Response::Status(frame)
    }

    pub fn snapshot(&self) -> Snapshot {
        self.controller.snapshot()
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Response {
    Ack,
    Error(ResponseError),
    Status(StatusFrame),
}

impl Response {
    pub fn write_to<W: Write>(&self, writer: &mut W) -> fmt::Result {
        match self {
            Response::Ack => writer.write_str("ACK"),
            Response::Error(error) => error.write_to(writer),
            Response::Status(frame) => frame.write_to(writer),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ResponseError {
    Parse(ParseError),
    Command(CommandError),
    Transport(TransportError),
}

impl ResponseError {
    fn write_to<W: Write>(&self, writer: &mut W) -> fmt::Result {
        match self {
            ResponseError::Parse(error) => {
                write!(writer, "ERR kind=PARSE code={}", format_parse_error(*error))
            }
            ResponseError::Command(error) => {
                write!(
                    writer,
                    "ERR kind=COMMAND code={}",
                    format_command_error(*error)
                )
            }
            ResponseError::Transport(error) => {
                write!(
                    writer,
                    "ERR kind=TRANSPORT code={}",
                    format_transport_error(*error)
                )
            }
        }
    }
}

fn format_parse_error(error: ParseError) -> &'static str {
    match error {
        ParseError::Empty => "EMPTY",
        ParseError::UnknownCommand => "UNKNOWN_COMMAND",
        ParseError::MissingArgument => "MISSING_ARGUMENT",
        ParseError::UnknownJoint => "UNKNOWN_JOINT",
        ParseError::InvalidAngle => "INVALID_ANGLE",
    }
}

fn format_command_error(error: CommandError) -> &'static str {
    match error {
        CommandError::Disarmed => "DISARMED",
        CommandError::Faulted(crate::Fault::HeartbeatTimedOut) => "FAULT_HEARTBEAT_TIMEOUT",
        CommandError::Faulted(crate::Fault::Brownout) => "FAULT_BROWNOUT",
        CommandError::MustDisarmBeforeClearingFault => "MUST_DISARM_BEFORE_CLEARING_FAULT",
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TransportError {
    LineTooLong,
    InvalidEncoding,
}

fn format_transport_error(error: TransportError) -> &'static str {
    match error {
        TransportError::LineTooLong => "LINE_TOO_LONG",
        TransportError::InvalidEncoding => "INVALID_ENCODING",
    }
}
