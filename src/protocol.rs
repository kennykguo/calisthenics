// ABOUTME: Parses the ASCII command protocol used between the Jetson and STM32 firmware.
// ABOUTME: Defines typed commands for arming, heartbeat, fault recovery, and joint moves.

use crate::Joint;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Command {
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

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ParseError {
    Empty,
    UnknownCommand,
    MissingArgument,
    UnknownJoint,
    InvalidAngle,
}

pub fn parse_command(input: &str) -> Result<Command, ParseError> {
    let mut parts = input.split_ascii_whitespace();
    let command = parts.next().ok_or(ParseError::Empty)?;

    match command {
        "ARM" => Ok(Command::Arm),
        "DISARM" => Ok(Command::Disarm),
        "HEARTBEAT" => Ok(Command::Heartbeat),
        "STATUS" => Ok(Command::Status),
        "CLEAR_FAULT" => Ok(Command::ClearFault),
        "HOME" => Ok(Command::Home),
        "OPEN_GRIPPER" => Ok(Command::OpenGripper),
        "CLOSE_GRIPPER" => Ok(Command::CloseGripper),
        "MOVE" => {
            let joint = parse_joint(parts.next().ok_or(ParseError::MissingArgument)?)?;
            let angle_deg = parts
                .next()
                .ok_or(ParseError::MissingArgument)?
                .parse::<u16>()
                .map_err(|_| ParseError::InvalidAngle)?;

            Ok(Command::MoveJoint { joint, angle_deg })
        }
        _ => Err(ParseError::UnknownCommand),
    }
}

fn parse_joint(input: &str) -> Result<Joint, ParseError> {
    match input {
        "BASE" => Ok(Joint::Base),
        "SHOULDER" => Ok(Joint::Shoulder),
        "ELBOW" => Ok(Joint::Elbow),
        "GRIPPER" => Ok(Joint::Gripper),
        _ => Err(ParseError::UnknownJoint),
    }
}
