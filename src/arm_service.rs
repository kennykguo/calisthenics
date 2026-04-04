// ABOUTME: Talks to the local arm service over newline-delimited JSON with typed request helpers.
// ABOUTME: Parses JSON replies into arm status, ok, and error results for host-side controller code.

use std::{
    io::{BufRead, BufReader, Write},
    net::TcpStream,
};

use crate::{ArmLimits, ArmPose, ArmStatus, Fault, GripperLimits, Joint, JointLimits, Snapshot};

pub struct ArmServiceClient {
    stream: TcpStream,
    reader: BufReader<TcpStream>,
    next_request_id: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ArmServiceReply {
    pub request_id: Option<String>,
    pub result: ArmServiceResult,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ArmServiceResult {
    Ok { command: String },
    Note { text: String },
    Limits { limits: ArmLimits },
    Status { command: String, status: ArmStatus },
    Error { command: String, message: String },
}

impl ArmServiceClient {
    pub fn connect(address: &str) -> Result<Self, String> {
        let stream = TcpStream::connect(address)
            .map_err(|error| format!("failed to connect to {address}: {error}"))?;
        let reader = BufReader::new(
            stream
                .try_clone()
                .map_err(|error| format!("failed to clone service socket: {error}"))?,
        );

        Ok(Self {
            stream,
            reader,
            next_request_id: 1,
        })
    }

    pub fn send_request(&mut self, command_json: &str) -> Result<ArmServiceReply, String> {
        let request_id = format!("req-{}", self.next_request_id);
        self.next_request_id = self.next_request_id.wrapping_add(1);
        let request_line = format!(
            "{{\"id\":\"{}\",{command_json}}}\n",
            escape_json(&request_id)
        );

        self.stream
            .write_all(request_line.as_bytes())
            .and_then(|_| self.stream.flush())
            .map_err(|error| format!("failed to write to arm service: {error}"))?;

        let mut response_line = String::new();
        let bytes_read = self
            .reader
            .read_line(&mut response_line)
            .map_err(|error| format!("failed to read from arm service: {error}"))?;

        if bytes_read == 0 {
            return Err(String::from("arm service closed the connection"));
        }

        let reply = parse_service_reply(response_line.trim_end())?;
        if reply.request_id.as_deref() != Some(request_id.as_str()) {
            return Err(format!(
                "arm service replied with mismatched id: expected `{request_id}`, got `{}`",
                reply.request_id.as_deref().unwrap_or("none")
            ));
        }

        Ok(reply)
    }

    pub fn status(&mut self) -> Result<ArmStatus, String> {
        match self.send_request("\"command\":\"status\"")?.result {
            ArmServiceResult::Status { status, .. } => Ok(status),
            other => Err(unexpected_reply("status", &other)),
        }
    }

    pub fn limits(&mut self) -> Result<ArmLimits, String> {
        match self.send_request("\"command\":\"limits\"")?.result {
            ArmServiceResult::Limits { limits } => Ok(limits),
            other => Err(unexpected_reply("limits", &other)),
        }
    }

    pub fn clear_fault(&mut self) -> Result<(), String> {
        match self.send_request("\"command\":\"clear-fault\"")?.result {
            ArmServiceResult::Ok { .. } => Ok(()),
            ArmServiceResult::Error { message, .. } => Err(message),
            other => Err(unexpected_reply("clear-fault", &other)),
        }
    }

    pub fn arm(&mut self) -> Result<(), String> {
        match self.send_request("\"command\":\"arm\"")?.result {
            ArmServiceResult::Ok { .. } => Ok(()),
            ArmServiceResult::Error { message, .. } => Err(message),
            other => Err(unexpected_reply("arm", &other)),
        }
    }

    pub fn disarm(&mut self) -> Result<(), String> {
        match self.send_request("\"command\":\"disarm\"")?.result {
            ArmServiceResult::Ok { .. } => Ok(()),
            ArmServiceResult::Error { message, .. } => Err(message),
            other => Err(unexpected_reply("disarm", &other)),
        }
    }

    pub fn home(&mut self) -> Result<ArmStatus, String> {
        match self.send_request("\"command\":\"home\"")?.result {
            ArmServiceResult::Status { status, .. } => Ok(status),
            ArmServiceResult::Error { message, .. } => Err(message),
            other => Err(unexpected_reply("home", &other)),
        }
    }

    pub fn open_gripper(&mut self) -> Result<ArmStatus, String> {
        match self.send_request("\"command\":\"open\"")?.result {
            ArmServiceResult::Status { status, .. } => Ok(status),
            ArmServiceResult::Error { message, .. } => Err(message),
            other => Err(unexpected_reply("open", &other)),
        }
    }

    pub fn close_gripper(&mut self) -> Result<ArmStatus, String> {
        match self.send_request("\"command\":\"close\"")?.result {
            ArmServiceResult::Status { status, .. } => Ok(status),
            ArmServiceResult::Error { message, .. } => Err(message),
            other => Err(unexpected_reply("close", &other)),
        }
    }

    pub fn wait(&mut self, duration_ms: u64) -> Result<ArmStatus, String> {
        match self
            .send_request(&format!(
                "\"command\":\"wait\",\"duration_ms\":{duration_ms}"
            ))?
            .result
        {
            ArmServiceResult::Status { status, .. } => Ok(status),
            ArmServiceResult::Error { message, .. } => Err(message),
            other => Err(unexpected_reply("wait", &other)),
        }
    }

    pub fn move_joint(&mut self, joint: Joint, angle_deg: u16) -> Result<ArmStatus, String> {
        match self
            .send_request(&format!(
                "\"command\":\"joint\",\"joint\":\"{}\",\"angle_deg\":{angle_deg}",
                joint_label(joint)
            ))?
            .result
        {
            ArmServiceResult::Status { status, .. } => Ok(status),
            ArmServiceResult::Error { message, .. } => Err(message),
            other => Err(unexpected_reply("joint", &other)),
        }
    }

    pub fn move_pose(&mut self, pose: ArmPose) -> Result<ArmStatus, String> {
        match self
            .send_request(&format!(
                "\"command\":\"pose\",\"base\":{},\"shoulder\":{},\"elbow\":{},\"gripper\":{}",
                pose.base, pose.shoulder, pose.elbow, pose.gripper
            ))?
            .result
        {
            ArmServiceResult::Status { status, .. } => Ok(status),
            ArmServiceResult::Error { message, .. } => Err(message),
            other => Err(unexpected_reply("pose", &other)),
        }
    }
}

fn unexpected_reply(command: &str, reply: &ArmServiceResult) -> String {
    format!("unexpected `{command}` reply: {reply:?}")
}

fn parse_service_reply(line: &str) -> Result<ArmServiceReply, String> {
    let request_id = extract_optional_json_string(line, "id").map(str::to_owned);
    let kind = extract_json_string(line, "kind")?;

    let result = match kind {
        "ok" => ArmServiceResult::Ok {
            command: extract_json_string(line, "command")?.to_owned(),
        },
        "note" => ArmServiceResult::Note {
            text: extract_json_string(line, "text")?.to_owned(),
        },
        "limits" => ArmServiceResult::Limits {
            limits: parse_limits(line)?,
        },
        "error" => ArmServiceResult::Error {
            command: extract_json_string(line, "command")?.to_owned(),
            message: extract_json_string(line, "message")?.to_owned(),
        },
        "status" => ArmServiceResult::Status {
            command: extract_json_string(line, "command")?.to_owned(),
            status: ArmStatus {
                seq: extract_json_number::<u32>(line, "seq")?,
                snapshot: Snapshot {
                    armed: extract_json_bool(line, "armed")?,
                    fault: parse_fault(extract_json_string(line, "fault")?)?,
                    moving: extract_json_bool(line, "moving")?,
                    active_joint: parse_active_joint(extract_json_string(line, "active")?)?,
                    servo_voltage_mv: parse_voltage_mv(extract_json_number::<u16>(
                        line,
                        "voltage_mv",
                    )?),
                    current_deg: extract_json_array4(line, "current")?,
                    target_deg: extract_json_array4(line, "target")?,
                },
            },
        },
        other => return Err(format!("unsupported service reply kind `{other}`")),
    };

    Ok(ArmServiceReply { request_id, result })
}

fn extract_json_string<'a>(line: &'a str, key: &str) -> Result<&'a str, String> {
    let prefix = format!("\"{key}\":\"");
    let start = line
        .find(&prefix)
        .ok_or_else(|| format!("missing string field `{key}`"))?
        + prefix.len();
    let rest = &line[start..];
    let end = rest
        .find('"')
        .ok_or_else(|| format!("unterminated string field `{key}`"))?;
    Ok(&rest[..end])
}

fn extract_optional_json_string<'a>(line: &'a str, key: &str) -> Option<&'a str> {
    let prefix = format!("\"{key}\":\"");
    let start = line.find(&prefix)? + prefix.len();
    let rest = &line[start..];
    let end = rest.find('"')?;
    Some(&rest[..end])
}

fn extract_json_object<'a>(line: &'a str, key: &str) -> Result<&'a str, String> {
    let prefix = format!("\"{key}\":{{");
    let start = line
        .find(&prefix)
        .ok_or_else(|| format!("missing object field `{key}`"))?
        + prefix.len()
        - 1;
    let rest = &line[start..];
    let mut depth = 0_usize;

    for (index, ch) in rest.char_indices() {
        match ch {
            '{' => depth += 1,
            '}' => {
                depth = depth.saturating_sub(1);
                if depth == 0 {
                    return Ok(&rest[..=index]);
                }
            }
            _ => {}
        }
    }

    Err(format!("unterminated object field `{key}`"))
}

fn extract_json_number<T>(line: &str, key: &str) -> Result<T, String>
where
    T: core::str::FromStr,
{
    let prefix = format!("\"{key}\":");
    let start = line
        .find(&prefix)
        .ok_or_else(|| format!("missing number field `{key}`"))?
        + prefix.len();
    let rest = &line[start..];
    let end = rest
        .find(|ch: char| !(ch.is_ascii_digit()))
        .unwrap_or(rest.len());

    rest[..end]
        .parse::<T>()
        .map_err(|_| format!("invalid number field `{key}`"))
}

fn extract_json_bool(line: &str, key: &str) -> Result<bool, String> {
    let prefix = format!("\"{key}\":");
    let start = line
        .find(&prefix)
        .ok_or_else(|| format!("missing bool field `{key}`"))?
        + prefix.len();
    let rest = &line[start..];

    if let Some(value) = rest.strip_prefix("true") {
        let _ = value;
        Ok(true)
    } else if let Some(value) = rest.strip_prefix("false") {
        let _ = value;
        Ok(false)
    } else {
        Err(format!("invalid bool field `{key}`"))
    }
}

fn extract_json_array4(line: &str, key: &str) -> Result<[u16; 4], String> {
    let prefix = format!("\"{key}\":[");
    let start = line
        .find(&prefix)
        .ok_or_else(|| format!("missing array field `{key}`"))?
        + prefix.len();
    let rest = &line[start..];
    let end = rest
        .find(']')
        .ok_or_else(|| format!("unterminated array field `{key}`"))?;
    let values = rest[..end]
        .split(',')
        .map(|value| {
            value
                .parse::<u16>()
                .map_err(|_| format!("invalid array item in `{key}`"))
        })
        .collect::<Result<Vec<_>, _>>()?;

    if values.len() != 4 {
        return Err(format!("array field `{key}` must contain 4 items"));
    }

    Ok([values[0], values[1], values[2], values[3]])
}

fn parse_fault(value: &str) -> Result<Option<Fault>, String> {
    match value {
        "NONE" => Ok(None),
        "HEARTBEAT_TIMEOUT" => Ok(Some(Fault::HeartbeatTimedOut)),
        "BROWNOUT" => Ok(Some(Fault::Brownout)),
        _ => Err(format!("unsupported fault `{value}`")),
    }
}

fn parse_active_joint(value: &str) -> Result<Option<Joint>, String> {
    match value {
        "NONE" => Ok(None),
        "BASE" => Ok(Some(Joint::Base)),
        "SHOULDER" => Ok(Some(Joint::Shoulder)),
        "ELBOW" => Ok(Some(Joint::Elbow)),
        "GRIPPER" => Ok(Some(Joint::Gripper)),
        _ => Err(format!("unsupported active joint `{value}`")),
    }
}

fn parse_limits(line: &str) -> Result<ArmLimits, String> {
    let base = extract_json_object(line, "base")?;
    let shoulder = extract_json_object(line, "shoulder")?;
    let elbow = extract_json_object(line, "elbow")?;
    let gripper = extract_json_object(line, "gripper")?;

    Ok(ArmLimits {
        base: JointLimits {
            min_deg: extract_json_number::<u16>(base, "min_deg")?,
            max_deg: extract_json_number::<u16>(base, "max_deg")?,
            home_deg: extract_json_number::<u16>(base, "home_deg")?,
        },
        shoulder: JointLimits {
            min_deg: extract_json_number::<u16>(shoulder, "min_deg")?,
            max_deg: extract_json_number::<u16>(shoulder, "max_deg")?,
            home_deg: extract_json_number::<u16>(shoulder, "home_deg")?,
        },
        elbow: JointLimits {
            min_deg: extract_json_number::<u16>(elbow, "min_deg")?,
            max_deg: extract_json_number::<u16>(elbow, "max_deg")?,
            home_deg: extract_json_number::<u16>(elbow, "home_deg")?,
        },
        gripper: GripperLimits {
            min_deg: extract_json_number::<u16>(gripper, "min_deg")?,
            max_deg: extract_json_number::<u16>(gripper, "max_deg")?,
            home_deg: extract_json_number::<u16>(gripper, "home_deg")?,
            closed_deg: extract_json_number::<u16>(gripper, "closed_deg")?,
        },
        heartbeat_timeout_ms: extract_json_number::<u32>(line, "heartbeat_timeout_ms")?,
        servo_voltage_min_mv: extract_json_number::<u16>(line, "servo_voltage_min_mv")?,
    })
}

fn parse_voltage_mv(voltage_mv: u16) -> Option<u16> {
    if voltage_mv == 0 {
        None
    } else {
        Some(voltage_mv)
    }
}

fn joint_label(joint: Joint) -> &'static str {
    match joint {
        Joint::Base => "base",
        Joint::Shoulder => "shoulder",
        Joint::Elbow => "elbow",
        Joint::Gripper => "gripper",
    }
}

fn escape_json(value: &str) -> String {
    let mut escaped = String::new();

    for ch in value.chars() {
        match ch {
            '\\' => escaped.push_str("\\\\"),
            '"' => escaped.push_str("\\\""),
            '\n' => escaped.push_str("\\n"),
            '\r' => escaped.push_str("\\r"),
            '\t' => escaped.push_str("\\t"),
            _ => escaped.push(ch),
        }
    }

    escaped
}

#[cfg(test)]
mod tests {
    use super::{ArmServiceReply, ArmServiceResult, escape_json, parse_service_reply};
    use crate::{ArmLimits, ArmStatus, Fault, GripperLimits, Joint, JointLimits, Snapshot};

    #[test]
    fn parses_status_replies() {
        assert_eq!(
            parse_service_reply(
                "{\"id\":\"req-7\",\"kind\":\"status\",\"command\":\"status\",\"seq\":7,\"armed\":true,\"fault\":\"BROWNOUT\",\"moving\":false,\"active\":\"NONE\",\"voltage_mv\":5032,\"current\":[90,90,100,38],\"target\":[90,90,100,38]}"
            ),
            Ok(ArmServiceReply {
                request_id: Some(String::from("req-7")),
                result: ArmServiceResult::Status {
                    command: String::from("status"),
                    status: ArmStatus {
                        seq: 7,
                        snapshot: Snapshot {
                            armed: true,
                            fault: Some(Fault::Brownout),
                            moving: false,
                            active_joint: None,
                            servo_voltage_mv: Some(5032),
                            current_deg: [90, 90, 100, 38],
                            target_deg: [90, 90, 100, 38],
                        },
                    },
                },
            })
        );
    }

    #[test]
    fn parses_ok_replies() {
        assert_eq!(
            parse_service_reply("{\"id\":\"req-2\",\"kind\":\"ok\",\"command\":\"clear-fault\"}"),
            Ok(ArmServiceReply {
                request_id: Some(String::from("req-2")),
                result: ArmServiceResult::Ok {
                    command: String::from("clear-fault")
                }
            })
        );
    }

    #[test]
    fn parses_error_replies() {
        assert_eq!(
            parse_service_reply(
                "{\"id\":\"req-9\",\"kind\":\"error\",\"command\":\"joint\",\"message\":\"shoulder angle 40 is outside the calibrated range 50..120\"}"
            ),
            Ok(ArmServiceReply {
                request_id: Some(String::from("req-9")),
                result: ArmServiceResult::Error {
                    command: String::from("joint"),
                    message: String::from(
                        "shoulder angle 40 is outside the calibrated range 50..120"
                    ),
                },
            })
        );
    }

    #[test]
    fn parses_active_joint_status_replies() {
        assert_eq!(
            parse_service_reply(
                "{\"id\":\"req-3\",\"kind\":\"status\",\"command\":\"status\",\"seq\":7,\"armed\":true,\"fault\":\"HEARTBEAT_TIMEOUT\",\"moving\":true,\"active\":\"ELBOW\",\"voltage_mv\":5000,\"current\":[91,90,100,120],\"target\":[100,90,100,120]}"
            ),
            Ok(ArmServiceReply {
                request_id: Some(String::from("req-3")),
                result: ArmServiceResult::Status {
                    command: String::from("status"),
                    status: ArmStatus {
                        seq: 7,
                        snapshot: Snapshot {
                            armed: true,
                            fault: Some(Fault::HeartbeatTimedOut),
                            moving: true,
                            active_joint: Some(Joint::Elbow),
                            servo_voltage_mv: Some(5000),
                            current_deg: [91, 90, 100, 120],
                            target_deg: [100, 90, 100, 120],
                        },
                    },
                },
            })
        );
    }

    #[test]
    fn parses_limits_replies() {
        assert_eq!(
            parse_service_reply(
                "{\"id\":\"req-5\",\"kind\":\"limits\",\"base\":{\"min_deg\":0,\"max_deg\":180,\"home_deg\":90},\"shoulder\":{\"min_deg\":50,\"max_deg\":120,\"home_deg\":90},\"elbow\":{\"min_deg\":50,\"max_deg\":125,\"home_deg\":100},\"gripper\":{\"min_deg\":18,\"max_deg\":120,\"home_deg\":120,\"closed_deg\":38},\"heartbeat_timeout_ms\":2000,\"servo_voltage_min_mv\":4700}"
            ),
            Ok(ArmServiceReply {
                request_id: Some(String::from("req-5")),
                result: ArmServiceResult::Limits {
                    limits: ArmLimits {
                        base: JointLimits {
                            min_deg: 0,
                            max_deg: 180,
                            home_deg: 90,
                        },
                        shoulder: JointLimits {
                            min_deg: 50,
                            max_deg: 120,
                            home_deg: 90,
                        },
                        elbow: JointLimits {
                            min_deg: 50,
                            max_deg: 125,
                            home_deg: 100,
                        },
                        gripper: GripperLimits {
                            min_deg: 18,
                            max_deg: 120,
                            home_deg: 120,
                            closed_deg: 38,
                        },
                        heartbeat_timeout_ms: 2000,
                        servo_voltage_min_mv: 4700,
                    }
                },
            })
        );
    }

    #[test]
    fn escapes_request_strings() {
        assert_eq!(
            escape_json("say \"hi\"\\later"),
            String::from("say \\\"hi\\\"\\\\later")
        );
    }
}
