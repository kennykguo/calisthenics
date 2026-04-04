// ABOUTME: Provides a typed host-side CLI for driving the STM32 arm controller over serial.
// ABOUTME: Offers status, scripted motion, and structured host-side logging on top of ArmSession.

use std::{
    fs::{self, File},
    io::{self, BufRead, Write},
    path::Path,
    time::Instant,
};

use arm_firmware::{ArmConfig, ArmPose, ArmSession, ArmStatus, Fault, Joint};

#[derive(Clone, Debug, Eq, PartialEq)]
enum CliCommand {
    Serve,
    RunScript(String),
    RecordScript {
        script_path: String,
        log_path: String,
    },
    SummarizeLog(String),
    CheckScript(String),
    ExpandScript(String),
    Note(String),
    Wait {
        duration_ms: u64,
    },
    Limits,
    Status,
    ClearFault,
    Arm,
    Disarm,
    Home,
    OpenGripper,
    CloseGripper,
    MoveJoint {
        joint: Joint,
        angle_deg: u16,
    },
    MovePose(ArmPose),
}

#[derive(Clone, Debug, Eq, PartialEq)]
enum Invocation {
    Offline(CliCommand),
    Online {
        device_path: String,
        command: CliCommand,
    },
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum OutputMode {
    Text,
    Json,
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct ParsedRequest {
    command: CliCommand,
    request_id: Option<String>,
}

fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), String> {
    let config = ArmConfig::f446re_mg90s();
    let (args, output_mode) = split_output_mode(std::env::args().skip(1).collect());
    match parse_invocation(args)? {
        Invocation::Offline(CliCommand::Limits) => {
            println!("{}", render_limits(ArmConfig::f446re_mg90s()))
        }
        Invocation::Offline(CliCommand::SummarizeLog(log_path)) => {
            let summary = summarize_log_file(&log_path)?;
            println!("{}", render_log_summary(&summary));
        }
        Invocation::Offline(CliCommand::CheckScript(script_path)) => {
            let _ = load_script_commands(&script_path, config)?;
            println!("OK");
        }
        Invocation::Offline(CliCommand::ExpandScript(script_path)) => {
            let commands = load_script_commands(&script_path, config)?;
            println!("{}", render_expanded_commands(&commands));
        }
        Invocation::Offline(command) => {
            return Err(format!(
                "offline invocation is not supported for `{}`",
                script_command_name(&command)
            ));
        }
        Invocation::Online {
            device_path,
            command: CliCommand::Serve,
        } => {
            let mut session = ArmSession::connect(&device_path)?;
            serve_commands(&mut session, output_mode)?;
        }
        Invocation::Online {
            device_path,
            command: CliCommand::RunScript(script_path),
        } => {
            let commands = load_script_commands(&script_path, config)?;
            let mut session = ArmSession::connect(&device_path)?;
            for command in &commands {
                execute_command(&mut session, command, output_mode, None)?;
            }
        }
        Invocation::Online {
            device_path,
            command:
                CliCommand::RecordScript {
                    script_path,
                    log_path,
                },
        } => {
            let mut session = ArmSession::connect(&device_path)?;
            run_script_with_log(&mut session, &script_path, &log_path, output_mode)?;
        }
        Invocation::Online {
            device_path,
            command,
        } => {
            validate_command(&command, config)?;
            let mut session = ArmSession::connect(&device_path)?;
            execute_command(&mut session, &command, output_mode, None)?;
        }
    }

    Ok(())
}

fn split_output_mode(mut args: Vec<String>) -> (Vec<String>, OutputMode) {
    if args.last().map(String::as_str) == Some("--json") {
        let _ = args.pop();
        (args, OutputMode::Json)
    } else {
        (args, OutputMode::Text)
    }
}

fn parse_invocation(args: Vec<String>) -> Result<Invocation, String> {
    let first = args.first().map(String::as_str).ok_or_else(usage)?;

    if matches!(first, "limits" | "check" | "expand" | "summarize") {
        return Ok(Invocation::Offline(parse_cli_command(args)?));
    }

    let (device_path, command_args) = args.split_first().ok_or_else(usage)?;
    if command_args.is_empty() {
        return Err(usage());
    }

    Ok(Invocation::Online {
        device_path: device_path.clone(),
        command: parse_cli_command(command_args.to_vec())?,
    })
}

fn usage() -> String {
    String::from(
        "usage: cargo run --bin armctl -- <limits|summarize <log.jsonl>|check <script>|<serial-device> <serve|status|clear-fault|arm|disarm|home|open|close|wait <ms>|note <text...>|joint <joint> <deg>|pose <base> <shoulder> <elbow> <gripper>|run <script>|record <script> <log.jsonl>>> [--json]",
    )
}

fn parse_cli_command(args: Vec<String>) -> Result<CliCommand, String> {
    let command = args.first().map(String::as_str).ok_or_else(usage)?;

    match command {
        "limits" => expect_exact_args(args, 1).map(|_| CliCommand::Limits),
        "summarize" => parse_summarize_log_command(&args),
        "check" => parse_check_script_command(&args),
        "expand" => parse_expand_script_command(&args),
        "serve" => expect_exact_args(args, 1).map(|_| CliCommand::Serve),
        "run" => parse_run_script_command(&args),
        "record" => parse_record_script_command(&args),
        "note" => parse_note_command(&args),
        "wait" => parse_wait_command(&args),
        "status" => expect_exact_args(args, 1).map(|_| CliCommand::Status),
        "clear-fault" => expect_exact_args(args, 1).map(|_| CliCommand::ClearFault),
        "arm" => expect_exact_args(args, 1).map(|_| CliCommand::Arm),
        "disarm" => expect_exact_args(args, 1).map(|_| CliCommand::Disarm),
        "home" => expect_exact_args(args, 1).map(|_| CliCommand::Home),
        "open" => expect_exact_args(args, 1).map(|_| CliCommand::OpenGripper),
        "close" => expect_exact_args(args, 1).map(|_| CliCommand::CloseGripper),
        "joint" => parse_joint_command(&args),
        "pose" => parse_pose_command(&args),
        _ => Err(usage()),
    }
}

fn run_script(session: &mut ArmSession, script_path: &str) -> Result<(), String> {
    let commands = load_script_commands(script_path, ArmConfig::f446re_mg90s())?;

    for command in &commands {
        execute_command(session, command, OutputMode::Text, None)?;
    }

    Ok(())
}

fn serve_commands(session: &mut ArmSession, output_mode: OutputMode) -> Result<(), String> {
    let stdin = io::stdin();

    for line in stdin.lock().lines() {
        let line = line.map_err(|error| format!("failed to read stdin: {error}"))?;
        match parse_request_line(&line) {
            Ok(Some(request)) => {
                if let Err(error) = execute_command(
                    session,
                    &request.command,
                    output_mode,
                    request.request_id.as_deref(),
                ) {
                    print_error_result(
                        script_command_name(&request.command),
                        &error,
                        output_mode,
                        request.request_id.as_deref(),
                    );
                }
            }
            Ok(None) => {}
            Err(error) => {
                print_error_result("parse", &error, output_mode, None);
            }
        }
    }

    Ok(())
}

fn run_script_with_log(
    session: &mut ArmSession,
    script_path: &str,
    log_path: &str,
    output_mode: OutputMode,
) -> Result<(), String> {
    let commands = load_script_commands(script_path, ArmConfig::f446re_mg90s())?;
    let mut log = ExecutionLog::create(log_path)?;

    for command in &commands {
        execute_command_with_log(session, command, &mut log, output_mode)?;
    }

    Ok(())
}

fn execute_command(
    session: &mut ArmSession,
    command: &CliCommand,
    output_mode: OutputMode,
    request_id: Option<&str>,
) -> Result<(), String> {
    match command {
        CliCommand::Serve => Err(String::from(
            "`serve` cannot be invoked inside an active session",
        )),
        CliCommand::RunScript(script_path) => run_script(session, script_path),
        CliCommand::RecordScript {
            script_path,
            log_path,
        } => run_script_with_log(session, script_path, log_path, output_mode),
        CliCommand::SummarizeLog(log_path) => {
            let summary = summarize_log_file(log_path)?;
            println!("{}", render_log_summary(&summary));
            Ok(())
        }
        CliCommand::CheckScript(script_path) => {
            let _ = load_script_commands(script_path, ArmConfig::f446re_mg90s())?;
            print_ok_result(command, output_mode, request_id);
            Ok(())
        }
        CliCommand::ExpandScript(script_path) => {
            let commands = load_script_commands(script_path, ArmConfig::f446re_mg90s())?;
            println!("{}", render_expanded_commands(&commands));
            Ok(())
        }
        CliCommand::Note(text) => {
            print_note_result(text, output_mode, request_id);
            Ok(())
        }
        CliCommand::Limits => {
            match output_mode {
                OutputMode::Text => println!("{}", render_limits(ArmConfig::f446re_mg90s())),
                OutputMode::Json => println!(
                    "{}",
                    render_limits_output(ArmConfig::f446re_mg90s(), request_id)
                ),
            }
            Ok(())
        }
        CliCommand::Wait { duration_ms } => {
            print_status_result(
                command,
                session.wait(*duration_ms)?,
                output_mode,
                request_id,
            );
            Ok(())
        }
        CliCommand::Status => {
            print_status_result(command, session.status()?, output_mode, request_id);
            Ok(())
        }
        CliCommand::ClearFault => {
            session.clear_fault()?;
            print_ok_result(command, output_mode, request_id);
            Ok(())
        }
        CliCommand::Arm => {
            session.arm()?;
            print_ok_result(command, output_mode, request_id);
            Ok(())
        }
        CliCommand::Disarm => {
            session.disarm()?;
            print_ok_result(command, output_mode, request_id);
            Ok(())
        }
        CliCommand::Home => {
            print_status_result(command, session.home()?, output_mode, request_id);
            Ok(())
        }
        CliCommand::OpenGripper => {
            print_status_result(command, session.open_gripper()?, output_mode, request_id);
            Ok(())
        }
        CliCommand::CloseGripper => {
            print_status_result(command, session.close_gripper()?, output_mode, request_id);
            Ok(())
        }
        CliCommand::MoveJoint { joint, angle_deg } => {
            print_status_result(
                command,
                session.move_joint(*joint, *angle_deg)?,
                output_mode,
                request_id,
            );
            Ok(())
        }
        CliCommand::MovePose(pose) => {
            print_status_result(command, session.move_pose(*pose)?, output_mode, request_id);
            Ok(())
        }
    }
}

fn print_error_result(
    command: &str,
    message: &str,
    output_mode: OutputMode,
    request_id: Option<&str>,
) {
    match output_mode {
        OutputMode::Text => println!("ERR kind=COMMAND code={message}"),
        OutputMode::Json => println!("{}", render_error_output(command, message, request_id)),
    }
}

fn expect_exact_args(args: Vec<String>, expected_len: usize) -> Result<Vec<String>, String> {
    if args.len() == expected_len {
        Ok(args)
    } else {
        Err(usage())
    }
}

fn parse_run_script_command(args: &[String]) -> Result<CliCommand, String> {
    if args.len() != 2 {
        return Err(usage());
    }

    Ok(CliCommand::RunScript(args[1].clone()))
}

fn parse_record_script_command(args: &[String]) -> Result<CliCommand, String> {
    if args.len() != 3 {
        return Err(usage());
    }

    Ok(CliCommand::RecordScript {
        script_path: args[1].clone(),
        log_path: args[2].clone(),
    })
}

fn parse_summarize_log_command(args: &[String]) -> Result<CliCommand, String> {
    if args.len() != 2 {
        return Err(usage());
    }

    Ok(CliCommand::SummarizeLog(args[1].clone()))
}

fn parse_check_script_command(args: &[String]) -> Result<CliCommand, String> {
    if args.len() != 2 {
        return Err(usage());
    }

    Ok(CliCommand::CheckScript(args[1].clone()))
}

fn parse_expand_script_command(args: &[String]) -> Result<CliCommand, String> {
    if args.len() != 2 {
        return Err(usage());
    }

    Ok(CliCommand::ExpandScript(args[1].clone()))
}

fn parse_wait_command(args: &[String]) -> Result<CliCommand, String> {
    if args.len() != 2 {
        return Err(usage());
    }

    let duration_ms = args[1]
        .parse::<u64>()
        .map_err(|_| String::from("wait duration must be a u64 millisecond value"))?;

    Ok(CliCommand::Wait { duration_ms })
}

fn parse_note_command(args: &[String]) -> Result<CliCommand, String> {
    if args.len() < 2 {
        return Err(usage());
    }

    Ok(CliCommand::Note(args[1..].join(" ")))
}

fn parse_joint_command(args: &[String]) -> Result<CliCommand, String> {
    if args.len() != 3 {
        return Err(usage());
    }

    let joint = parse_joint_name(&args[1])?;
    let angle_deg = args[2]
        .parse::<u16>()
        .map_err(|_| String::from("joint angle must be a u16 degree value"))?;

    Ok(CliCommand::MoveJoint { joint, angle_deg })
}

fn parse_pose_command(args: &[String]) -> Result<CliCommand, String> {
    if args.len() != 5 {
        return Err(usage());
    }

    let base = parse_pose_angle(&args[1], "base")?;
    let shoulder = parse_pose_angle(&args[2], "shoulder")?;
    let elbow = parse_pose_angle(&args[3], "elbow")?;
    let gripper = parse_pose_angle(&args[4], "gripper")?;

    Ok(CliCommand::MovePose(ArmPose {
        base,
        shoulder,
        elbow,
        gripper,
    }))
}

fn parse_pose_angle(value: &str, label: &str) -> Result<u16, String> {
    value
        .parse::<u16>()
        .map_err(|_| format!("{label} angle must be a u16 degree value"))
}

fn parse_joint_name(value: &str) -> Result<Joint, String> {
    match value.to_ascii_uppercase().as_str() {
        "BASE" => Ok(Joint::Base),
        "SHOULDER" => Ok(Joint::Shoulder),
        "ELBOW" => Ok(Joint::Elbow),
        "GRIPPER" => Ok(Joint::Gripper),
        _ => Err(String::from(
            "joint must be one of: base, shoulder, elbow, gripper",
        )),
    }
}

fn parse_script_line(line: &str) -> Result<Option<CliCommand>, String> {
    let trimmed = line.trim();
    if trimmed.is_empty() || trimmed.starts_with('#') {
        return Ok(None);
    }

    let parts = trimmed
        .split_ascii_whitespace()
        .map(String::from)
        .collect::<Vec<_>>();

    parse_cli_command(parts).map(Some)
}

fn parse_request_line(line: &str) -> Result<Option<ParsedRequest>, String> {
    let trimmed = line.trim();
    if trimmed.starts_with('{') {
        return parse_json_request_line(trimmed).map(Some);
    }

    parse_script_line(line).map(|command| {
        command.map(|command| ParsedRequest {
            command,
            request_id: None,
        })
    })
}

fn parse_json_request_line(line: &str) -> Result<ParsedRequest, String> {
    let command = extract_json_string(line, "command")?;
    let request_id = extract_optional_json_string(line, "id").map(str::to_owned);

    let command = match command {
        "limits" => CliCommand::Limits,
        "status" => CliCommand::Status,
        "clear-fault" => CliCommand::ClearFault,
        "arm" => CliCommand::Arm,
        "disarm" => CliCommand::Disarm,
        "home" => CliCommand::Home,
        "open" => CliCommand::OpenGripper,
        "close" => CliCommand::CloseGripper,
        "wait" => CliCommand::Wait {
            duration_ms: extract_json_number::<u64>(line, "duration_ms")?,
        },
        "note" => CliCommand::Note(extract_json_string(line, "text")?.to_owned()),
        "joint" => CliCommand::MoveJoint {
            joint: parse_joint_name(extract_json_string(line, "joint")?)?,
            angle_deg: extract_json_number::<u16>(line, "angle_deg")?,
        },
        "pose" => CliCommand::MovePose(ArmPose {
            base: extract_json_number::<u16>(line, "base")?,
            shoulder: extract_json_number::<u16>(line, "shoulder")?,
            elbow: extract_json_number::<u16>(line, "elbow")?,
            gripper: extract_json_number::<u16>(line, "gripper")?,
        }),
        other => return Err(format!("unsupported json command `{other}`")),
    };

    Ok(ParsedRequest {
        command,
        request_id,
    })
}

fn load_script_commands(script_path: &str, config: ArmConfig) -> Result<Vec<CliCommand>, String> {
    let script = fs::read_to_string(script_path)
        .map_err(|error| format!("failed to read {script_path}: {error}"))?;
    let mut commands = Vec::new();

    for (line_number, line) in script.lines().enumerate() {
        match parse_script_line(line) {
            Ok(Some(CliCommand::Serve)) => {
                return Err(format!(
                    "line {} in {script_path}: nested `serve` is not allowed",
                    line_number + 1
                ));
            }
            Ok(Some(CliCommand::RunScript(_))) => {
                return Err(format!(
                    "line {} in {script_path}: nested `run` is not allowed",
                    line_number + 1
                ));
            }
            Ok(Some(CliCommand::RecordScript { .. })) => {
                return Err(format!(
                    "line {} in {script_path}: nested `record` is not allowed",
                    line_number + 1
                ));
            }
            Ok(Some(CliCommand::SummarizeLog(_))) => {
                return Err(format!(
                    "line {} in {script_path}: nested `summarize` is not allowed",
                    line_number + 1
                ));
            }
            Ok(Some(CliCommand::CheckScript(_))) => {
                return Err(format!(
                    "line {} in {script_path}: nested `check` is not allowed",
                    line_number + 1
                ));
            }
            Ok(Some(command)) => {
                validate_command(&command, config).map_err(|error| {
                    format!("line {} in {script_path}: {error}", line_number + 1)
                })?;
                commands.push(command);
            }
            Ok(None) => {}
            Err(error) => {
                return Err(format!(
                    "line {} in {script_path}: {error}",
                    line_number + 1
                ));
            }
        }
    }

    validate_script_commands(&commands, config)?;

    Ok(commands)
}

fn validate_command(command: &CliCommand, config: ArmConfig) -> Result<(), String> {
    match command {
        CliCommand::MoveJoint { joint, angle_deg } => {
            validate_joint_angle(*joint, *angle_deg, config)
        }
        CliCommand::MovePose(pose) => {
            validate_joint_angle(Joint::Base, pose.base, config)?;
            validate_joint_angle(Joint::Shoulder, pose.shoulder, config)?;
            validate_joint_angle(Joint::Elbow, pose.elbow, config)?;
            validate_joint_angle(Joint::Gripper, pose.gripper, config)
        }
        CliCommand::Note(_) => Ok(()),
        _ => Ok(()),
    }
}

fn validate_joint_angle(joint: Joint, angle_deg: u16, config: ArmConfig) -> Result<(), String> {
    let limits = config.joints[joint.index()];
    if angle_deg < limits.min_deg || angle_deg > limits.max_deg {
        return Err(format!(
            "{} angle {} is outside the calibrated range {}..{}",
            joint_label(joint),
            angle_deg,
            limits.min_deg,
            limits.max_deg
        ));
    }

    Ok(())
}

fn validate_script_commands(commands: &[CliCommand], config: ArmConfig) -> Result<(), String> {
    let mut armed = false;

    for command in commands {
        validate_command(command, config)?;

        match command {
            CliCommand::Arm => armed = true,
            CliCommand::Disarm => armed = false,
            CliCommand::MoveJoint { .. }
            | CliCommand::MovePose(_)
            | CliCommand::Home
            | CliCommand::OpenGripper
            | CliCommand::CloseGripper => {
                if !armed {
                    return Err(format!(
                        "motion command `{}` requires the arm to be armed first",
                        script_command_name(command)
                    ));
                }
            }
            _ => {}
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{
        CliCommand, ExecutionLog, Invocation, LogEvent, OutputMode, ParsedRequest,
        parse_cli_command, parse_invocation, parse_request_line, parse_script_line,
        render_error_output, render_expanded_commands, render_limits, render_limits_output,
        render_log_event, render_log_summary, render_status_output, split_output_mode,
        summarize_log_text, validate_command, validate_script_commands,
    };
    use arm_firmware::{ArmConfig, ArmPose, ArmStatus, Fault, Joint, Snapshot};
    use std::{
        env, fs,
        time::{SystemTime, UNIX_EPOCH},
    };

    fn strings(values: &[&str]) -> Vec<String> {
        values.iter().map(|value| String::from(*value)).collect()
    }

    #[test]
    fn parses_joint_commands() {
        assert_eq!(
            parse_cli_command(strings(&["joint", "shoulder", "110"])),
            Ok(CliCommand::MoveJoint {
                joint: Joint::Shoulder,
                angle_deg: 110,
            })
        );
    }

    #[test]
    fn parses_pose_commands() {
        assert_eq!(
            parse_cli_command(strings(&["pose", "90", "95", "100", "120"])),
            Ok(CliCommand::MovePose(ArmPose {
                base: 90,
                shoulder: 95,
                elbow: 100,
                gripper: 120,
            }))
        );
    }

    #[test]
    fn parses_run_script_commands() {
        assert_eq!(
            parse_cli_command(strings(&["run", "demo.arm"])),
            Ok(CliCommand::RunScript(String::from("demo.arm")))
        );
    }

    #[test]
    fn parses_serve_commands() {
        assert_eq!(
            parse_cli_command(strings(&["serve"])),
            Ok(CliCommand::Serve)
        );
    }

    #[test]
    fn parses_record_script_commands() {
        assert_eq!(
            parse_cli_command(strings(&["record", "demo.arm", "demo.jsonl"])),
            Ok(CliCommand::RecordScript {
                script_path: String::from("demo.arm"),
                log_path: String::from("demo.jsonl"),
            })
        );
    }

    #[test]
    fn parses_check_script_commands() {
        assert_eq!(
            parse_cli_command(strings(&["check", "demo.arm"])),
            Ok(CliCommand::CheckScript(String::from("demo.arm")))
        );
    }

    #[test]
    fn parses_summarize_log_commands() {
        assert_eq!(
            parse_cli_command(strings(&["summarize", "demo.jsonl"])),
            Ok(CliCommand::SummarizeLog(String::from("demo.jsonl")))
        );
    }

    #[test]
    fn parses_expand_script_commands() {
        assert_eq!(
            parse_cli_command(strings(&["expand", "demo.arm"])),
            Ok(CliCommand::ExpandScript(String::from("demo.arm")))
        );
    }

    #[test]
    fn parses_limits_commands() {
        assert_eq!(
            parse_cli_command(strings(&["limits"])),
            Ok(CliCommand::Limits)
        );
    }

    #[test]
    fn parses_note_commands() {
        assert_eq!(
            parse_cli_command(strings(&["note", "close", "gripper"])),
            Ok(CliCommand::Note(String::from("close gripper")))
        );
    }

    #[test]
    fn parses_offline_limits_invocations_without_a_device() {
        assert_eq!(
            parse_invocation(strings(&["limits"])),
            Ok(Invocation::Offline(CliCommand::Limits))
        );
    }

    #[test]
    fn parses_offline_check_invocations_without_a_device() {
        assert_eq!(
            parse_invocation(strings(&["check", "demo.arm"])),
            Ok(Invocation::Offline(CliCommand::CheckScript(String::from(
                "demo.arm"
            ))))
        );
    }

    #[test]
    fn parses_offline_summarize_invocations_without_a_device() {
        assert_eq!(
            parse_invocation(strings(&["summarize", "demo.jsonl"])),
            Ok(Invocation::Offline(CliCommand::SummarizeLog(String::from(
                "demo.jsonl"
            ))))
        );
    }

    #[test]
    fn parses_offline_expand_invocations_without_a_device() {
        assert_eq!(
            parse_invocation(strings(&["expand", "demo.arm"])),
            Ok(Invocation::Offline(CliCommand::ExpandScript(String::from(
                "demo.arm"
            ))))
        );
    }

    #[test]
    fn parses_online_status_invocations_with_a_device() {
        assert_eq!(
            parse_invocation(strings(&["/dev/ttyACM0", "status"])),
            Ok(Invocation::Online {
                device_path: String::from("/dev/ttyACM0"),
                command: CliCommand::Status,
            })
        );
    }

    #[test]
    fn parses_online_record_invocations_with_a_device() {
        assert_eq!(
            parse_invocation(strings(&[
                "/dev/ttyACM0",
                "record",
                "demo.arm",
                "demo.jsonl",
            ])),
            Ok(Invocation::Online {
                device_path: String::from("/dev/ttyACM0"),
                command: CliCommand::RecordScript {
                    script_path: String::from("demo.arm"),
                    log_path: String::from("demo.jsonl"),
                },
            })
        );
    }

    #[test]
    fn parses_online_serve_invocations_with_a_device() {
        assert_eq!(
            parse_invocation(strings(&["/dev/ttyACM0", "serve"])),
            Ok(Invocation::Online {
                device_path: String::from("/dev/ttyACM0"),
                command: CliCommand::Serve,
            })
        );
    }

    #[test]
    fn strips_json_output_mode_from_the_end_of_args() {
        assert_eq!(
            split_output_mode(strings(&["/dev/ttyACM0", "status", "--json"])),
            (strings(&["/dev/ttyACM0", "status"]), OutputMode::Json)
        );
    }

    #[test]
    fn keeps_text_output_mode_without_the_json_flag() {
        assert_eq!(
            split_output_mode(strings(&["limits"])),
            (strings(&["limits"]), OutputMode::Text)
        );
    }

    #[test]
    fn parses_wait_commands() {
        assert_eq!(
            parse_cli_command(strings(&["wait", "750"])),
            Ok(CliCommand::Wait { duration_ms: 750 })
        );
    }

    #[test]
    fn rejects_unknown_joint_names() {
        assert_eq!(
            parse_cli_command(strings(&["joint", "wrist", "90"])),
            Err(String::from(
                "joint must be one of: base, shoulder, elbow, gripper"
            ))
        );
    }

    #[test]
    fn ignores_blank_and_comment_script_lines() {
        assert_eq!(parse_script_line("   "), Ok(None));
        assert_eq!(parse_script_line("# close around the object"), Ok(None));
    }

    #[test]
    fn parses_wait_script_lines() {
        assert_eq!(
            parse_script_line("wait 500"),
            Ok(Some(CliCommand::Wait { duration_ms: 500 }))
        );
    }

    #[test]
    fn parses_note_script_lines() {
        assert_eq!(
            parse_script_line("note open gripper"),
            Ok(Some(CliCommand::Note(String::from("open gripper"))))
        );
    }

    #[test]
    fn parses_json_status_request_lines() {
        assert_eq!(
            parse_request_line("{\"command\":\"status\"}"),
            Ok(Some(ParsedRequest {
                command: CliCommand::Status,
                request_id: None,
            }))
        );
    }

    #[test]
    fn parses_json_limits_request_lines() {
        assert_eq!(
            parse_request_line("{\"command\":\"limits\"}"),
            Ok(Some(ParsedRequest {
                command: CliCommand::Limits,
                request_id: None,
            }))
        );
    }

    #[test]
    fn parses_json_joint_request_lines() {
        assert_eq!(
            parse_request_line("{\"command\":\"joint\",\"joint\":\"base\",\"angle_deg\":120}"),
            Ok(Some(ParsedRequest {
                command: CliCommand::MoveJoint {
                    joint: Joint::Base,
                    angle_deg: 120,
                },
                request_id: None,
            }))
        );
    }

    #[test]
    fn parses_json_pose_request_lines() {
        assert_eq!(
            parse_request_line(
                "{\"command\":\"pose\",\"base\":90,\"shoulder\":95,\"elbow\":100,\"gripper\":120}"
            ),
            Ok(Some(ParsedRequest {
                command: CliCommand::MovePose(ArmPose {
                    base: 90,
                    shoulder: 95,
                    elbow: 100,
                    gripper: 120,
                }),
                request_id: None,
            }))
        );
    }

    #[test]
    fn parses_json_request_ids() {
        assert_eq!(
            parse_request_line("{\"id\":\"req-7\",\"command\":\"status\"}"),
            Ok(Some(ParsedRequest {
                command: CliCommand::Status,
                request_id: Some(String::from("req-7")),
            }))
        );
    }

    #[test]
    fn parses_json_request_lines_with_whitespace() {
        assert_eq!(
            parse_request_line("{\"id\": \"req-7\", \"command\": \"limits\"}"),
            Ok(Some(ParsedRequest {
                command: CliCommand::Limits,
                request_id: Some(String::from("req-7")),
            }))
        );
    }

    #[test]
    fn rejects_joint_commands_outside_the_calibrated_range() {
        assert_eq!(
            validate_command(
                &CliCommand::MoveJoint {
                    joint: Joint::Shoulder,
                    angle_deg: 40,
                },
                ArmConfig::f446re_mg90s(),
            ),
            Err(String::from(
                "shoulder angle 40 is outside the calibrated range 50..120"
            ))
        );
    }

    #[test]
    fn rejects_pose_commands_outside_the_calibrated_range() {
        assert_eq!(
            validate_command(
                &CliCommand::MovePose(ArmPose {
                    base: 90,
                    shoulder: 90,
                    elbow: 130,
                    gripper: 120,
                }),
                ArmConfig::f446re_mg90s(),
            ),
            Err(String::from(
                "elbow angle 130 is outside the calibrated range 50..125"
            ))
        );
    }

    #[test]
    fn rejects_motion_commands_before_arm_in_scripts() {
        assert_eq!(
            validate_script_commands(
                &[CliCommand::MoveJoint {
                    joint: Joint::Base,
                    angle_deg: 90,
                }],
                ArmConfig::f446re_mg90s(),
            ),
            Err(String::from(
                "motion command `joint` requires the arm to be armed first"
            ))
        );
    }

    #[test]
    fn allows_motion_commands_after_arm_in_scripts() {
        assert_eq!(
            validate_script_commands(
                &[
                    CliCommand::Note(String::from("safe to arm")),
                    CliCommand::Arm,
                    CliCommand::MoveJoint {
                        joint: Joint::Base,
                        angle_deg: 90,
                    },
                    CliCommand::Wait { duration_ms: 500 },
                    CliCommand::Home,
                ],
                ArmConfig::f446re_mg90s(),
            ),
            Ok(())
        );
    }

    #[test]
    fn renders_limits_as_stable_json() {
        assert_eq!(
            render_limits(ArmConfig::f446re_mg90s()),
            String::from(
                "{\"base\":{\"min_deg\":0,\"max_deg\":180,\"home_deg\":90},\"shoulder\":{\"min_deg\":50,\"max_deg\":120,\"home_deg\":90},\"elbow\":{\"min_deg\":50,\"max_deg\":125,\"home_deg\":100},\"gripper\":{\"min_deg\":18,\"max_deg\":120,\"home_deg\":120,\"closed_deg\":38},\"heartbeat_timeout_ms\":2000,\"servo_voltage_min_mv\":4700}"
            )
        );
    }

    #[test]
    fn renders_limits_output_with_request_ids() {
        assert_eq!(
            render_limits_output(ArmConfig::f446re_mg90s(), Some("req-7")),
            String::from(
                "{\"id\":\"req-7\",\"kind\":\"limits\",\"base\":{\"min_deg\":0,\"max_deg\":180,\"home_deg\":90},\"shoulder\":{\"min_deg\":50,\"max_deg\":120,\"home_deg\":90},\"elbow\":{\"min_deg\":50,\"max_deg\":125,\"home_deg\":100},\"gripper\":{\"min_deg\":18,\"max_deg\":120,\"home_deg\":120,\"closed_deg\":38},\"heartbeat_timeout_ms\":2000,\"servo_voltage_min_mv\":4700}"
            )
        );
    }

    #[test]
    fn expands_pose_commands_into_safe_joint_order() {
        assert_eq!(
            render_expanded_commands(&[CliCommand::MovePose(ArmPose {
                base: 90,
                shoulder: 95,
                elbow: 100,
                gripper: 120,
            })]),
            String::from("MOVE BASE 90\nMOVE SHOULDER 95\nMOVE ELBOW 100\nMOVE GRIPPER 120")
        );
    }

    #[test]
    fn expands_wait_and_macro_commands_verbatim() {
        assert_eq!(
            render_expanded_commands(&[
                CliCommand::ClearFault,
                CliCommand::Arm,
                CliCommand::Note(String::from("close gripper")),
                CliCommand::Wait { duration_ms: 500 },
                CliCommand::OpenGripper,
            ]),
            String::from("CLEAR_FAULT\nARM\nNOTE close gripper\nWAIT 500\nOPEN_GRIPPER")
        );
    }

    #[test]
    fn renders_note_log_events_as_json_lines() {
        assert_eq!(
            render_log_event(&LogEvent::Note {
                elapsed_ms: 25,
                text: "close gripper",
            }),
            String::from("{\"t_ms\":25,\"kind\":\"note\",\"text\":\"close gripper\"}")
        );
    }

    #[test]
    fn renders_status_log_events_as_json_lines() {
        assert_eq!(
            render_log_event(&LogEvent::Status {
                elapsed_ms: 75,
                command: "close",
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
            }),
            String::from(
                "{\"t_ms\":75,\"kind\":\"status\",\"command\":\"close\",\"seq\":7,\"armed\":true,\"fault\":\"BROWNOUT\",\"moving\":false,\"active\":\"NONE\",\"voltage_mv\":5032,\"current\":[90,90,100,38],\"target\":[90,90,100,38]}"
            )
        );
    }

    #[test]
    fn renders_status_output_as_stable_json() {
        assert_eq!(
            render_status_output(
                "status",
                ArmStatus {
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
                OutputMode::Json,
                None,
            ),
            String::from(
                "{\"kind\":\"status\",\"command\":\"status\",\"seq\":7,\"armed\":true,\"fault\":\"BROWNOUT\",\"moving\":false,\"active\":\"NONE\",\"voltage_mv\":5032,\"current\":[90,90,100,38],\"target\":[90,90,100,38]}"
            )
        );
    }

    #[test]
    fn renders_status_output_with_request_ids() {
        assert_eq!(
            render_status_output(
                "status",
                ArmStatus {
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
                OutputMode::Json,
                Some("req-7"),
            ),
            String::from(
                "{\"id\":\"req-7\",\"kind\":\"status\",\"command\":\"status\",\"seq\":7,\"armed\":true,\"fault\":\"BROWNOUT\",\"moving\":false,\"active\":\"NONE\",\"voltage_mv\":5032,\"current\":[90,90,100,38],\"target\":[90,90,100,38]}"
            )
        );
    }

    #[test]
    fn renders_error_output_as_stable_json() {
        assert_eq!(
            render_error_output(
                "joint",
                "shoulder angle 40 is outside the calibrated range 50..120",
                None,
            ),
            String::from(
                "{\"kind\":\"error\",\"command\":\"joint\",\"message\":\"shoulder angle 40 is outside the calibrated range 50..120\"}"
            )
        );
    }

    #[test]
    fn renders_error_output_with_request_ids() {
        assert_eq!(
            render_error_output("parse", "missing string field `command`", Some("req-7")),
            String::from(
                "{\"id\":\"req-7\",\"kind\":\"error\",\"command\":\"parse\",\"message\":\"missing string field `command`\"}"
            )
        );
    }

    #[test]
    fn creates_parent_directories_for_record_logs() {
        let unique_suffix = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("system clock should be after unix epoch")
            .as_nanos();
        let log_dir = env::temp_dir().join(format!(
            "armctl-log-test-{}-{}",
            std::process::id(),
            unique_suffix
        ));
        let log_path = log_dir.join("nested/gripper-check.jsonl");
        let log_path_string = log_path.to_string_lossy().into_owned();

        let _log = ExecutionLog::create(&log_path_string)
            .expect("record log creation should make parent directories");

        assert!(log_path.exists());
        fs::remove_dir_all(log_dir).expect("temporary log directory should be removable");
    }

    #[test]
    fn summarizes_recorded_logs_as_stable_json() {
        let summary = summarize_log_text(
            concat!(
                "{\"t_ms\":2,\"kind\":\"ok\",\"command\":\"clear-fault\"}\n",
                "{\"t_ms\":16,\"kind\":\"status\",\"command\":\"status\",\"seq\":225,\"armed\":true,\"fault\":\"NONE\",\"moving\":false,\"active\":\"NONE\",\"voltage_mv\":5148,\"current\":[90,90,100,120],\"target\":[90,90,100,120]}\n",
                "{\"t_ms\":16,\"kind\":\"note\",\"text\":\"close gripper\"}\n",
                "{\"t_ms\":1760,\"kind\":\"status\",\"command\":\"close\",\"seq\":229,\"armed\":true,\"fault\":\"NONE\",\"moving\":false,\"active\":\"NONE\",\"voltage_mv\":5138,\"current\":[90,90,100,38],\"target\":[90,90,100,38]}\n"
            ),
        )
        .expect("summary parsing should succeed");

        assert_eq!(
            render_log_summary(&summary),
            String::from(
                "{\"total_events\":4,\"ok_events\":1,\"note_events\":1,\"status_events\":2,\"error_events\":0,\"duration_ms\":1760,\"min_voltage_mv\":5138,\"max_voltage_mv\":5148,\"final_fault\":\"NONE\",\"final_current\":[90,90,100,38],\"final_target\":[90,90,100,38]}"
            )
        );
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct LogSummary {
    total_events: u32,
    ok_events: u32,
    note_events: u32,
    status_events: u32,
    error_events: u32,
    duration_ms: u128,
    min_voltage_mv: Option<u16>,
    max_voltage_mv: Option<u16>,
    final_fault: String,
    final_current: [u16; 4],
    final_target: [u16; 4],
}

struct ExecutionLog {
    file: File,
    started_at: Instant,
}

impl ExecutionLog {
    fn create(log_path: &str) -> Result<Self, String> {
        if let Some(parent) = Path::new(log_path).parent() {
            if !parent.as_os_str().is_empty() {
                fs::create_dir_all(parent)
                    .map_err(|error| format!("failed to create {}: {error}", parent.display()))?;
            }
        }

        let file = File::create(log_path)
            .map_err(|error| format!("failed to create {log_path}: {error}"))?;

        Ok(Self {
            file,
            started_at: Instant::now(),
        })
    }

    fn write_note(&mut self, text: &str) -> Result<(), String> {
        self.write_event(LogEvent::Note {
            elapsed_ms: self.elapsed_ms(),
            text,
        })
    }

    fn write_ok(&mut self, command: &CliCommand) -> Result<(), String> {
        self.write_event(LogEvent::Ok {
            elapsed_ms: self.elapsed_ms(),
            command: script_command_name(command),
        })
    }

    fn write_status(&mut self, command: &CliCommand, status: ArmStatus) -> Result<(), String> {
        self.write_event(LogEvent::Status {
            elapsed_ms: self.elapsed_ms(),
            command: script_command_name(command),
            status,
        })
    }

    fn write_error(&mut self, command: &CliCommand, message: &str) -> Result<(), String> {
        self.write_event(LogEvent::Error {
            elapsed_ms: self.elapsed_ms(),
            command: script_command_name(command),
            message,
        })
    }

    fn write_event(&mut self, event: LogEvent<'_>) -> Result<(), String> {
        writeln!(self.file, "{}", render_log_event(&event))
            .map_err(|error| format!("failed to write execution log: {error}"))
    }

    fn elapsed_ms(&self) -> u128 {
        self.started_at.elapsed().as_millis()
    }
}

enum LogEvent<'a> {
    Ok {
        elapsed_ms: u128,
        command: &'a str,
    },
    Note {
        elapsed_ms: u128,
        text: &'a str,
    },
    Status {
        elapsed_ms: u128,
        command: &'a str,
        status: ArmStatus,
    },
    Error {
        elapsed_ms: u128,
        command: &'a str,
        message: &'a str,
    },
}

fn summarize_log_file(log_path: &str) -> Result<LogSummary, String> {
    let log_text = fs::read_to_string(log_path)
        .map_err(|error| format!("failed to read {log_path}: {error}"))?;
    summarize_log_text(&log_text)
}

fn summarize_log_text(log_text: &str) -> Result<LogSummary, String> {
    let mut summary = LogSummary {
        total_events: 0,
        ok_events: 0,
        note_events: 0,
        status_events: 0,
        error_events: 0,
        duration_ms: 0,
        min_voltage_mv: None,
        max_voltage_mv: None,
        final_fault: String::from("NONE"),
        final_current: [0, 0, 0, 0],
        final_target: [0, 0, 0, 0],
    };

    for (index, raw_line) in log_text.lines().enumerate() {
        let line = raw_line.trim();
        if line.is_empty() {
            continue;
        }

        summary.total_events += 1;
        summary.duration_ms = summary.duration_ms.max(
            extract_json_number::<u128>(line, "t_ms")
                .map_err(|error| format!("line {}: {error}", index + 1))?,
        );

        match extract_json_string(line, "kind")
            .map_err(|error| format!("line {}: {error}", index + 1))?
        {
            "ok" => summary.ok_events += 1,
            "note" => summary.note_events += 1,
            "error" => summary.error_events += 1,
            "status" => {
                summary.status_events += 1;
                let voltage_mv = extract_json_number::<u16>(line, "voltage_mv")
                    .map_err(|error| format!("line {}: {error}", index + 1))?;
                summary.min_voltage_mv = Some(
                    summary
                        .min_voltage_mv
                        .map_or(voltage_mv, |current| current.min(voltage_mv)),
                );
                summary.max_voltage_mv = Some(
                    summary
                        .max_voltage_mv
                        .map_or(voltage_mv, |current| current.max(voltage_mv)),
                );
                summary.final_fault = extract_json_string(line, "fault")
                    .map_err(|error| format!("line {}: {error}", index + 1))?
                    .to_owned();
                summary.final_current = extract_json_array4(line, "current")
                    .map_err(|error| format!("line {}: {error}", index + 1))?;
                summary.final_target = extract_json_array4(line, "target")
                    .map_err(|error| format!("line {}: {error}", index + 1))?;
            }
            kind => {
                return Err(format!("line {}: unsupported log kind `{kind}`", index + 1));
            }
        }
    }

    Ok(summary)
}

fn execute_command_with_log(
    session: &mut ArmSession,
    command: &CliCommand,
    log: &mut ExecutionLog,
    output_mode: OutputMode,
) -> Result<(), String> {
    let result = match command {
        CliCommand::Note(text) => {
            print_note_result(text, output_mode, None);
            log.write_note(text)
        }
        CliCommand::Wait { duration_ms } => match session.wait(*duration_ms) {
            Ok(status) => {
                print_status_result(command, status, output_mode, None);
                log.write_status(command, status)
            }
            Err(error) => {
                log.write_error(command, &error)?;
                return Err(error);
            }
        },
        CliCommand::Status => match session.status() {
            Ok(status) => {
                print_status_result(command, status, output_mode, None);
                log.write_status(command, status)
            }
            Err(error) => {
                log.write_error(command, &error)?;
                return Err(error);
            }
        },
        CliCommand::ClearFault => match session.clear_fault() {
            Ok(()) => {
                print_ok_result(command, output_mode, None);
                log.write_ok(command)
            }
            Err(error) => {
                log.write_error(command, &error)?;
                return Err(error);
            }
        },
        CliCommand::Arm => match session.arm() {
            Ok(()) => {
                print_ok_result(command, output_mode, None);
                log.write_ok(command)
            }
            Err(error) => {
                log.write_error(command, &error)?;
                return Err(error);
            }
        },
        CliCommand::Disarm => match session.disarm() {
            Ok(()) => {
                print_ok_result(command, output_mode, None);
                log.write_ok(command)
            }
            Err(error) => {
                log.write_error(command, &error)?;
                return Err(error);
            }
        },
        CliCommand::Home => match session.home() {
            Ok(status) => {
                print_status_result(command, status, output_mode, None);
                log.write_status(command, status)
            }
            Err(error) => {
                log.write_error(command, &error)?;
                return Err(error);
            }
        },
        CliCommand::OpenGripper => match session.open_gripper() {
            Ok(status) => {
                print_status_result(command, status, output_mode, None);
                log.write_status(command, status)
            }
            Err(error) => {
                log.write_error(command, &error)?;
                return Err(error);
            }
        },
        CliCommand::CloseGripper => match session.close_gripper() {
            Ok(status) => {
                print_status_result(command, status, output_mode, None);
                log.write_status(command, status)
            }
            Err(error) => {
                log.write_error(command, &error)?;
                return Err(error);
            }
        },
        CliCommand::MoveJoint { joint, angle_deg } => {
            match session.move_joint(*joint, *angle_deg) {
                Ok(status) => {
                    print_status_result(command, status, output_mode, None);
                    log.write_status(command, status)
                }
                Err(error) => {
                    log.write_error(command, &error)?;
                    return Err(error);
                }
            }
        }
        CliCommand::MovePose(pose) => match session.move_pose(*pose) {
            Ok(status) => {
                print_status_result(command, status, output_mode, None);
                log.write_status(command, status)
            }
            Err(error) => {
                log.write_error(command, &error)?;
                return Err(error);
            }
        },
        CliCommand::RunScript(_)
        | CliCommand::Serve
        | CliCommand::RecordScript { .. }
        | CliCommand::SummarizeLog(_)
        | CliCommand::CheckScript(_)
        | CliCommand::ExpandScript(_)
        | CliCommand::Limits => {
            return Err(format!(
                "`{}` is not allowed inside a recorded script run",
                script_command_name(command)
            ));
        }
    };

    result
}

fn print_ok_result(command: &CliCommand, output_mode: OutputMode, request_id: Option<&str>) {
    match output_mode {
        OutputMode::Text => println!("OK"),
        OutputMode::Json => println!(
            "{}",
            render_ok_output(script_command_name(command), request_id)
        ),
    }
}

fn print_note_result(text: &str, output_mode: OutputMode, request_id: Option<&str>) {
    match output_mode {
        OutputMode::Text => println!("NOTE {text}"),
        OutputMode::Json => println!("{}", render_note_output(text, request_id)),
    }
}

fn print_status_result(
    command: &CliCommand,
    status: ArmStatus,
    output_mode: OutputMode,
    request_id: Option<&str>,
) {
    match output_mode {
        OutputMode::Text => println!("{}", status.to_line()),
        OutputMode::Json => {
            println!(
                "{}",
                render_status_output(
                    script_command_name(command),
                    status,
                    output_mode,
                    request_id
                )
            )
        }
    }
}

fn render_ok_output(command: &str, request_id: Option<&str>) -> String {
    format!(
        "{{{}{}}}",
        render_json_id_prefix(request_id),
        format!("\"kind\":\"ok\",\"command\":\"{}\"", escape_json(command))
    )
}

fn render_note_output(text: &str, request_id: Option<&str>) -> String {
    format!(
        "{{{}{}}}",
        render_json_id_prefix(request_id),
        format!("\"kind\":\"note\",\"text\":\"{}\"", escape_json(text))
    )
}

fn render_error_output(command: &str, message: &str, request_id: Option<&str>) -> String {
    format!(
        "{{{}{}}}",
        render_json_id_prefix(request_id),
        format!(
            "\"kind\":\"error\",\"command\":\"{}\",\"message\":\"{}\"",
            escape_json(command),
            escape_json(message)
        )
    )
}

fn render_status_output(
    command: &str,
    status: ArmStatus,
    output_mode: OutputMode,
    request_id: Option<&str>,
) -> String {
    match output_mode {
        OutputMode::Text => status.to_line(),
        OutputMode::Json => {
            let snapshot = status.snapshot;
            format!(
                concat!(
                    "{{",
                    "{}",
                    "\"kind\":\"status\",",
                    "\"command\":\"{}\",",
                    "\"seq\":{},",
                    "\"armed\":{},",
                    "\"fault\":\"{}\",",
                    "\"moving\":{},",
                    "\"active\":\"{}\",",
                    "\"voltage_mv\":{},",
                    "\"current\":[{},{},{},{}],",
                    "\"target\":[{},{},{},{}]",
                    "}}"
                ),
                render_json_id_prefix(request_id),
                escape_json(command),
                status.seq,
                snapshot.armed,
                render_fault(snapshot.fault),
                snapshot.moving,
                render_active_joint(snapshot.active_joint),
                snapshot.servo_voltage_mv.unwrap_or(0),
                snapshot.current_deg[0],
                snapshot.current_deg[1],
                snapshot.current_deg[2],
                snapshot.current_deg[3],
                snapshot.target_deg[0],
                snapshot.target_deg[1],
                snapshot.target_deg[2],
                snapshot.target_deg[3],
            )
        }
    }
}

fn render_json_id_prefix(request_id: Option<&str>) -> String {
    match request_id {
        Some(id) => format!("\"id\":\"{}\",", escape_json(id)),
        None => String::new(),
    }
}

fn render_log_event(event: &LogEvent<'_>) -> String {
    match event {
        LogEvent::Ok {
            elapsed_ms,
            command,
        } => {
            format!(
                "{{\"t_ms\":{elapsed_ms},\"kind\":\"ok\",\"command\":\"{}\"}}",
                escape_json(command)
            )
        }
        LogEvent::Note { elapsed_ms, text } => {
            format!(
                "{{\"t_ms\":{elapsed_ms},\"kind\":\"note\",\"text\":\"{}\"}}",
                escape_json(text)
            )
        }
        LogEvent::Status {
            elapsed_ms,
            command,
            status,
        } => render_status_log_event(*elapsed_ms, command, status),
        LogEvent::Error {
            elapsed_ms,
            command,
            message,
        } => {
            format!(
                "{{\"t_ms\":{elapsed_ms},\"kind\":\"error\",\"command\":\"{}\",\"message\":\"{}\"}}",
                escape_json(command),
                escape_json(message)
            )
        }
    }
}

fn render_log_summary(summary: &LogSummary) -> String {
    format!(
        concat!(
            "{{",
            "\"total_events\":{},",
            "\"ok_events\":{},",
            "\"note_events\":{},",
            "\"status_events\":{},",
            "\"error_events\":{},",
            "\"duration_ms\":{},",
            "\"min_voltage_mv\":{},",
            "\"max_voltage_mv\":{},",
            "\"final_fault\":\"{}\",",
            "\"final_current\":[{},{},{},{}],",
            "\"final_target\":[{},{},{},{}]",
            "}}"
        ),
        summary.total_events,
        summary.ok_events,
        summary.note_events,
        summary.status_events,
        summary.error_events,
        summary.duration_ms,
        summary.min_voltage_mv.unwrap_or(0),
        summary.max_voltage_mv.unwrap_or(0),
        escape_json(&summary.final_fault),
        summary.final_current[0],
        summary.final_current[1],
        summary.final_current[2],
        summary.final_current[3],
        summary.final_target[0],
        summary.final_target[1],
        summary.final_target[2],
        summary.final_target[3],
    )
}

fn render_status_log_event(elapsed_ms: u128, command: &str, status: &ArmStatus) -> String {
    let snapshot = status.snapshot;
    format!(
        concat!(
            "{{",
            "\"t_ms\":{},",
            "\"kind\":\"status\",",
            "\"command\":\"{}\",",
            "\"seq\":{},",
            "\"armed\":{},",
            "\"fault\":\"{}\",",
            "\"moving\":{},",
            "\"active\":\"{}\",",
            "\"voltage_mv\":{},",
            "\"current\":[{},{},{},{}],",
            "\"target\":[{},{},{},{}]",
            "}}"
        ),
        elapsed_ms,
        escape_json(command),
        status.seq,
        snapshot.armed,
        render_fault(snapshot.fault),
        snapshot.moving,
        render_active_joint(snapshot.active_joint),
        snapshot.servo_voltage_mv.unwrap_or(0),
        snapshot.current_deg[0],
        snapshot.current_deg[1],
        snapshot.current_deg[2],
        snapshot.current_deg[3],
        snapshot.target_deg[0],
        snapshot.target_deg[1],
        snapshot.target_deg[2],
        snapshot.target_deg[3],
    )
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

fn extract_json_field_rest<'a>(line: &'a str, key: &str) -> Result<&'a str, String> {
    let key_prefix = format!("\"{key}\"");
    let mut search_start = 0;

    while let Some(relative_start) = line[search_start..].find(&key_prefix) {
        let start = search_start + relative_start + key_prefix.len();
        let rest = line[start..].trim_start();

        if let Some(rest) = rest.strip_prefix(':') {
            return Ok(rest.trim_start());
        }

        search_start += relative_start + key_prefix.len();
    }

    Err(format!("missing string field `{key}`"))
}

fn extract_json_string<'a>(line: &'a str, key: &str) -> Result<&'a str, String> {
    let rest = extract_json_field_rest(line, key)?;
    let rest = rest
        .strip_prefix('"')
        .ok_or_else(|| format!("missing string field `{key}`"))?;
    let end = rest
        .find('"')
        .ok_or_else(|| format!("unterminated string field `{key}`"))?;
    Ok(&rest[..end])
}

fn extract_optional_json_string<'a>(line: &'a str, key: &str) -> Option<&'a str> {
    let rest = extract_json_field_rest(line, key).ok()?;
    let rest = rest.strip_prefix('"')?;
    let end = rest.find('"')?;
    Some(&rest[..end])
}

fn extract_json_number<T>(line: &str, key: &str) -> Result<T, String>
where
    T: core::str::FromStr,
{
    let rest = extract_json_field_rest(line, key)?;
    let end = rest
        .find(|ch: char| !(ch.is_ascii_digit()))
        .unwrap_or(rest.len());

    rest[..end]
        .parse::<T>()
        .map_err(|_| format!("invalid number field `{key}`"))
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
    let items = rest[..end]
        .split(',')
        .map(|item| {
            item.parse::<u16>()
                .map_err(|_| format!("invalid array item in `{key}`"))
        })
        .collect::<Result<Vec<_>, _>>()?;

    if items.len() != 4 {
        return Err(format!("array field `{key}` must contain 4 items"));
    }

    Ok([items[0], items[1], items[2], items[3]])
}

fn render_fault(fault: Option<Fault>) -> &'static str {
    match fault {
        None => "NONE",
        Some(Fault::HeartbeatTimedOut) => "HEARTBEAT_TIMEOUT",
        Some(Fault::Brownout) => "BROWNOUT",
    }
}

fn render_active_joint(active_joint: Option<Joint>) -> &'static str {
    match active_joint {
        None => "NONE",
        Some(Joint::Base) => "BASE",
        Some(Joint::Shoulder) => "SHOULDER",
        Some(Joint::Elbow) => "ELBOW",
        Some(Joint::Gripper) => "GRIPPER",
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

fn joint_name(joint: Joint) -> &'static str {
    match joint {
        Joint::Base => "BASE",
        Joint::Shoulder => "SHOULDER",
        Joint::Elbow => "ELBOW",
        Joint::Gripper => "GRIPPER",
    }
}

fn render_limits(config: ArmConfig) -> String {
    let limits = config.limits();
    format!(
        concat!(
            "{{",
            "\"base\":{{\"min_deg\":{},\"max_deg\":{},\"home_deg\":{}}},",
            "\"shoulder\":{{\"min_deg\":{},\"max_deg\":{},\"home_deg\":{}}},",
            "\"elbow\":{{\"min_deg\":{},\"max_deg\":{},\"home_deg\":{}}},",
            "\"gripper\":{{\"min_deg\":{},\"max_deg\":{},\"home_deg\":{},\"closed_deg\":{}}},",
            "\"heartbeat_timeout_ms\":{},",
            "\"servo_voltage_min_mv\":{}",
            "}}"
        ),
        limits.base.min_deg,
        limits.base.max_deg,
        limits.base.home_deg,
        limits.shoulder.min_deg,
        limits.shoulder.max_deg,
        limits.shoulder.home_deg,
        limits.elbow.min_deg,
        limits.elbow.max_deg,
        limits.elbow.home_deg,
        limits.gripper.min_deg,
        limits.gripper.max_deg,
        limits.gripper.home_deg,
        limits.gripper.closed_deg,
        limits.heartbeat_timeout_ms,
        limits.servo_voltage_min_mv,
    )
}

fn render_limits_output(config: ArmConfig, request_id: Option<&str>) -> String {
    format!(
        concat!(
            "{{",
            "{}",
            "\"kind\":\"limits\",",
            "\"base\":{{\"min_deg\":{},\"max_deg\":{},\"home_deg\":{}}},",
            "\"shoulder\":{{\"min_deg\":{},\"max_deg\":{},\"home_deg\":{}}},",
            "\"elbow\":{{\"min_deg\":{},\"max_deg\":{},\"home_deg\":{}}},",
            "\"gripper\":{{\"min_deg\":{},\"max_deg\":{},\"home_deg\":{},\"closed_deg\":{}}},",
            "\"heartbeat_timeout_ms\":{},",
            "\"servo_voltage_min_mv\":{}",
            "}}"
        ),
        render_json_id_prefix(request_id),
        config.limits().base.min_deg,
        config.limits().base.max_deg,
        config.limits().base.home_deg,
        config.limits().shoulder.min_deg,
        config.limits().shoulder.max_deg,
        config.limits().shoulder.home_deg,
        config.limits().elbow.min_deg,
        config.limits().elbow.max_deg,
        config.limits().elbow.home_deg,
        config.limits().gripper.min_deg,
        config.limits().gripper.max_deg,
        config.limits().gripper.home_deg,
        config.limits().gripper.closed_deg,
        config.limits().heartbeat_timeout_ms,
        config.limits().servo_voltage_min_mv,
    )
}

fn render_expanded_commands(commands: &[CliCommand]) -> String {
    commands
        .iter()
        .flat_map(expanded_command_lines)
        .collect::<Vec<_>>()
        .join("\n")
}

fn expanded_command_lines(command: &CliCommand) -> Vec<String> {
    match command {
        CliCommand::RunScript(script_path) => vec![format!("RUN {script_path}")],
        CliCommand::RecordScript {
            script_path,
            log_path,
        } => vec![format!("RECORD {script_path} {log_path}")],
        CliCommand::SummarizeLog(log_path) => vec![format!("SUMMARIZE {log_path}")],
        CliCommand::CheckScript(script_path) => vec![format!("CHECK {script_path}")],
        CliCommand::ExpandScript(script_path) => vec![format!("EXPAND {script_path}")],
        CliCommand::Serve => vec![String::from("SERVE")],
        CliCommand::Note(text) => vec![format!("NOTE {text}")],
        CliCommand::Wait { duration_ms } => vec![format!("WAIT {duration_ms}")],
        CliCommand::Limits => vec![String::from("LIMITS")],
        CliCommand::Status => vec![String::from("STATUS")],
        CliCommand::ClearFault => vec![String::from("CLEAR_FAULT")],
        CliCommand::Arm => vec![String::from("ARM")],
        CliCommand::Disarm => vec![String::from("DISARM")],
        CliCommand::Home => vec![String::from("HOME")],
        CliCommand::OpenGripper => vec![String::from("OPEN_GRIPPER")],
        CliCommand::CloseGripper => vec![String::from("CLOSE_GRIPPER")],
        CliCommand::MoveJoint { joint, angle_deg } => {
            vec![format!("MOVE {} {angle_deg}", joint_name(*joint))]
        }
        CliCommand::MovePose(pose) => vec![
            format!("MOVE BASE {}", pose.base),
            format!("MOVE SHOULDER {}", pose.shoulder),
            format!("MOVE ELBOW {}", pose.elbow),
            format!("MOVE GRIPPER {}", pose.gripper),
        ],
    }
}

fn script_command_name(command: &CliCommand) -> &'static str {
    match command {
        CliCommand::MoveJoint { .. } => "joint",
        CliCommand::MovePose(_) => "pose",
        CliCommand::Home => "home",
        CliCommand::OpenGripper => "open",
        CliCommand::CloseGripper => "close",
        CliCommand::Serve => "serve",
        CliCommand::RunScript(_) => "run",
        CliCommand::RecordScript { .. } => "record",
        CliCommand::SummarizeLog(_) => "summarize",
        CliCommand::CheckScript(_) => "check",
        CliCommand::ExpandScript(_) => "expand",
        CliCommand::Note(_) => "note",
        CliCommand::Wait { .. } => "wait",
        CliCommand::Limits => "limits",
        CliCommand::Status => "status",
        CliCommand::ClearFault => "clear-fault",
        CliCommand::Arm => "arm",
        CliCommand::Disarm => "disarm",
    }
}
