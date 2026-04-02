// ABOUTME: Runs a host-side serial session against the STM32 firmware for repeatable hardware checks.
// ABOUTME: Configures the tty, streams board output, and injects commands with automatic heartbeats.

use std::{
    fs::OpenOptions,
    io::{Read, Write},
    process::Command,
    time::Instant,
};

const COMMAND_GAP_MS: u64 = 300;
const HEARTBEAT_PERIOD_MS: u64 = 500;
const SETTLE_MS: u64 = 1_500;
const READ_DRAIN_MS: u64 = 200;

#[derive(Clone, Debug, Eq, PartialEq)]
struct ScheduledWrite {
    at_ms: u64,
    line: String,
}

fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), String> {
    let mut args = std::env::args().skip(1);
    let device_path = args
        .next()
        .ok_or_else(|| usage("cargo run --bin host -- <serial-device> <command> [command...]"))?;
    let commands: Vec<String> = args.collect();

    if commands.is_empty() {
        return Err(usage(
            "cargo run --bin host -- <serial-device> <command> [command...]",
        ));
    }

    run_session(&device_path, &commands)
}

fn usage(invocation: &str) -> String {
    format!("usage: {invocation}")
}

fn run_session(device_path: &str, commands: &[String]) -> Result<(), String> {
    configure_serial_device(device_path)?;

    let mut serial = OpenOptions::new()
        .read(true)
        .write(true)
        .open(device_path)
        .map_err(|error| format!("failed to open {device_path}: {error}"))?;
    let session_plan = build_session_plan(commands, COMMAND_GAP_MS, HEARTBEAT_PERIOD_MS, SETTLE_MS);
    let session_end_ms = session_end_ms(&session_plan, READ_DRAIN_MS);
    let session_start = Instant::now();
    let mut next_write_index = 0;
    let mut line_buffer = SerialLineBuffer::new();
    let mut read_buffer = [0_u8; 64];

    loop {
        let elapsed_ms = session_elapsed_ms(session_start);

        while let Some(scheduled_write) = session_plan.get(next_write_index) {
            if scheduled_write.at_ms > elapsed_ms {
                break;
            }

            println!(">> {}", scheduled_write.line);
            serial
                .write_all(scheduled_write.line.as_bytes())
                .and_then(|_| serial.write_all(b"\r"))
                .and_then(|_| serial.flush())
                .map_err(|error| format!("failed to write to {device_path}: {error}"))?;
            next_write_index += 1;
        }

        if (next_write_index >= session_plan.len()) && (elapsed_ms > session_end_ms) {
            break;
        }

        match serial.read(&mut read_buffer) {
            Ok(0) => {}
            Ok(read_len) => {
                for byte in &read_buffer[..read_len] {
                    if let Some(line) = line_buffer.push_byte(*byte) {
                        println!("{line}");
                    }
                }
            }
            Err(error) if error.kind() == std::io::ErrorKind::Interrupted => {}
            Err(error) => {
                return Err(format!("serial read error on {device_path}: {error}"));
            }
        }
    }

    Ok(())
}

fn configure_serial_device(device_path: &str) -> Result<(), String> {
    let status = Command::new("stty")
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

fn session_elapsed_ms(session_start: Instant) -> u64 {
    session_start
        .elapsed()
        .as_millis()
        .min(u128::from(u64::MAX)) as u64
}

fn session_end_ms(session_plan: &[ScheduledWrite], read_drain_ms: u64) -> u64 {
    session_plan
        .last()
        .map(|scheduled_write| scheduled_write.at_ms.saturating_add(read_drain_ms))
        .unwrap_or(read_drain_ms)
}

fn build_session_plan(
    commands: &[String],
    command_gap_ms: u64,
    heartbeat_period_ms: u64,
    settle_ms: u64,
) -> Vec<ScheduledWrite> {
    let mut session_plan = Vec::new();
    let mut command_times = Vec::with_capacity(commands.len());

    for (index, command) in commands.iter().enumerate() {
        let at_ms = index as u64 * command_gap_ms;
        session_plan.push(ScheduledWrite {
            at_ms,
            line: command.clone(),
        });
        command_times.push(at_ms);
    }

    if let Some(last_command_ms) = command_times.last().copied() {
        let session_end_ms = last_command_ms + settle_ms;
        let mut heartbeat_ms = heartbeat_period_ms;

        while heartbeat_ms < session_end_ms {
            if !command_times.contains(&heartbeat_ms) {
                session_plan.push(ScheduledWrite {
                    at_ms: heartbeat_ms,
                    line: String::from("HEARTBEAT"),
                });
            }

            heartbeat_ms += heartbeat_period_ms;
        }
    }

    session_plan.sort_by_key(|scheduled_write| scheduled_write.at_ms);
    session_plan
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
}

#[cfg(test)]
mod tests {
    use super::{ScheduledWrite, SerialLineBuffer, build_session_plan, session_end_ms, usage};

    #[test]
    fn prints_a_usage_string_for_missing_arguments() {
        assert_eq!(
            usage("cargo run --bin host -- <serial-device> <command>"),
            "usage: cargo run --bin host -- <serial-device> <command>"
        );
    }

    #[test]
    fn inserts_heartbeats_between_commands_and_settle_time() {
        let commands = vec![
            String::from("ARM"),
            String::from("MOVE BASE 150"),
            String::from("HOME"),
        ];

        let session_plan = build_session_plan(&commands, 300, 500, 1_500);

        assert_eq!(
            session_plan,
            vec![
                ScheduledWrite {
                    at_ms: 0,
                    line: String::from("ARM"),
                },
                ScheduledWrite {
                    at_ms: 300,
                    line: String::from("MOVE BASE 150"),
                },
                ScheduledWrite {
                    at_ms: 500,
                    line: String::from("HEARTBEAT"),
                },
                ScheduledWrite {
                    at_ms: 600,
                    line: String::from("HOME"),
                },
                ScheduledWrite {
                    at_ms: 1_000,
                    line: String::from("HEARTBEAT"),
                },
                ScheduledWrite {
                    at_ms: 1_500,
                    line: String::from("HEARTBEAT"),
                },
                ScheduledWrite {
                    at_ms: 2_000,
                    line: String::from("HEARTBEAT"),
                },
            ]
        );
    }

    #[test]
    fn does_not_duplicate_heartbeat_when_a_command_lands_on_the_same_tick() {
        let commands = vec![String::from("ARM"), String::from("HOME")];

        let session_plan = build_session_plan(&commands, 500, 500, 500);

        assert_eq!(
            session_plan,
            vec![
                ScheduledWrite {
                    at_ms: 0,
                    line: String::from("ARM"),
                },
                ScheduledWrite {
                    at_ms: 500,
                    line: String::from("HOME"),
                },
            ]
        );
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
    fn keeps_the_session_open_for_a_short_drain_after_the_last_write() {
        let session_plan = vec![
            ScheduledWrite {
                at_ms: 0,
                line: String::from("CLEAR_FAULT"),
            },
            ScheduledWrite {
                at_ms: 300,
                line: String::from("ARM"),
            },
        ];

        assert_eq!(session_end_ms(&session_plan, 200), 500);
    }
}
