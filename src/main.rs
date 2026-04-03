// ABOUTME: Provides a raw host-side CLI for sending firmware protocol lines to the STM32 arm.
// ABOUTME: Uses the shared ArmSession transport so low-level bench commands and typed tools stay aligned.

use arm_firmware::ArmSession;

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

    let mut session = ArmSession::connect(&device_path)?;

    for command in commands {
        println!(">> {command}");
        if let Some(status) = session.run_text_command(&command)? {
            println!("{}", status.to_line());
        } else {
            println!("ACK");
        }
    }

    Ok(())
}

fn usage(invocation: &str) -> String {
    format!("usage: {invocation}")
}

#[cfg(test)]
mod tests {
    use super::usage;

    #[test]
    fn prints_a_usage_string_for_missing_arguments() {
        assert_eq!(
            usage("cargo run --bin host -- <serial-device> <command>"),
            "usage: cargo run --bin host -- <serial-device> <command>"
        );
    }
}
