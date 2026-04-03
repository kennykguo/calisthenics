// ABOUTME: Verifies the firmware runtime that connects ASCII command lines to controller behavior.
// ABOUTME: Covers acknowledgements, error reporting, and monotonic status sequencing.

use arm_firmware::{ArmConfig, Runtime};

fn test_config() -> ArmConfig {
    ArmConfig::f446re_mg90s()
}

fn render_response(response: &arm_firmware::Response) -> String {
    let mut line = String::new();
    response.write_to(&mut line).unwrap();
    line
}

#[test]
fn acknowledges_valid_command_lines() {
    let mut runtime = Runtime::new(test_config());

    assert_eq!(render_response(&runtime.process_line(0, "ARM")), "ACK");
    assert_eq!(
        render_response(&runtime.process_line(1, "MOVE BASE 100")),
        "ACK"
    );
}

#[test]
fn reports_parse_and_command_errors() {
    let mut runtime = Runtime::new(test_config());

    assert_eq!(
        render_response(&runtime.process_line(0, "MOVE")),
        "ERR kind=PARSE code=MISSING_ARGUMENT"
    );
    assert_eq!(
        render_response(&runtime.process_line(1, "MOVE BASE 100")),
        "ERR kind=COMMAND code=DISARMED"
    );
}

#[test]
fn returns_status_snapshots_for_explicit_status_requests() {
    let mut runtime = Runtime::new(test_config());

    runtime.process_line(0, "ARM");
    runtime.process_line(1, "MOVE BASE 100");
    runtime.observe_servo_voltage_mv(5000);
    runtime.tick(10);

    assert_eq!(
        render_response(&runtime.process_line(11, "STATUS")),
        "STATUS seq=0 armed=1 fault=NONE moving=1 active=BASE voltage_mv=5000 current=91,90,100,120 target=100,90,100,120"
    );
    assert_eq!(
        render_response(&runtime.process_line(12, "STATUS")),
        "STATUS seq=1 armed=1 fault=NONE moving=1 active=BASE voltage_mv=5000 current=91,90,100,120 target=100,90,100,120"
    );
}

#[test]
fn emits_status_with_monotonic_sequence_numbers() {
    let mut runtime = Runtime::new(test_config());

    runtime.process_line(0, "ARM");
    runtime.process_line(1, "MOVE BASE 100");
    runtime.observe_servo_voltage_mv(5000);
    runtime.tick(10);

    let first = render_response(&runtime.status());
    let second = render_response(&runtime.status());

    assert_eq!(
        first,
        "STATUS seq=0 armed=1 fault=NONE moving=1 active=BASE voltage_mv=5000 current=91,90,100,120 target=100,90,100,120"
    );
    assert_eq!(
        second,
        "STATUS seq=1 armed=1 fault=NONE moving=1 active=BASE voltage_mv=5000 current=91,90,100,120 target=100,90,100,120"
    );
}
