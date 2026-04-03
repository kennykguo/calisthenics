// ABOUTME: Verifies the byte-oriented firmware app shell around the controller runtime.
// ABOUTME: Covers UART line assembly, overflow handling, and periodic control and status ticks.

use arm_firmware::{App, ArmConfig, Response};

fn test_config() -> ArmConfig {
    ArmConfig::f446re_mg90s()
}

fn render_response(response: &Response) -> String {
    let mut line = String::new();
    response.write_to(&mut line).unwrap();
    line
}

fn feed_line<const N: usize>(app: &mut App<N>, now_ms: u32, line: &str) -> Option<String> {
    let mut last = None;

    for byte in line.bytes().chain([b'\n']) {
        if let Some(response) = app.receive_byte(now_ms, byte) {
            last = Some(render_response(&response));
        }
    }

    last
}

#[test]
fn assembles_uart_bytes_into_commands() {
    let mut app = App::<32>::new(test_config(), 20, 100);

    assert_eq!(feed_line(&mut app, 0, "ARM"), Some(String::from("ACK")));
    assert_eq!(
        feed_line(&mut app, 1, "MOVE BASE 100"),
        Some(String::from("ACK"))
    );
}

#[test]
fn accepts_carriage_return_as_a_line_terminator() {
    let mut app = App::<32>::new(test_config(), 20, 100);
    let mut response = None;

    for byte in b"ARM\r" {
        if let Some(line) = app.receive_byte(0, *byte) {
            response = Some(render_response(&line));
        }
    }

    assert_eq!(response, Some(String::from("ACK")));
}

#[test]
fn ignores_line_feed_after_carriage_return() {
    let mut app = App::<32>::new(test_config(), 20, 100);
    let mut responses = Vec::new();

    for byte in b"ARM\r\n" {
        if let Some(line) = app.receive_byte(0, *byte) {
            responses.push(render_response(&line));
        }
    }

    assert_eq!(responses, vec![String::from("ACK")]);
}

#[test]
fn reports_line_overflow() {
    let mut app = App::<8>::new(test_config(), 20, 100);

    let response = feed_line(&mut app, 0, "MOVE BASE 100");

    assert_eq!(
        response,
        Some(String::from("ERR kind=TRANSPORT code=LINE_TOO_LONG"))
    );
}

#[test]
fn drives_control_ticks_and_periodic_status() {
    let mut app = App::<32>::new(test_config(), 20, 100);

    feed_line(&mut app, 0, "ARM");
    feed_line(&mut app, 0, "MOVE BASE 100");
    app.observe_servo_voltage_mv(5000);

    assert!(app.poll(20).is_none());
    assert!(app.poll(40).is_none());
    assert!(app.poll(80).is_none());

    let status = render_response(&app.poll(100).unwrap());

    assert_eq!(
        status,
        "STATUS seq=0 armed=1 fault=NONE moving=1 active=BASE voltage_mv=5000 current=95,90,100,120 target=100,90,100,120"
    );
}

#[test]
fn can_disable_periodic_status_for_polled_links() {
    let mut app = App::<32>::new_without_periodic_status(test_config(), 20);

    feed_line(&mut app, 0, "ARM");
    feed_line(&mut app, 0, "MOVE BASE 100");
    app.observe_servo_voltage_mv(5000);

    assert!(app.poll(20).is_none());
    assert!(app.poll(100).is_none());

    let status = feed_line(&mut app, 110, "STATUS");

    assert_eq!(
        status,
        Some(String::from(
            "STATUS seq=0 armed=1 fault=NONE moving=1 active=BASE voltage_mv=5000 current=95,90,100,120 target=100,90,100,120"
        ))
    );
}
