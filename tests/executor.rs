// ABOUTME: Verifies the end-to-end firmware executor against a fake platform.
// ABOUTME: Covers UART responses, servo output updates, and periodic status transmission.

use arm_firmware::{ArmConfig, Executor, Joint, Platform};

fn test_config() -> ArmConfig {
    ArmConfig::f446re_mg90s()
}

struct FakePlatform {
    servo_angles: [u16; 4],
    servo_enabled: [bool; 4],
    tx_lines: Vec<String>,
}

impl FakePlatform {
    fn new() -> Self {
        Self {
            servo_angles: [0; 4],
            servo_enabled: [false; 4],
            tx_lines: Vec::new(),
        }
    }
}

impl Platform for FakePlatform {
    fn set_servo_enabled(&mut self, joint: Joint, enabled: bool) {
        self.servo_enabled[joint.index()] = enabled;
    }

    fn set_servo_angle(&mut self, joint: Joint, angle_deg: u16) {
        self.servo_angles[joint.index()] = angle_deg;
    }

    fn transmit_line(&mut self, line: &str) {
        self.tx_lines.push(line.to_owned());
    }
}

fn feed_line<const N: usize>(
    executor: &mut Executor<N>,
    platform: &mut FakePlatform,
    now_ms: u32,
    line: &str,
) {
    for byte in line.bytes().chain([b'\n']) {
        executor.receive_byte(platform, now_ms, byte);
    }
}

#[test]
fn transmits_acknowledgements_for_complete_command_lines() {
    let mut executor = Executor::<32>::new(test_config(), 20, 100);
    let mut platform = FakePlatform::new();

    feed_line(&mut executor, &mut platform, 0, "ARM");

    assert_eq!(platform.tx_lines, vec![String::from("ACK")]);
}

#[test]
fn keeps_servo_outputs_disabled_while_disarmed() {
    let mut executor = Executor::<32>::new(test_config(), 20, 100);
    let mut platform = FakePlatform::new();

    executor.tick(&mut platform, 0);

    assert_eq!(platform.servo_enabled, [false, false, false, false]);
}

#[test]
fn applies_controller_outputs_to_servo_channels_after_arming() {
    let mut executor = Executor::<32>::new(test_config(), 20, 100);
    let mut platform = FakePlatform::new();

    feed_line(&mut executor, &mut platform, 0, "ARM");
    feed_line(&mut executor, &mut platform, 1, "MOVE BASE 100");
    executor.tick(&mut platform, 20);

    assert_eq!(platform.servo_enabled, [true, true, true, true]);
    assert_eq!(platform.servo_angles, [91, 90, 100, 120]);
}

#[test]
fn emits_periodic_status_lines() {
    let mut executor = Executor::<32>::new(test_config(), 20, 100);
    let mut platform = FakePlatform::new();

    feed_line(&mut executor, &mut platform, 0, "ARM");
    feed_line(&mut executor, &mut platform, 0, "MOVE BASE 100");
    executor.observe_servo_voltage_mv(&mut platform, 5000);
    executor.tick(&mut platform, 20);
    executor.tick(&mut platform, 100);

    assert_eq!(
        platform.tx_lines.last(),
        Some(&String::from(
            "STATUS seq=0 armed=1 fault=NONE moving=1 active=BASE voltage_mv=5000 current=95,90,100,120 target=100,90,100,120"
        ))
    );
}
