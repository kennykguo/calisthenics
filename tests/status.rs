// ABOUTME: Verifies status reporting and home sequencing for the STM32 arm controller.
// ABOUTME: Covers status line formatting and sequential return-to-home behavior.

use arm_firmware::{ArmConfig, ArmController, Command, Joint, parse_command};

fn test_config() -> ArmConfig {
    ArmConfig::f446re_mg90s()
}

#[test]
fn parses_home_command() {
    assert_eq!(parse_command("HOME"), Ok(Command::Home));
}

#[test]
fn home_command_returns_joints_to_home_in_joint_order() {
    let mut controller = ArmController::new(test_config());

    controller.handle_command(0, Command::Arm).unwrap();
    controller
        .handle_command(
            0,
            Command::MoveJoint {
                joint: Joint::Base,
                angle_deg: 100,
            },
        )
        .unwrap();
    controller
        .handle_command(
            0,
            Command::MoveJoint {
                joint: Joint::Shoulder,
                angle_deg: 100,
            },
        )
        .unwrap();

    for now_ms in (10..=120).step_by(10) {
        controller.tick(now_ms);
    }

    assert_eq!(controller.snapshot().active_joint, Some(Joint::Shoulder));
    assert_eq!(controller.snapshot().current_deg, [100, 92, 100, 120]);

    controller.handle_command(130, Command::Home).unwrap();

    controller.tick(140);
    let first_tick = controller.snapshot();
    assert_eq!(first_tick.active_joint, Some(Joint::Base));
    assert_eq!(first_tick.current_deg, [99, 92, 100, 120]);

    for now_ms in (150..=230).step_by(10) {
        controller.tick(now_ms);
    }
    let second_tick = controller.snapshot();
    assert_eq!(second_tick.active_joint, Some(Joint::Base));
    assert_eq!(second_tick.current_deg, [90, 92, 100, 120]);

    controller.tick(240);
    let third_tick = controller.snapshot();
    assert_eq!(third_tick.active_joint, Some(Joint::Shoulder));
    assert_eq!(third_tick.current_deg, [90, 91, 100, 120]);

    controller.tick(250);
    let fourth_tick = controller.snapshot();
    assert_eq!(fourth_tick.active_joint, Some(Joint::Shoulder));
    assert_eq!(fourth_tick.current_deg, [90, 90, 100, 120]);
}

#[test]
fn status_frame_reports_motion_fault_and_voltage() {
    let mut controller = ArmController::new(test_config());

    controller.handle_command(0, Command::Arm).unwrap();
    controller.observe_servo_voltage_mv(5000);
    controller
        .handle_command(
            0,
            Command::MoveJoint {
                joint: Joint::Base,
                angle_deg: 100,
            },
        )
        .unwrap();
    controller.tick(10);

    let status = controller.status_frame(7);
    let mut line = String::new();
    status.write_to(&mut line).unwrap();

    assert_eq!(
        line,
        "STATUS seq=7 armed=1 fault=NONE moving=1 active=BASE voltage_mv=5000 current=91,90,100,120 target=100,90,100,120"
    );
}
