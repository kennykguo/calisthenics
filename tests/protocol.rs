// ABOUTME: Verifies the firmware command protocol and controller command handling.
// ABOUTME: Covers ASCII parsing, disarm behavior, and explicit fault recovery rules.

use arm_firmware::{
    ArmConfig, ArmController, Command, CommandError, Fault, Joint, ParseError, parse_command,
};

fn test_config() -> ArmConfig {
    ArmConfig::f446re_mg90s()
}

#[test]
fn parses_basic_commands() {
    assert_eq!(parse_command("ARM"), Ok(Command::Arm));
    assert_eq!(parse_command("DISARM"), Ok(Command::Disarm));
    assert_eq!(parse_command("HEARTBEAT"), Ok(Command::Heartbeat));
    assert_eq!(parse_command("CLEAR_FAULT"), Ok(Command::ClearFault));
    assert_eq!(parse_command("OPEN_GRIPPER"), Ok(Command::OpenGripper));
    assert_eq!(parse_command("CLOSE_GRIPPER"), Ok(Command::CloseGripper));
    assert_eq!(
        parse_command("MOVE SHOULDER 120"),
        Ok(Command::MoveJoint {
            joint: Joint::Shoulder,
            angle_deg: 120,
        })
    );
}

#[test]
fn rejects_unknown_commands_and_bad_move_arguments() {
    assert_eq!(parse_command(""), Err(ParseError::Empty));
    assert_eq!(parse_command("JUMP"), Err(ParseError::UnknownCommand));
    assert_eq!(parse_command("MOVE"), Err(ParseError::MissingArgument));
    assert_eq!(
        parse_command("MOVE WRIST 90"),
        Err(ParseError::UnknownJoint)
    );
    assert_eq!(
        parse_command("MOVE BASE nope"),
        Err(ParseError::InvalidAngle)
    );
}

#[test]
fn disarm_clears_pending_motion_and_holds_current_position() {
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
    controller.tick(10);

    controller.handle_command(11, Command::Disarm).unwrap();
    let snapshot = controller.snapshot();

    assert_eq!(snapshot.armed, false);
    assert_eq!(snapshot.active_joint, None);
    assert_eq!(snapshot.current_deg[Joint::Base.index()], 91);
    assert_eq!(snapshot.target_deg[Joint::Base.index()], 91);
}

#[test]
fn heartbeat_timeout_can_be_cleared_while_armed_and_stationary() {
    let mut controller = ArmController::new(test_config());

    controller.handle_command(0, Command::Arm).unwrap();
    controller.handle_command(0, Command::Heartbeat).unwrap();
    controller.tick(2_001);

    assert_eq!(controller.snapshot().fault, Some(Fault::HeartbeatTimedOut));
    assert_eq!(controller.snapshot().armed, true);
    assert_eq!(
        controller.handle_command(252, Command::Arm),
        Err(CommandError::Faulted(Fault::HeartbeatTimedOut))
    );

    controller.handle_command(253, Command::ClearFault).unwrap();
    controller.handle_command(254, Command::Heartbeat).unwrap();
    controller
        .handle_command(
            255,
            Command::MoveJoint {
                joint: Joint::Base,
                angle_deg: 100,
            },
        )
        .unwrap();

    assert_eq!(controller.snapshot().fault, None);
    assert_eq!(controller.snapshot().armed, true);
}

#[test]
fn brownout_clear_still_requires_disarm() {
    let mut controller = ArmController::new(test_config());

    controller.handle_command(0, Command::Arm).unwrap();
    controller.handle_command(0, Command::Heartbeat).unwrap();
    controller.observe_servo_voltage_mv(4600);

    assert_eq!(controller.snapshot().fault, Some(Fault::Brownout));
    assert_eq!(
        controller.handle_command(1, Command::ClearFault),
        Err(CommandError::MustDisarmBeforeClearingFault)
    );

    controller.handle_command(2, Command::Disarm).unwrap();
    controller.handle_command(3, Command::ClearFault).unwrap();

    assert_eq!(controller.snapshot().fault, None);
    assert_eq!(controller.snapshot().armed, false);
}

#[test]
fn gripper_macros_use_the_calibrated_positions() {
    let mut controller = ArmController::new(test_config());

    controller.handle_command(0, Command::Arm).unwrap();
    controller.handle_command(0, Command::OpenGripper).unwrap();
    assert_eq!(controller.snapshot().target_deg[Joint::Gripper.index()], 120);

    controller.handle_command(1, Command::CloseGripper).unwrap();
    assert_eq!(controller.snapshot().target_deg[Joint::Gripper.index()], 60);
}
