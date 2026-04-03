// ABOUTME: Exercises the host-testable firmware safety core for the STM32 arm controller.
// ABOUTME: Verifies arming, heartbeat timeout, target clamping, and staged joint motion.

use arm_firmware::{ArmConfig, ArmController, CommandError, Fault, Joint};

fn test_config() -> ArmConfig {
    ArmConfig::f446re_mg90s()
}

#[test]
fn starts_disarmed_at_safe_home_targets() {
    let controller = ArmController::new(test_config());
    let snapshot = controller.snapshot();

    assert_eq!(snapshot.armed, false);
    assert_eq!(snapshot.fault, None);
    assert_eq!(snapshot.active_joint, None);
    assert_eq!(snapshot.current_deg, [90, 90, 100, 120]);
    assert_eq!(snapshot.target_deg, [90, 90, 100, 120]);
}

#[test]
fn rejects_motion_commands_while_disarmed() {
    let mut controller = ArmController::new(test_config());

    assert_eq!(
        controller.queue_target(Joint::Base, 120),
        Err(CommandError::Disarmed)
    );
}

#[test]
fn clamps_targets_to_joint_limits() {
    let mut controller = ArmController::new(test_config());
    controller.arm(0).unwrap();
    controller.heartbeat(0);

    let clamped = controller.queue_target(Joint::Shoulder, 180).unwrap();

    assert_eq!(clamped, 120);
    assert_eq!(
        controller.snapshot().target_deg[Joint::Shoulder.index()],
        120
    );
}

#[test]
fn clamps_elbow_targets_to_the_calibrated_maximum() {
    let mut controller = ArmController::new(test_config());
    controller.arm(0).unwrap();
    controller.heartbeat(0);

    let clamped = controller.queue_target(Joint::Elbow, 135).unwrap();

    assert_eq!(clamped, 125);
    assert_eq!(controller.snapshot().target_deg[Joint::Elbow.index()], 125);
}

#[test]
fn clamps_shoulder_targets_to_the_calibrated_minimum() {
    let mut controller = ArmController::new(test_config());
    controller.arm(0).unwrap();
    controller.heartbeat(0);

    let clamped = controller.queue_target(Joint::Shoulder, 40).unwrap();

    assert_eq!(clamped, 50);
    assert_eq!(
        controller.snapshot().target_deg[Joint::Shoulder.index()],
        50
    );
}

#[test]
fn clamps_elbow_targets_to_the_calibrated_minimum() {
    let mut controller = ArmController::new(test_config());
    controller.arm(0).unwrap();
    controller.heartbeat(0);

    let clamped = controller.queue_target(Joint::Elbow, 40).unwrap();

    assert_eq!(clamped, 50);
    assert_eq!(controller.snapshot().target_deg[Joint::Elbow.index()], 50);
}

#[test]
fn allows_gripper_to_open_to_one_hundred_twenty_degrees() {
    let mut controller = ArmController::new(test_config());
    controller.arm(0).unwrap();
    controller.heartbeat(0);

    let clamped = controller.queue_target(Joint::Gripper, 120).unwrap();

    assert_eq!(clamped, 120);
    assert_eq!(
        controller.snapshot().target_deg[Joint::Gripper.index()],
        120
    );
}

#[test]
fn heartbeat_timeout_latches_a_fault_and_stops_motion() {
    let mut controller = ArmController::new(test_config());
    controller.arm(0).unwrap();
    controller.heartbeat(0);
    controller.queue_target(Joint::Base, 100).unwrap();

    controller.tick(2_001);
    let snapshot = controller.snapshot();

    assert_eq!(snapshot.fault, Some(Fault::HeartbeatTimedOut));
    assert_eq!(snapshot.active_joint, None);
    assert_eq!(snapshot.current_deg[Joint::Base.index()], 90);
}

#[test]
fn accepted_motion_commands_refresh_the_watchdog_deadline() {
    let mut controller = ArmController::new(test_config());
    controller.arm(0).unwrap();

    controller
        .handle_command(
            1_500,
            arm_firmware::Command::MoveJoint {
                joint: Joint::Base,
                angle_deg: 100,
            },
        )
        .unwrap();
    controller.tick(3_499);

    assert_eq!(controller.snapshot().fault, None);

    controller.tick(3_501);

    assert_eq!(controller.snapshot().fault, Some(Fault::HeartbeatTimedOut));
}

#[test]
fn moves_one_joint_at_a_time_in_queue_order() {
    let mut controller = ArmController::new(test_config());
    controller.arm(0).unwrap();
    controller.heartbeat(0);
    controller.queue_target(Joint::Base, 100).unwrap();
    controller.queue_target(Joint::Shoulder, 100).unwrap();

    controller.heartbeat(10);
    controller.tick(10);
    let first_tick = controller.snapshot();
    assert_eq!(first_tick.active_joint, Some(Joint::Base));
    assert_eq!(first_tick.current_deg, [91, 90, 100, 120]);

    controller.heartbeat(20);
    controller.tick(20);
    let second_tick = controller.snapshot();
    assert_eq!(second_tick.current_deg, [92, 90, 100, 120]);

    controller.heartbeat(30);
    controller.tick(30);
    let third_tick = controller.snapshot();
    assert_eq!(third_tick.active_joint, Some(Joint::Base));
    assert_eq!(third_tick.current_deg, [93, 90, 100, 120]);
}

#[test]
fn brownout_latches_a_fault_and_freezes_motion() {
    let mut controller = ArmController::new(test_config());
    controller.arm(0).unwrap();
    controller.heartbeat(0);
    controller.queue_target(Joint::Base, 100).unwrap();
    controller.tick(10);

    controller.observe_servo_voltage_mv(4600);
    let snapshot = controller.snapshot();

    assert_eq!(snapshot.fault, Some(Fault::Brownout));
    assert_eq!(snapshot.active_joint, None);
    assert_eq!(snapshot.current_deg[Joint::Base.index()], 91);
    assert_eq!(snapshot.target_deg[Joint::Base.index()], 91);
    assert_eq!(
        controller.queue_target(Joint::Base, 110),
        Err(CommandError::Faulted(Fault::Brownout))
    );
}
