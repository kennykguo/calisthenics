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
    assert_eq!(snapshot.current_deg, [90, 90, 102, 75]);
    assert_eq!(snapshot.target_deg, [90, 90, 102, 75]);
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
fn heartbeat_timeout_latches_a_fault_and_stops_motion() {
    let mut controller = ArmController::new(test_config());
    controller.arm(0).unwrap();
    controller.heartbeat(0);
    controller.queue_target(Joint::Base, 100).unwrap();

    controller.tick(251);
    let snapshot = controller.snapshot();

    assert_eq!(snapshot.fault, Some(Fault::HeartbeatTimedOut));
    assert_eq!(snapshot.active_joint, None);
    assert_eq!(snapshot.current_deg[Joint::Base.index()], 90);
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
    assert_eq!(first_tick.current_deg, [95, 90, 102, 75]);

    controller.heartbeat(20);
    controller.tick(20);
    let second_tick = controller.snapshot();
    assert_eq!(second_tick.current_deg, [100, 90, 102, 75]);

    controller.heartbeat(30);
    controller.tick(30);
    let third_tick = controller.snapshot();
    assert_eq!(third_tick.active_joint, Some(Joint::Shoulder));
    assert_eq!(third_tick.current_deg, [100, 95, 102, 75]);
}
