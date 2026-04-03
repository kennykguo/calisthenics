// ABOUTME: Verifies the host-side arm session types used to drive the STM32 controller.
// ABOUTME: Covers typed request rendering, pose sequencing, and parsed status snapshots.

use arm_firmware::{ArmPose, ArmRequest, ArmStatus, Fault, Joint, Snapshot, parse_status_line};

#[test]
fn renders_typed_requests_into_protocol_lines() {
    assert_eq!(ArmRequest::Arm.to_line(), "ARM");
    assert_eq!(
        ArmRequest::MoveJoint {
            joint: Joint::Shoulder,
            angle_deg: 110,
        }
        .to_line(),
        "MOVE SHOULDER 110"
    );
}

#[test]
fn expands_pose_requests_in_safe_joint_order() {
    let pose = ArmPose {
        base: 90,
        shoulder: 100,
        elbow: 95,
        gripper: 120,
    };

    assert_eq!(
        pose.requests(),
        vec![
            ArmRequest::MoveJoint {
                joint: Joint::Base,
                angle_deg: 90,
            },
            ArmRequest::MoveJoint {
                joint: Joint::Shoulder,
                angle_deg: 100,
            },
            ArmRequest::MoveJoint {
                joint: Joint::Elbow,
                angle_deg: 95,
            },
            ArmRequest::MoveJoint {
                joint: Joint::Gripper,
                angle_deg: 120,
            },
        ]
    );
}

#[test]
fn parses_status_lines_into_shared_status_snapshots() {
    assert_eq!(
        parse_status_line(
            "STATUS seq=7 armed=1 fault=BROWNOUT moving=1 active=ELBOW voltage_mv=5032 current=90,91,92,120 target=90,100,110,120"
        ),
        Some(ArmStatus {
            seq: 7,
            snapshot: Snapshot {
                armed: true,
                fault: Some(Fault::Brownout),
                moving: true,
                active_joint: Some(Joint::Elbow),
                servo_voltage_mv: Some(5032),
                current_deg: [90, 91, 92, 120],
                target_deg: [90, 100, 110, 120],
            },
        })
    );
}
