// ABOUTME: Formats controller state into compact ASCII status lines for the Jetson link.
// ABOUTME: Reports arm state, active motion, faults, voltage, and current and target joint angles.

use core::fmt::{self, Write};

use crate::{Fault, Joint, Snapshot};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct StatusFrame {
    pub seq: u32,
    pub snapshot: Snapshot,
}

impl StatusFrame {
    pub fn write_to<W: Write>(&self, writer: &mut W) -> fmt::Result {
        write!(
            writer,
            "STATUS seq={} armed={} fault={} moving={} active={} voltage_mv={} current={},{},{},{} target={},{},{},{}",
            self.seq,
            self.snapshot.armed as u8,
            format_fault(self.snapshot.fault),
            self.snapshot.moving as u8,
            format_joint(self.snapshot.active_joint),
            format_voltage(self.snapshot.servo_voltage_mv),
            self.snapshot.current_deg[0],
            self.snapshot.current_deg[1],
            self.snapshot.current_deg[2],
            self.snapshot.current_deg[3],
            self.snapshot.target_deg[0],
            self.snapshot.target_deg[1],
            self.snapshot.target_deg[2],
            self.snapshot.target_deg[3],
        )
    }
}

fn format_fault(fault: Option<Fault>) -> &'static str {
    match fault {
        None => "NONE",
        Some(Fault::HeartbeatTimedOut) => "HEARTBEAT_TIMEOUT",
        Some(Fault::Brownout) => "BROWNOUT",
    }
}

fn format_joint(joint: Option<Joint>) -> &'static str {
    match joint {
        None => "NONE",
        Some(Joint::Base) => "BASE",
        Some(Joint::Shoulder) => "SHOULDER",
        Some(Joint::Elbow) => "ELBOW",
        Some(Joint::Gripper) => "GRIPPER",
    }
}

fn format_voltage(servo_voltage_mv: Option<u16>) -> u16 {
    servo_voltage_mv.unwrap_or(0)
}
