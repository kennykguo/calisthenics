// ABOUTME: Defines the firmware safety core for the STM32 arm controller.
// ABOUTME: Models arming, heartbeat, joint limits, and staged joint motion without hardware access.

#![cfg_attr(not(test), no_std)]

mod app;
mod byte_queue;
mod executor;
mod protocol;
mod runtime;
mod servo_pwm;
mod servo_rail;
mod status;

pub use app::App;
pub use byte_queue::ByteQueue;
pub use executor::{Executor, Platform};
pub use protocol::{Command, ParseError, parse_command};
pub use runtime::{Response, ResponseError, Runtime, TransportError};
pub use servo_pwm::ServoPwmConfig;
pub use servo_rail::ServoRailSense;
pub use status::StatusFrame;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Joint {
    Base,
    Shoulder,
    Elbow,
    Gripper,
}

impl Joint {
    pub const ALL: [Joint; 4] = [Joint::Base, Joint::Shoulder, Joint::Elbow, Joint::Gripper];

    pub const fn index(self) -> usize {
        match self {
            Joint::Base => 0,
            Joint::Shoulder => 1,
            Joint::Elbow => 2,
            Joint::Gripper => 3,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct JointConfig {
    pub min_deg: u16,
    pub max_deg: u16,
    pub home_deg: u16,
    pub max_step_deg: u16,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ArmConfig {
    pub joints: [JointConfig; 4],
    pub gripper_closed_deg: u16,
    pub heartbeat_timeout_ms: u32,
    pub servo_voltage_min_mv: u16,
}

impl ArmConfig {
    pub const fn f446re_mg90s() -> Self {
        Self {
            joints: [
                JointConfig {
                    min_deg: 0,
                    max_deg: 180,
                    home_deg: 90,
                    max_step_deg: 1,
                },
                JointConfig {
                    min_deg: 50,
                    max_deg: 120,
                    home_deg: 90,
                    max_step_deg: 1,
                },
                JointConfig {
                    min_deg: 50,
                    max_deg: 125,
                    home_deg: 100,
                    max_step_deg: 1,
                },
                JointConfig {
                    min_deg: 18,
                    max_deg: 120,
                    home_deg: 120,
                    max_step_deg: 1,
                },
            ],
            gripper_closed_deg: 60,
            heartbeat_timeout_ms: 2_000,
            servo_voltage_min_mv: 4700,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Fault {
    HeartbeatTimedOut,
    Brownout,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CommandError {
    Disarmed,
    Faulted(Fault),
    MustDisarmBeforeClearingFault,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Snapshot {
    pub armed: bool,
    pub fault: Option<Fault>,
    pub moving: bool,
    pub active_joint: Option<Joint>,
    pub servo_voltage_mv: Option<u16>,
    pub current_deg: [u16; 4],
    pub target_deg: [u16; 4],
}

pub struct ArmController {
    config: ArmConfig,
    armed: bool,
    fault: Option<Fault>,
    active_joint: Option<Joint>,
    queued_joints: [Option<Joint>; 4],
    queued_joint_count: usize,
    last_heartbeat_ms: Option<u32>,
    servo_voltage_mv: Option<u16>,
    current_deg: [u16; 4],
    target_deg: [u16; 4],
}

impl ArmController {
    pub fn new(config: ArmConfig) -> Self {
        let home = [
            config.joints[0].home_deg,
            config.joints[1].home_deg,
            config.joints[2].home_deg,
            config.joints[3].home_deg,
        ];

        Self {
            config,
            armed: false,
            fault: None,
            active_joint: None,
            queued_joints: [None, None, None, None],
            queued_joint_count: 0,
            last_heartbeat_ms: None,
            servo_voltage_mv: None,
            current_deg: home,
            target_deg: home,
        }
    }

    pub fn arm(&mut self, now_ms: u32) -> Result<(), CommandError> {
        if let Some(fault) = self.fault {
            return Err(CommandError::Faulted(fault));
        }

        self.armed = true;
        self.last_heartbeat_ms = Some(now_ms);
        Ok(())
    }

    pub fn heartbeat(&mut self, now_ms: u32) {
        self.last_heartbeat_ms = Some(now_ms);
    }

    pub fn observe_servo_voltage_mv(&mut self, servo_voltage_mv: u16) {
        self.servo_voltage_mv = Some(servo_voltage_mv);

        if servo_voltage_mv < self.config.servo_voltage_min_mv {
            self.latch_fault(Fault::Brownout);
        }
    }

    pub fn disarm(&mut self) {
        self.armed = false;
        self.last_heartbeat_ms = None;
        self.freeze_motion();
    }

    pub fn clear_fault(&mut self, now_ms: u32) -> Result<(), CommandError> {
        if self.armed {
            if self.fault == Some(Fault::HeartbeatTimedOut) && !self.is_moving() {
                self.fault = None;
                self.last_heartbeat_ms = Some(now_ms);
                return Ok(());
            }

            return Err(CommandError::MustDisarmBeforeClearingFault);
        }

        self.fault = None;
        self.last_heartbeat_ms = None;
        Ok(())
    }

    pub fn queue_target(&mut self, joint: Joint, angle_deg: u16) -> Result<u16, CommandError> {
        if let Some(fault) = self.fault {
            return Err(CommandError::Faulted(fault));
        }

        if !self.armed {
            return Err(CommandError::Disarmed);
        }

        let clamped = self.clamp_joint_target(joint, angle_deg);
        let joint_index = joint.index();

        self.target_deg[joint_index] = clamped;

        if self.current_deg[joint_index] != clamped
            && self.active_joint != Some(joint)
            && !self.is_joint_queued(joint)
        {
            self.queued_joints[self.queued_joint_count] = Some(joint);
            self.queued_joint_count += 1;
        }

        Ok(clamped)
    }

    pub fn handle_command(&mut self, now_ms: u32, command: Command) -> Result<(), CommandError> {
        match command {
            Command::Arm => self.arm(now_ms),
            Command::Disarm => {
                self.disarm();
                Ok(())
            }
            Command::Heartbeat => {
                if let Some(fault) = self.fault {
                    return Err(CommandError::Faulted(fault));
                }

                self.heartbeat(now_ms);
                Ok(())
            }
            Command::ClearFault => self.clear_fault(now_ms),
            Command::Home => {
                self.reset_motion_plan();

                for joint in Joint::ALL {
                    let home_deg = self.config.joints[joint.index()].home_deg;
                    self.queue_target(joint, home_deg)?;
                }
                self.heartbeat(now_ms);
                Ok(())
            }
            Command::OpenGripper => {
                let open_deg = self.config.joints[Joint::Gripper.index()].home_deg;
                self.queue_target(Joint::Gripper, open_deg)?;
                self.heartbeat(now_ms);
                Ok(())
            }
            Command::CloseGripper => {
                self.queue_target(Joint::Gripper, self.config.gripper_closed_deg)?;
                self.heartbeat(now_ms);
                Ok(())
            }
            Command::MoveJoint { joint, angle_deg } => {
                self.queue_target(joint, angle_deg)?;
                self.heartbeat(now_ms);
                Ok(())
            }
        }
    }

    pub fn status_frame(&self, seq: u32) -> StatusFrame {
        StatusFrame {
            seq,
            snapshot: self.snapshot(),
        }
    }

    pub fn tick(&mut self, now_ms: u32) {
        if self.fault.is_some() {
            return;
        }

        if let Some(last_heartbeat_ms) = self.last_heartbeat_ms {
            if now_ms.saturating_sub(last_heartbeat_ms) > self.config.heartbeat_timeout_ms {
                self.latch_fault(Fault::HeartbeatTimedOut);
                return;
            }
        }

        if !self.armed {
            return;
        }

        if let Some(joint) = self.active_joint {
            if self.current_deg[joint.index()] == self.target_deg[joint.index()] {
                self.active_joint = None;
            }
        }

        if self.active_joint.is_none() {
            self.active_joint = self.pop_next_joint();
        }

        if let Some(joint) = self.active_joint {
            self.step_joint_toward_target(joint);
        }
    }

    pub fn snapshot(&self) -> Snapshot {
        Snapshot {
            armed: self.armed,
            fault: self.fault,
            moving: self.is_moving(),
            active_joint: self.active_joint,
            servo_voltage_mv: self.servo_voltage_mv,
            current_deg: self.current_deg,
            target_deg: self.target_deg,
        }
    }

    fn clamp_joint_target(&self, joint: Joint, angle_deg: u16) -> u16 {
        let limits = self.config.joints[joint.index()];

        if angle_deg < limits.min_deg {
            limits.min_deg
        } else if angle_deg > limits.max_deg {
            limits.max_deg
        } else {
            angle_deg
        }
    }

    fn is_joint_queued(&self, joint: Joint) -> bool {
        let mut index = 0;

        while index < self.queued_joint_count {
            if self.queued_joints[index] == Some(joint) {
                return true;
            }
            index += 1;
        }

        false
    }

    fn pop_next_joint(&mut self) -> Option<Joint> {
        if self.queued_joint_count == 0 {
            return None;
        }

        let joint = self.queued_joints[0];
        let mut index = 1;

        while index < self.queued_joint_count {
            self.queued_joints[index - 1] = self.queued_joints[index];
            index += 1;
        }

        self.queued_joint_count -= 1;
        self.queued_joints[self.queued_joint_count] = None;
        joint
    }

    fn step_joint_toward_target(&mut self, joint: Joint) {
        let index = joint.index();
        let current = self.current_deg[index];
        let target = self.target_deg[index];
        let step = self.config.joints[index].max_step_deg;

        if current < target {
            let next = current.saturating_add(step);
            self.current_deg[index] = if next > target { target } else { next };
            return;
        }

        if current > target {
            let next = current.saturating_sub(step);
            self.current_deg[index] = if next < target { target } else { next };
        }
    }

    fn is_moving(&self) -> bool {
        self.active_joint.is_some()
            || self.queued_joint_count > 0
            || self.current_deg != self.target_deg
    }

    fn latch_fault(&mut self, fault: Fault) {
        self.fault = Some(fault);
        self.last_heartbeat_ms = None;
        self.freeze_motion();
    }

    fn freeze_motion(&mut self) {
        self.reset_motion_plan();
        self.target_deg = self.current_deg;
    }

    fn reset_motion_plan(&mut self) {
        self.active_joint = None;
        self.queued_joints = [None, None, None, None];
        self.queued_joint_count = 0;
    }
}
