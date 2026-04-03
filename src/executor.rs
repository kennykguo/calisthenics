// ABOUTME: Bridges the firmware app state machine to platform-specific servo and serial I/O.
// ABOUTME: Applies current joint targets to servo outputs and forwards response lines over telemetry.

use crate::{App, ArmConfig, Joint, Response};

pub trait Platform {
    fn set_servo_enabled(&mut self, joint: Joint, enabled: bool);
    fn set_servo_angle(&mut self, joint: Joint, angle_deg: u16);
    fn transmit_line(&mut self, line: &str);
}

pub struct Executor<const N: usize> {
    app: App<N>,
}

impl<const N: usize> Executor<N> {
    pub fn new(config: ArmConfig, control_period_ms: u32, status_period_ms: u32) -> Self {
        Self {
            app: App::new(config, control_period_ms, status_period_ms),
        }
    }

    pub fn new_without_periodic_status(config: ArmConfig, control_period_ms: u32) -> Self {
        Self {
            app: App::new_without_periodic_status(config, control_period_ms),
        }
    }

    pub fn receive_byte<P: Platform>(&mut self, platform: &mut P, now_ms: u32, byte: u8) {
        if let Some(response) = self.app.receive_byte(now_ms, byte) {
            self.emit_response(platform, &response);
        }

        self.apply_outputs(platform);
    }

    pub fn observe_servo_voltage_mv<P: Platform>(
        &mut self,
        platform: &mut P,
        servo_voltage_mv: u16,
    ) {
        self.app.observe_servo_voltage_mv(servo_voltage_mv);
        self.apply_outputs(platform);
    }

    pub fn tick<P: Platform>(&mut self, platform: &mut P, now_ms: u32) {
        if let Some(response) = self.app.poll(now_ms) {
            self.emit_response(platform, &response);
        }

        self.apply_outputs(platform);
    }

    fn apply_outputs<P: Platform>(&mut self, platform: &mut P) {
        let snapshot = self.app.snapshot();
        let servo_enabled = snapshot.armed;

        for joint in Joint::ALL {
            platform.set_servo_enabled(joint, servo_enabled);
            platform.set_servo_angle(joint, snapshot.current_deg[joint.index()]);
        }
    }

    fn emit_response<P: Platform>(&self, platform: &mut P, response: &Response) {
        let mut line = ResponseLine::new();
        response.write_to(&mut line).ok();
        platform.transmit_line(line.as_str());
    }
}

struct ResponseLine {
    bytes: [u8; 160],
    len: usize,
}

impl ResponseLine {
    fn new() -> Self {
        Self {
            bytes: [0; 160],
            len: 0,
        }
    }

    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.bytes[..self.len]).unwrap_or("")
    }
}

impl core::fmt::Write for ResponseLine {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        if self.len + bytes.len() > self.bytes.len() {
            return Err(core::fmt::Error);
        }

        self.bytes[self.len..self.len + bytes.len()].copy_from_slice(bytes);
        self.len += bytes.len();
        Ok(())
    }
}
