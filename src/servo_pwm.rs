// ABOUTME: Converts abstract servo angles into pulse widths and timer duty ticks.
// ABOUTME: Keeps hobby-servo signal math testable outside the STM32 hardware shell.

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ServoPwmConfig {
    pub period_us: u32,
    pub min_pulse_us: u32,
    pub max_pulse_us: u32,
    pub max_angle_deg: u16,
}

impl ServoPwmConfig {
    pub fn pulse_us_for_angle(&self, angle_deg: u16) -> u32 {
        let clamped_angle = u32::from(angle_deg.min(self.max_angle_deg));
        let pulse_span_us = self.max_pulse_us - self.min_pulse_us;

        self.min_pulse_us + ((clamped_angle * pulse_span_us) / u32::from(self.max_angle_deg))
    }

    pub fn duty_for_angle(&self, angle_deg: u16, pwm_period_ticks: u16) -> u16 {
        let pulse_us = self.pulse_us_for_angle(angle_deg);
        ((pulse_us * u32::from(pwm_period_ticks)) / self.period_us) as u16
    }
}
