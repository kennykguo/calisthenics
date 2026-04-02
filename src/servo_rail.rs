// ABOUTME: Converts the servo-rail sense voltage seen by the ADC back into actual rail millivolts.
// ABOUTME: Keeps the brownout threshold tied to the physical resistor divider on the board.

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ServoRailSense {
    pub top_ohms: u32,
    pub bottom_ohms: u32,
}

impl ServoRailSense {
    pub const fn new(top_ohms: u32, bottom_ohms: u32) -> Self {
        Self {
            top_ohms,
            bottom_ohms,
        }
    }

    pub fn rail_mv_from_pin_mv(self, pin_mv: u16) -> u16 {
        let total_ohms = self.top_ohms.saturating_add(self.bottom_ohms);
        let rail_mv = u32::from(pin_mv).saturating_mul(total_ohms) / self.bottom_ohms.max(1);

        rail_mv.min(u32::from(u16::MAX)) as u16
    }
}
