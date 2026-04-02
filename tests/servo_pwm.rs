// ABOUTME: Verifies hobby-servo pulse conversion used by the STM32 board shell.
// ABOUTME: Covers angle clamping and mapping from abstract angles to timer duty values.

use arm_firmware::ServoPwmConfig;

fn test_config() -> ServoPwmConfig {
    ServoPwmConfig {
        period_us: 20_000,
        min_pulse_us: 500,
        max_pulse_us: 2_500,
        max_angle_deg: 180,
    }
}

#[test]
fn maps_angles_to_servo_pulse_widths() {
    let config = test_config();

    assert_eq!(config.pulse_us_for_angle(0), 500);
    assert_eq!(config.pulse_us_for_angle(90), 1_500);
    assert_eq!(config.pulse_us_for_angle(180), 2_500);
}

#[test]
fn clamps_out_of_range_angles() {
    let config = test_config();

    assert_eq!(config.pulse_us_for_angle(200), 2_500);
}

#[test]
fn converts_pulse_widths_into_timer_duty_ticks() {
    let config = test_config();

    assert_eq!(config.duty_for_angle(0, 20_000), 500);
    assert_eq!(config.duty_for_angle(90, 20_000), 1_500);
    assert_eq!(config.duty_for_angle(180, 20_000), 2_500);
}
