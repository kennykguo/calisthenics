// ABOUTME: Verifies conversion from the ADC sense pin voltage back to the servo rail voltage.
// ABOUTME: Covers the divider math used by the brownout monitor in the STM32 board shell.

use arm_firmware::ServoRailSense;

#[test]
fn reconstructs_the_servo_rail_for_an_equal_divider() {
    let sense = ServoRailSense::new(10_000, 10_000);

    assert_eq!(sense.rail_mv_from_pin_mv(2_500), 5_000);
}

#[test]
fn reconstructs_the_servo_rail_for_an_uneven_divider() {
    let sense = ServoRailSense::new(4_700, 3_300);

    assert_eq!(sense.rail_mv_from_pin_mv(2_062), 4_998);
}

#[test]
fn saturates_when_the_computed_rail_voltage_exceeds_u16_range() {
    let sense = ServoRailSense::new(100_000, 1);

    assert_eq!(sense.rail_mv_from_pin_mv(4_000), u16::MAX);
}
