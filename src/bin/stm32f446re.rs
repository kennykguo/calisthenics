// ABOUTME: Boots the STM32F446RE firmware shell around the tested arm safety core.
// ABOUTME: Polls UART, drives servo PWM outputs, and emits status over a provisional Nucleo pin map.

#![no_main]
#![no_std]

use core::{cell::RefCell, fmt::Write};

use arm_firmware::{
    ArmConfig, ByteQueue, Executor, Joint, Platform, ServoPwmConfig, ServoRailSense,
};
use cortex_m::{
    interrupt::{self as cm_interrupt, Mutex},
    peripheral::NVIC,
};
use cortex_m_rt::entry;
use panic_halt as _;
use stm32f4xx_hal::{
    adc::{
        Adc,
        config::{AdcConfig, SampleTime},
    },
    pac::{self, interrupt},
    prelude::*,
    rcc::Config,
    serial::{Event as SerialEvent, Tx, config::Config as SerialConfig},
    timer::PwmChannel,
};

const UART_BAUD_RATE: u32 = 115_200;
const UART_RX_QUEUE_CAPACITY: usize = 128;
const CONTROL_PERIOD_MS: u32 = 20;
const STATUS_PERIOD_MS: u32 = 100;
const LOOP_DELAY_MS: u32 = 1;
const SERVO_RAIL_SAMPLE_PERIOD_MS: u32 = 100;
const SERVO_PWM: ServoPwmConfig = ServoPwmConfig {
    period_us: 20_000,
    min_pulse_us: 500,
    max_pulse_us: 2_500,
    max_angle_deg: 180,
};
const SERVO_RAIL_SENSE: ServoRailSense = ServoRailSense::new(10_000, 10_000);

type ServoBase = PwmChannel<pac::TIM3, 0>;
type ServoShoulder = PwmChannel<pac::TIM3, 1>;
type ServoElbow = PwmChannel<pac::TIM3, 2>;
type ServoGripper = PwmChannel<pac::TIM3, 3>;

static UART_RX_QUEUE: Mutex<RefCell<ByteQueue<UART_RX_QUEUE_CAPACITY>>> =
    Mutex::new(RefCell::new(ByteQueue::new()));

struct BoardPlatform {
    tx: Tx<pac::USART2, u8>,
    base: ServoBase,
    shoulder: ServoShoulder,
    elbow: ServoElbow,
    gripper: ServoGripper,
    pwm_period_ticks: u16,
}

impl BoardPlatform {
    fn new(
        tx: Tx<pac::USART2, u8>,
        mut base: ServoBase,
        mut shoulder: ServoShoulder,
        mut elbow: ServoElbow,
        mut gripper: ServoGripper,
    ) -> Self {
        base.enable();
        shoulder.enable();
        elbow.enable();
        gripper.enable();

        let pwm_period_ticks = base.get_max_duty();

        Self {
            tx,
            base,
            shoulder,
            elbow,
            gripper,
            pwm_period_ticks,
        }
    }

    fn try_read_byte(&mut self) -> Option<u8> {
        cm_interrupt::free(|cs| UART_RX_QUEUE.borrow(cs).borrow_mut().pop())
    }

    fn angle_to_duty(&self, angle_deg: u16) -> u16 {
        SERVO_PWM.duty_for_angle(angle_deg, self.pwm_period_ticks)
    }
}

impl Platform for BoardPlatform {
    fn set_servo_enabled(&mut self, joint: Joint, enabled: bool) {
        match (joint, enabled) {
            (Joint::Base, true) => self.base.enable(),
            (Joint::Base, false) => self.base.disable(),
            (Joint::Shoulder, true) => self.shoulder.enable(),
            (Joint::Shoulder, false) => self.shoulder.disable(),
            (Joint::Elbow, true) => self.elbow.enable(),
            (Joint::Elbow, false) => self.elbow.disable(),
            (Joint::Gripper, true) => self.gripper.enable(),
            (Joint::Gripper, false) => self.gripper.disable(),
        }
    }

    fn set_servo_angle(&mut self, joint: Joint, angle_deg: u16) {
        let duty = self.angle_to_duty(angle_deg);

        match joint {
            Joint::Base => self.base.set_duty(duty),
            Joint::Shoulder => self.shoulder.set_duty(duty),
            Joint::Elbow => self.elbow.set_duty(duty),
            Joint::Gripper => self.gripper.set_duty(duty),
        }
    }

    fn transmit_line(&mut self, line: &str) {
        let _ = write!(self.tx, "{line}\r\n");
    }
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.freeze(
        Config::hsi()
            .sysclk(84.MHz())
            .pclk1(42.MHz())
            .pclk2(84.MHz()),
    );

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let servo_rail_pin = gpioa.pa0.into_analog();

    let mut serial = dp
        .USART2
        .serial(
            (gpioa.pa2, gpioa.pa3),
            SerialConfig::default().baudrate(UART_BAUD_RATE.bps()),
            &mut rcc,
        )
        .unwrap()
        .with_u8_data();
    serial.listen(SerialEvent::RxNotEmpty);
    let (tx, _rx) = serial.split();

    let (_, (base, shoulder, elbow, gripper)) =
        dp.TIM3.pwm_us(SERVO_PWM.period_us.micros(), &mut rcc);
    let base = base.with(gpioa.pa6);
    let shoulder = shoulder.with(gpioa.pa7);
    let elbow = elbow.with(gpiob.pb0);
    let gripper = gripper.with(gpiob.pb1);
    let mut adc = Adc::new(dp.ADC1, true, AdcConfig::default(), &mut rcc);

    let mut platform = BoardPlatform::new(tx, base, shoulder, elbow, gripper);
    let mut executor = Executor::<64>::new(
        ArmConfig::f446re_mg90s(),
        CONTROL_PERIOD_MS,
        STATUS_PERIOD_MS,
    );
    let mut delay = cp.SYST.delay(&rcc.clocks);
    let mut now_ms = 0_u32;
    let mut last_servo_rail_sample_ms = 0_u32;

    unsafe {
        NVIC::unmask(pac::Interrupt::USART2);
    }

    loop {
        while let Some(byte) = platform.try_read_byte() {
            executor.receive_byte(&mut platform, now_ms, byte);
        }

        if now_ms.saturating_sub(last_servo_rail_sample_ms) >= SERVO_RAIL_SAMPLE_PERIOD_MS {
            last_servo_rail_sample_ms = now_ms;

            let pin_sample = adc.convert(&servo_rail_pin, SampleTime::Cycles_480);
            let pin_mv = adc.sample_to_millivolts(pin_sample);
            let servo_rail_mv = SERVO_RAIL_SENSE.rail_mv_from_pin_mv(pin_mv);

            executor.observe_servo_voltage_mv(&mut platform, servo_rail_mv);
        }

        executor.tick(&mut platform, now_ms);

        delay.delay_ms(LOOP_DELAY_MS);
        now_ms = now_ms.wrapping_add(LOOP_DELAY_MS);
    }
}

#[interrupt]
fn USART2() {
    let usart = unsafe { &*pac::USART2::ptr() };

    loop {
        let status = usart.sr().read();

        if status.pe().bit_is_set()
            || status.fe().bit_is_set()
            || status.nf().bit_is_set()
            || status.ore().bit_is_set()
        {
            let _ = usart.dr().read();
            continue;
        }

        if !status.rxne().bit_is_set() {
            break;
        }

        let byte = usart.dr().read().dr().bits() as u8;
        cm_interrupt::free(|cs| {
            let _ = UART_RX_QUEUE.borrow(cs).borrow_mut().push(byte);
        });
    }
}
