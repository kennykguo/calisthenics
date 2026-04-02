// ABOUTME: Provides the byte-oriented firmware shell around the controller runtime.
// ABOUTME: Assembles UART lines, runs periodic control ticks, and emits transport responses.

use crate::{ArmConfig, Response, ResponseError, Runtime, Snapshot, TransportError};

pub struct App<const N: usize> {
    runtime: Runtime,
    line_buffer: LineBuffer<N>,
    control_period_ms: u32,
    status_period_ms: u32,
    last_control_ms: u32,
    last_status_ms: u32,
}

impl<const N: usize> App<N> {
    pub fn new(config: ArmConfig, control_period_ms: u32, status_period_ms: u32) -> Self {
        Self {
            runtime: Runtime::new(config),
            line_buffer: LineBuffer::new(),
            control_period_ms,
            status_period_ms,
            last_control_ms: 0,
            last_status_ms: 0,
        }
    }

    pub fn receive_byte(&mut self, now_ms: u32, byte: u8) -> Option<Response> {
        match self.line_buffer.push_byte(byte) {
            Ok(Some(line)) => Some(match line.as_str() {
                Ok(line) => self.runtime.process_line(now_ms, line),
                Err(_) => {
                    Response::Error(ResponseError::Transport(TransportError::InvalidEncoding))
                }
            }),
            Ok(None) => None,
            Err(LineBufferError::LineTooLong) => Some(Response::Error(ResponseError::Transport(
                TransportError::LineTooLong,
            ))),
        }
    }

    pub fn observe_servo_voltage_mv(&mut self, servo_voltage_mv: u16) {
        self.runtime.observe_servo_voltage_mv(servo_voltage_mv);
    }

    pub fn poll(&mut self, now_ms: u32) -> Option<Response> {
        while now_ms.saturating_sub(self.last_control_ms) >= self.control_period_ms {
            self.last_control_ms = self.last_control_ms.saturating_add(self.control_period_ms);
            self.runtime.tick(self.last_control_ms);
        }

        if now_ms.saturating_sub(self.last_status_ms) >= self.status_period_ms {
            self.last_status_ms = self.last_status_ms.saturating_add(self.status_period_ms);
            return Some(self.runtime.status());
        }

        None
    }

    pub fn snapshot(&self) -> Snapshot {
        self.runtime.snapshot()
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum LineBufferError {
    LineTooLong,
}

struct LineBuffer<const N: usize> {
    bytes: [u8; N],
    len: usize,
    discarding: bool,
    ignore_next_lf: bool,
}

impl<const N: usize> LineBuffer<N> {
    fn new() -> Self {
        Self {
            bytes: [0; N],
            len: 0,
            discarding: false,
            ignore_next_lf: false,
        }
    }

    fn push_byte(&mut self, byte: u8) -> Result<Option<Line<N>>, LineBufferError> {
        if self.ignore_next_lf {
            self.ignore_next_lf = false;
            if byte == b'\n' {
                return Ok(None);
            }
        }

        if self.discarding {
            if (byte == b'\n') || (byte == b'\r') {
                self.discarding = false;
            }
            return Ok(None);
        }

        if byte == b'\r' {
            let line = Line {
                bytes: self.bytes,
                len: self.len,
            };
            self.len = 0;
            self.ignore_next_lf = true;
            return Ok(Some(line));
        }

        if byte == b'\n' {
            let line = Line {
                bytes: self.bytes,
                len: self.len,
            };
            self.len = 0;
            return Ok(Some(line));
        }

        if self.len >= N {
            self.len = 0;
            self.discarding = true;
            return Err(LineBufferError::LineTooLong);
        }

        self.bytes[self.len] = byte;
        self.len += 1;
        Ok(None)
    }
}

struct Line<const N: usize> {
    bytes: [u8; N],
    len: usize,
}

impl<const N: usize> Line<N> {
    fn as_str(&self) -> Result<&str, core::str::Utf8Error> {
        core::str::from_utf8(&self.bytes[..self.len])
    }
}
