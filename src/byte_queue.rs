// ABOUTME: Stores incoming UART bytes in a fixed-capacity ring buffer without heap allocation.
// ABOUTME: Lets interrupt handlers enqueue serial data while the main loop drains it safely later.

pub struct ByteQueue<const N: usize> {
    bytes: [u8; N],
    head: usize,
    tail: usize,
    len: usize,
}

impl<const N: usize> ByteQueue<N> {
    pub const fn new() -> Self {
        Self {
            bytes: [0; N],
            head: 0,
            tail: 0,
            len: 0,
        }
    }

    pub fn push(&mut self, byte: u8) -> bool {
        if self.len == N {
            return false;
        }

        self.bytes[self.tail] = byte;
        self.tail = (self.tail + 1) % N;
        self.len += 1;
        true
    }

    pub fn pop(&mut self) -> Option<u8> {
        if self.len == 0 {
            return None;
        }

        let byte = self.bytes[self.head];
        self.head = (self.head + 1) % N;
        self.len -= 1;
        Some(byte)
    }
}
