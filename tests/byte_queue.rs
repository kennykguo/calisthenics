// ABOUTME: Verifies the interrupt-safe UART byte queue used by the STM32 board shell.
// ABOUTME: Covers queue order, wraparound behavior, and overflow rejection.

use arm_firmware::ByteQueue;

#[test]
fn preserves_fifo_order() {
    let mut queue = ByteQueue::<4>::new();

    assert!(queue.push(b'A'));
    assert!(queue.push(b'B'));
    assert_eq!(queue.pop(), Some(b'A'));
    assert_eq!(queue.pop(), Some(b'B'));
    assert_eq!(queue.pop(), None);
}

#[test]
fn wraps_around_after_pops() {
    let mut queue = ByteQueue::<3>::new();

    assert!(queue.push(b'A'));
    assert!(queue.push(b'B'));
    assert_eq!(queue.pop(), Some(b'A'));
    assert!(queue.push(b'C'));
    assert!(queue.push(b'D'));

    assert_eq!(queue.pop(), Some(b'B'));
    assert_eq!(queue.pop(), Some(b'C'));
    assert_eq!(queue.pop(), Some(b'D'));
    assert_eq!(queue.pop(), None);
}

#[test]
fn rejects_bytes_when_full() {
    let mut queue = ByteQueue::<2>::new();

    assert!(queue.push(b'A'));
    assert!(queue.push(b'B'));
    assert!(!queue.push(b'C'));

    assert_eq!(queue.pop(), Some(b'A'));
    assert_eq!(queue.pop(), Some(b'B'));
    assert_eq!(queue.pop(), None);
}
