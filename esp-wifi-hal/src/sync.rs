use core::{
    future::{poll_fn, Future},
    task::Poll,
};

use esp_hal::asynch::AtomicWaker;
use portable_atomic::{AtomicU8, AtomicUsize, Ordering};

pub enum TxSlotStatus {
    Done,
    Timeout,
    Collision,
}
pub struct TxSlotStateSignal {
    state: AtomicU8,
    waker: AtomicWaker,
}
impl TxSlotStateSignal {
    const PENDING: u8 = 0;
    const DONE: u8 = 1;
    const TIMEOUT: u8 = 2;
    const COLLISION: u8 = 3;
    pub const fn new() -> Self {
        Self {
            state: AtomicU8::new(Self::PENDING),
            waker: AtomicWaker::new(),
        }
    }
    pub fn reset(&self) {
        self.state.store(Self::PENDING, Ordering::Relaxed);
    }
    pub fn signal(&self, slot_status: TxSlotStatus) {
        self.state.store(
            match slot_status {
                TxSlotStatus::Done => Self::DONE,
                TxSlotStatus::Timeout => Self::TIMEOUT,
                TxSlotStatus::Collision => Self::COLLISION,
            },
            Ordering::Relaxed,
        );
        self.waker.wake();
    }
    pub fn wait(&self) -> impl Future<Output = TxSlotStatus> + use<'_> {
        poll_fn(|cx| {
            let state = self.state.load(Ordering::Acquire);
            if state != Self::PENDING {
                self.reset();
                Poll::Ready(match state {
                    Self::DONE => TxSlotStatus::Done,
                    Self::TIMEOUT => TxSlotStatus::Timeout,
                    Self::COLLISION => TxSlotStatus::Collision,
                    _ => unreachable!(),
                })
            } else {
                self.waker.register(cx.waker());
                Poll::Pending
            }
        })
    }
}

/// A synchronization primitive, which allows queueing a number signals, to be awaited.
pub struct SignalQueue {
    waker: AtomicWaker,
    queued_signals: AtomicUsize,
}
impl SignalQueue {
    pub const fn new() -> Self {
        Self {
            waker: AtomicWaker::new(),
            queued_signals: AtomicUsize::new(0),
        }
    }
    /// Increments the queue signals by one.
    pub fn put(&self) {
        self.queued_signals.fetch_add(1, Ordering::Relaxed);
        self.waker.wake();
    }
    /// Reset the amount of signals in the queue back to zero.
    pub fn reset(&self) {
        self.queued_signals.store(0, Ordering::Relaxed);
    }
    /// Asynchronously wait for the next signal.
    pub async fn next(&self) {
        poll_fn(|cx| {
            let queued_signals = self.queued_signals.load(Ordering::Relaxed);
            if queued_signals == 0 {
                self.waker.register(cx.waker());
                Poll::Pending
            } else {
                self.queued_signals
                    .store(queued_signals - 1, Ordering::Relaxed);
                Poll::Ready(())
            }
        })
        .await
    }
}
