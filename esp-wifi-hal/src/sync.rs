use core::{
    cell::RefCell,
    future::{poll_fn, Future},
    ops::{Deref, Range},
    task::Poll,
};

use embassy_sync::blocking_mutex;
use esp_hal::asynch::AtomicWaker;
use portable_atomic::{AtomicU8, AtomicUsize, Ordering};

use crate::{ll::HardwareTxResult, DefaultRawMutex};

/// A primitive for signaling the status of a TX slot.
pub struct HardwareTxResultSignal {
    state: AtomicU8,
    waker: AtomicWaker,
}
impl HardwareTxResultSignal {
    /// A transmission is currently pending or the slot is inactive.
    const PENDING_OR_INACTIVE: u8 = 0;
    /// The tranmission has completed successfully.
    const SUCCESS: u8 = 1;
    /// A timeout occured while transmitting.
    const TIMEOUT: u8 = 2;
    /// A collision occured while transmitting.
    const COLLISION: u8 = 3;
    /// Create a new state signal.
    pub const fn new() -> Self {
        Self {
            state: AtomicU8::new(Self::PENDING_OR_INACTIVE),
            waker: AtomicWaker::new(),
        }
    }
    /// Reset the slot state signal.
    pub fn reset(&self) {
        self.state
            .store(Self::PENDING_OR_INACTIVE, Ordering::Relaxed);
    }
    /// Signal a slot state.
    pub fn signal(&self, slot_status: HardwareTxResult) {
        self.state.store(
            match slot_status {
                HardwareTxResult::Success => Self::SUCCESS,
                HardwareTxResult::Timeout => Self::TIMEOUT,
                HardwareTxResult::Collision => Self::COLLISION,
            },
            Ordering::Relaxed,
        );
        self.waker.wake();
    }
    /// Wait for a slot state change.
    pub fn wait(&self) -> impl Future<Output = HardwareTxResult> + use<'_> {
        poll_fn(|cx| {
            let state = self.state.load(Ordering::Acquire);
            if state != Self::PENDING_OR_INACTIVE {
                self.reset();
                Poll::Ready(match state {
                    Self::SUCCESS => HardwareTxResult::Success,
                    Self::TIMEOUT => HardwareTxResult::Timeout,
                    Self::COLLISION => HardwareTxResult::Collision,
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

/// This is a slot borrowed from the slot queue.
///
/// It is used, to make sure that access to a slot is exclusive.
/// It will return the slot back into the slot queue once dropped.
pub(crate) struct BorrowedTxSlot<'a> {
    state: &'a blocking_mutex::Mutex<DefaultRawMutex, RefCell<u8>>,
    slot: usize,
}
impl Deref for BorrowedTxSlot<'_> {
    type Target = usize;
    fn deref(&self) -> &Self::Target {
        &self.slot
    }
}
impl Drop for BorrowedTxSlot<'_> {
    fn drop(&mut self) {
        // We can ignore the result here, because we know that this slot was taken from the queue,
        // and therefore the queue must have space for it.
        self.state.lock(|rc| {
            *rc.borrow_mut() |= 1 << self.slot;
        });
        trace!("Slot {} is now free again.", self.slot);
    }
}
/// This keeps track of all the TX slots available, by using a queue of slot numbers in the
/// background, which makes it possible to await a slot becoming free.
pub(crate) struct TxSlotQueue {
    state: blocking_mutex::Mutex<DefaultRawMutex, RefCell<u8>>,
    waker: AtomicWaker,
}
impl TxSlotQueue {
    /// Create a new slot manager.
    pub fn new(slots: Range<usize>) -> Self {
        assert!(slots.end <= 5);
        Self {
            state: blocking_mutex::Mutex::new(RefCell::new(
                slots.fold(0u8, |acc, bit_index| acc | (1 << bit_index)),
            )),
            waker: AtomicWaker::new(),
        }
    }
    /// Asynchronously wait for a new slot to become available.
    pub async fn wait_for_slot(&self) -> BorrowedTxSlot<'_> {
        let slot = poll_fn(|cx| {
            self.state.lock(|rc| {
                let mut state = rc.borrow_mut();
                let trailing_zeros = state.trailing_zeros();
                if trailing_zeros == 8 {
                    self.waker.register(cx.waker());
                    Poll::Pending
                } else {
                    *state &= !(1 << trailing_zeros);
                    Poll::Ready(trailing_zeros as usize)
                }
            })
        })
        .await;
        BorrowedTxSlot {
            state: &self.state,
            slot,
        }
    }
}
/// A drop guard, which executes the provided closure on drop.
pub struct DropGuard<F: FnMut()> {
    drop_closure: F
}
impl<F: FnMut()> DropGuard<F> {
    #[inline]
    /// Create a new drop guard.
    pub const fn new(drop_closure: F) -> Self {
        Self {
            drop_closure
        }
    }
    #[inline]
    /// Defuse the drop guard.
    ///
    /// This will prevent the drop closure from being run.
    pub const fn defuse(self) {
        core::mem::forget(self);
    }
}
impl<F: FnMut()> Drop for DropGuard<F> {
    fn drop(&mut self) {
        (self.drop_closure)();
    }
}
