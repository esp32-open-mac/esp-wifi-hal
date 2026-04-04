use core::{
    future::{poll_fn, Future},
    task::Poll,
};

use esp_hal::asynch::AtomicWaker;
use portable_atomic::{AtomicU8, AtomicUsize, Ordering};

use crate::ll::ChannelAccessError;

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
            .store(Self::PENDING_OR_INACTIVE, Ordering::Release);
    }
    /// Signal a slot state.
    pub fn signal(&self, slot_status: Result<(), ChannelAccessError>) {
        self.state.store(
            match slot_status {
                Ok(()) => Self::SUCCESS,
                Err(ChannelAccessError::Timeout) => Self::TIMEOUT,
                Err(ChannelAccessError::Collision) => Self::COLLISION,
            },
            Ordering::Relaxed,
        );
        self.waker.wake();
    }
    /// Wait for a slot state change.
    pub fn wait(
        &self,
    ) -> impl Future<Output = Result<(), ChannelAccessError>> + Send + Sync + use<'_> {
        poll_fn(|cx| {
            // We need a single core lock here, as the state may internally be updated by the ISR.
            esp_sync::RawMutex::new().lock(|| {
                let state = self.state.load(Ordering::Acquire);
                if state != Self::PENDING_OR_INACTIVE {
                    self.reset();
                    Poll::Ready(match state {
                        Self::SUCCESS => Ok(()),
                        Self::TIMEOUT => Err(ChannelAccessError::Timeout),
                        Self::COLLISION => Err(ChannelAccessError::Collision),
                        _ => unreachable!(),
                    })
                } else {
                    self.waker.register(cx.waker());
                    Poll::Pending
                }
            })
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
        if self.queued_signals.fetch_add(1, Ordering::AcqRel) == 0 {
            // If there were already some signals queued up, there's no need to wake the task again.
            self.waker.wake();
        }
    }
    /// Reset the amount of signals in the queue back to zero.
    pub fn reset(&self) {
        self.queued_signals.store(0, Ordering::Release);
    }
    /// Asynchronously wait for the next signal.
    pub fn next(&self) -> impl Future<Output = ()> + Send + Sync + use<'_> {
        poll_fn(|cx| {
            esp_sync::RawMutex::new().lock(|| {
                let queued_signals = self.queued_signals.load(Ordering::Acquire);
                if queued_signals == 0 {
                    self.waker.register(cx.waker());
                    Poll::Pending
                } else {
                    self.queued_signals
                        .store(queued_signals - 1, Ordering::Release);
                    Poll::Ready(())
                }
            })
        })
    }
}
/// A drop guard, which executes the provided closure on drop.
pub struct DropGuard<F: FnMut()> {
    drop_closure: F,
}
impl<F: FnMut()> DropGuard<F> {
    #[inline]
    /// Create a new drop guard.
    pub const fn new(drop_closure: F) -> Self {
        Self { drop_closure }
    }
    #[allow(unused)]
    #[inline]
    /// Defuse the drop guard.
    ///
    /// This will prevent the drop closure from being run.
    pub const fn defuse(self) {
        core::mem::forget(self);
    }
    #[inline]
    /// Disable the drop guard by running the provided drop closure.
    ///
    /// This exists to explicitly run the drop closure. It's called `detonate`, since that seemed
    /// like the logical counterpart to [Self::defuse].
    pub fn detonate(self) {}
}
impl<F: FnMut()> Drop for DropGuard<F> {
    fn drop(&mut self) {
        (self.drop_closure)();
    }
}
