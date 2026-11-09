use core::{
    cell::UnsafeCell,
    task::{Context, Poll},
};

use critical_section::CriticalSection;
use embassy_sync::waitqueue::WakerRegistration;
use esp_hal::ram;

use crate::ll::ChannelAccessError;

struct TxResultSignalState {
    state: u8,
    waker: WakerRegistration,
}
impl TxResultSignalState {
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
            state: Self::PENDING_OR_INACTIVE,
            waker: WakerRegistration::new(),
        }
    }
    /// Reset the slot state signal.
    pub fn reset(&mut self) {
        self.state = Self::PENDING_OR_INACTIVE;
    }
    #[inline]
    /// Signal a slot state.
    pub fn signal(&mut self, slot_status: Result<(), ChannelAccessError>) {
        self.state = match slot_status {
            Ok(()) => Self::SUCCESS,
            Err(ChannelAccessError::Timeout) => Self::TIMEOUT,
            Err(ChannelAccessError::Collision) => Self::COLLISION,
        };
        self.waker.wake();
    }
    /// Poll the signals state.
    pub fn poll(&mut self, cx: &mut Context<'_>) -> Poll<Result<(), ChannelAccessError>> {
        let state = self.state;
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
    }
}
/// A primitive for signaling the status of a TX slot.
pub struct HardwareTxResultSignal {
    inner: critical_section::Mutex<UnsafeCell<TxResultSignalState>>,
}
impl HardwareTxResultSignal {
    /// Create a new state signal.
    pub const fn new() -> Self {
        Self {
            inner: critical_section::Mutex::new(UnsafeCell::new(TxResultSignalState::new())),
        }
    }
    /// Access the inenr state.
    ///
    /// # Safety
    /// This must not be called in a reentrant way.
    unsafe fn access_inner<T>(
        &self,
        cs: CriticalSection<'_>,
        f: impl FnOnce(&mut TxResultSignalState) -> T,
    ) -> T {
        f(unsafe { self.inner.borrow(cs).get().as_mut_unchecked() })
    }
    /// Reset the slot state signal.
    pub fn reset(&self, cs: CriticalSection<'_>) {
        unsafe { self.access_inner(cs, TxResultSignalState::reset) }
    }
    #[inline]
    /// Signal a slot state.
    pub fn signal(&self, cs: CriticalSection<'_>, slot_status: Result<(), ChannelAccessError>) {
        unsafe {
            self.access_inner(cs, |ref_mut| ref_mut.signal(slot_status));
        }
    }

    /// Poll the signals state.
    pub fn poll(
        &self,
        cs: CriticalSection<'_>,
        cx: &mut Context<'_>,
    ) -> Poll<Result<(), ChannelAccessError>> {
        unsafe { self.access_inner(cs, |ref_mut| ref_mut.poll(cx)) }
    }
}

struct SignalQueueState {
    queued_signals: usize,
    waker: WakerRegistration,
}
impl SignalQueueState {
    pub const fn new() -> Self {
        Self {
            queued_signals: 0,
            waker: WakerRegistration::new(),
        }
    }
    /// Increments the queue signals by one.
    pub fn put(&mut self) {
        self.queued_signals += 1;
        if self.queued_signals == 1 {
            // If there were already some signals queued up, there's no need to wake the task again.
            self.waker.wake();
        }
    }
    /// Reset the amount of signals in the queue back to zero.
    pub fn reset(&mut self) {
        self.queued_signals = 0;
    }
    pub fn poll(&mut self, cx: &mut Context<'_>) -> Poll<()> {
        let queued_signals = self.queued_signals;
        if queued_signals == 0 {
            self.waker.register(cx.waker());
            Poll::Pending
        } else {
            self.queued_signals -= 1;
            Poll::Ready(())
        }
    }
}
/// A synchronization primitive, which allows queueing a number signals, to be awaited.
pub struct SignalQueue {
    inner: critical_section::Mutex<UnsafeCell<SignalQueueState>>,
}
impl SignalQueue {
    /// Create a new signal queue.
    pub const fn new() -> Self {
        Self {
            inner: critical_section::Mutex::new(UnsafeCell::new(SignalQueueState::new())),
        }
    }
    /// Access the inenr state.
    ///
    /// # Safety
    /// This must not be called in a reentrant way.
    unsafe fn access_inner<T>(
        &self,
        cs: CriticalSection<'_>,
        f: impl FnOnce(&mut SignalQueueState) -> T,
    ) -> T {
        f(unsafe { self.inner.borrow(cs).get().as_mut_unchecked() })
    }
    #[ram]
    /// Increments the queue signals by one.
    pub fn put(&self, cs: CriticalSection<'_>) {
        unsafe { self.access_inner(cs, SignalQueueState::put) }
    }
    /// Reset the amount of signals in the queue back to zero.
    pub fn reset(&self, cs: CriticalSection<'_>) {
        unsafe { self.access_inner(cs, SignalQueueState::reset) }
    }
    /// Asynchronously wait for the next signal.
    pub fn poll_next(&self, cs: CriticalSection<'_>, cx: &mut Context<'_>) -> Poll<()> {
        unsafe { self.access_inner(cs, |ref_mut| ref_mut.poll(cx)) }
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
