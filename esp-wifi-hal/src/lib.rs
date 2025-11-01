//! # `esp-wifi-hal`
//! This is an experimental driver for the Wi-Fi peripheral on ESP32-series chips.
//! Currently we support both the plain ESP32 and the ESP32-S2.
//! ## Hardware overview
//! This chapter will give a short overview of the structure of the Wi-Fi MAC peripheral.
//!
//! ### Receive (RX)
//! Receiving frames is fairly simple. We pass the hardware a pointer to a DMA descriptor, which
//! then points to the next descriptor and so on. We call this the RX DMA list.
//!
//! Inside the driver, we initialize this list and pass it to the hardware. When an interrupt is
//! received and the status code indicates, that a frame was received, we internally increase the
//! number of frames, that are currently waiting in the RX queue. Internally, [WiFi::receive]
//! asynchronously waits for a frame to be received and takes the first descriptor out of the list.
//! For unknown reasons, sometimes these frames can be empty, so we loop and wait for a valid frame
//! to be received. That DMA descriptor is then wrapped into a [BorrowedBuffer], which is then
//! returned. Once that [BorrowedBuffer] is dropped, it will automatically be appended to the end
//! of the DMA list.
//!
//! ### Transmit (TX)
//! Transmitting frames also makes use of DMA descriptors, but in a slightly different way. There
//! are five hardware transmission slots, which consist of a number of registers, that are repeated
//! five times. The slot a frame is transmitted on is assigned through a queue, which allows
//! asynchronously waiting for a slot to become available. Once a slot has been acquired, we pin a
//! DMA descriptor onto the stack and write it's address to a register. Other parameters, like data
//! rate, encryption key ID, HT-SIG field etc., are also written to registers for that TX slot.
//! Another bunch of parameters indicate, if we should wait for an ACK to be received. Once all
//! parameters are set, we set two bits, that mark the slot as valid an ready for tranmission. We
//! then wait for an interrupt to arrive, which indicates the transmission to have ended in one of
//! three states: Complete, Timeout, Collision. We do not know what the collision state means, but
//! we can handle it anyways.
//!
//! ### Filtering
//! The central element of the MAC are the MAC address filters. They aren't only relevant for RX,
//! but also for TX, Cryptography, TSF etc.
//! Depending on the chip, there are either three or four of these filters.
//! Each of these filters can filter for receiver address (RA) and BSSID.
//! This means, that the hardware can filter out and ACK frames for four different RAs and BSSIDs
//! at the same time. We call each of these filters a virtual interface (VIF), since they
//! technically allow us to run these interfaces independently of each other, but don't exist as
//! separate RX chains in hardware.
//! During TX it's also possible to specify which VIF the tranmssion is for, so that the hardware
//! can wait for an ACK to the RA associated with that interface.
//!
//! The way this MAC filtering works is relatively complicated and not fully understood yet. On
//! some chips (including the ESP32 and ESP32-S2), it's possible to specify an address mask,
//! which indicates the bits, that should be taken in to account. There are also bits, that make
//! certain types of frames pass the filter, which we call the scanning mode.

#![no_std]
#![allow(unexpected_cfgs)]
#![deny(missing_docs)]

#[macro_use]
extern crate defmt_or_log;

// This fixes defmt_or_log messing with important macros used in const eval.
#[macro_use]
extern crate core;

mod crypto;
mod dma_list;
mod ffi;
pub mod ll;
mod rates;
mod sync;
mod wmac;

#[cfg(feature = "esp32")]
use esp32 as esp_pac;
#[cfg(feature = "esp32s2")]
use esp32s2 as esp_pac;

pub use crypto::*;
pub use dma_list::WiFiResources;
pub use ll::{INTERFACE_COUNT, KEY_SLOT_COUNT, RxFilterBank};
pub use rates::*;
pub use wmac::*;

#[cfg(not(feature = "critical_section"))]
type DefaultRawMutex = embassy_sync::blocking_mutex::raw::NoopRawMutex;
#[cfg(feature = "critical_section")]
type DefaultRawMutex = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
