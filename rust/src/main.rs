//! TC66 custom firmware, Milestone 0 — Rust port of reference-c/.
//!
//! Brings up the 48 MHz clock + USB CDC-ACM (usb.rs) and runs a polled main loop
//! dispatching the bare-5-byte serial commands:
//!   query -> "firm"      (proves the app is running; host tools gate on this)
//!   getva -> 192-B static AES packet
//!   updat -> copy the SRAM streaming flasher to 0x20001400 and jump (no K1)
//!   rotat/nextp/lastp -> silent no-ops (no display in M0)
//!
//! Interrupts stay masked the whole time (startup.rs __pre_init does cpsid i);
//! USB is polled. Debugging is SRAM markers only — see markers.rs.

#![no_std]
#![no_main]

mod flasher;
mod getva;
mod markers;
mod regs;
mod startup;
mod usb;

use cortex_m_rt::entry;
use markers::*;

/// Bumped to prove a self-hosted reflash actually took (read at MK_BUILD).
const BUILD_ID: u32 = 2;

#[entry]
fn main() -> ! {
    mk(MK_BOOT, 0xB007_B007);
    mk(MK_BUILD, BUILD_ID);
    usb::init();
    loop {
        usb::poll();
    }
}

#[inline(never)]
fn match5(b: &[u8], cmd: &[u8; 5]) -> bool {
    b.len() >= 5 && b[..5] == cmd[..]
}

/// Called by usb::handle_ep3 with the received CDC OUT packet.
/// Returns true iff it was `updat` (caller must then NOT re-arm EP3).
#[inline(never)]
pub fn on_packet(buf: &[u8]) -> bool {
    if match5(buf, b"query") {
        usb::cdc_send(b"firm");
        mk(MK_STAGE, STAGE_QUERY);
        return false;
    }
    if match5(buf, b"getva") {
        usb::cdc_send(&getva::GETVA_REPLY[0..64]); /* pac1 */
        usb::cdc_send(&getva::GETVA_REPLY[64..128]); /* pac2 */
        usb::cdc_send(&getva::GETVA_REPLY[128..192]); /* pac3 */
        return false;
    }
    if match5(buf, b"updat") {
        flasher::launch(); /* does not return */
    }
    /* rotat / nextp / lastp / unknown -> silent */
    false
}
