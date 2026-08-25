//! TC66 custom firmware — M0 on the ecosystem USB stack.
//!
//! usb-device + usbd-serial + stm32-usbd (via the small UsbPeripheral impl in
//! usb_bus.rs) provide CDC-ACM; we keep our own clock bring-up and the PA9 D+
//! soft-connect choreography (board.rs), the SRAM marker debugging (markers.rs),
//! and the polled main loop dispatching the bare-5-byte commands:
//!   query -> "firm", getva -> 192-B static packet, updat -> Rust SRAM flasher
//!   (flasher.rs), rotat/nextp/lastp -> silent no-ops.
//!
//! Endpoint layout is whatever the stack allocates (expected: EP1 = interrupt IN,
//! EP2 = bulk OUT+IN); `updat` discovers the bulk endpoint by scanning EPnR and
//! the flasher reads its PMA buffers from the BTABLE, so nothing is hard-coded.
//! Interrupts stay masked (startup.rs); everything runs off dev.poll().

#![no_std]
#![no_main]

mod board;
mod flasher;
mod getva;
mod i2c;
mod markers;
mod startup;
mod usb_bus;

use cortex_m_rt::entry;
use gd32f1::gd32f150 as pac;
use markers::*;
use usb_device::device::{StringDescriptors, UsbDeviceBuilder, UsbRev, UsbVidPid};
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

/// Bumped to prove a self-hosted reflash actually took (read at MK_BUILD).
const BUILD_ID: u32 = 8;

type Dev<'a> = UsbDevice<'a, usb_bus::UsbBus>;
type Serial<'a> = SerialPort<'a, usb_bus::UsbBus>;

#[entry]
fn main() -> ! {
    mk(MK_BOOT, 0xB007_B007);
    mk(MK_BUILD, BUILD_ID);
    mk(MK_HEART, 0);
    mk(MK_CMDCNT, 0);
    mk(MK_ENUM, 0);

    let p = pac::Peripherals::take().unwrap();
    board::clock_init(&p.rcu);

    /* Force a fresh enumeration of THIS app: the bootloader handed off with D+
     * still pulled up and the host bound to the bootloader. Disconnect, bring
     * the stack up, reconnect. */
    board::pa9_config_output(&p.rcu, &p.gpioa);
    board::pa9_low(&p.gpioa);
    board::delay_ms(120);

    let usb_alloc = usb_bus::UsbBus::new(usb_bus::Gd32Usb);
    let mut serial = SerialPort::new(&usb_alloc);
    let mut dev = UsbDeviceBuilder::new(&usb_alloc, UsbVidPid(0x28E9, 0x018A))
        .strings(&[StringDescriptors::default()
            .manufacturer("RuiDeng")
            .product("TC66 USB Type-C Meter")
            .serial_number("TC66")])
        .unwrap()
        .device_class(USB_CLASS_CDC)
        .device_release(0x0100)
        .usb_rev(UsbRev::Usb200)
        .max_packet_size_0(64)
        .unwrap()
        .build(); /* build() enables the peripheral (CNTR power-up, BTABLE=0) */
    mk(MK_STAGE, STAGE_USB_INIT);
    board::pa9_high(&p.gpioa);

    let mut buf = [0u8; 64];
    loop {
        mk(MK_HEART, mk_get(MK_HEART) + 1);
        if !dev.poll(&mut [&mut serial]) {
            continue;
        }
        if dev.state() == UsbDeviceState::Configured && mk_get(MK_STAGE) < STAGE_CONFIG {
            mk(MK_STAGE, STAGE_CONFIG);
        }
        if let Ok(n) = serial.read(&mut buf) {
            if n > 0 {
                on_packet(&mut dev, &mut serial, &buf[..n]);
            }
        }
    }
}

#[inline(never)]
fn match5(b: &[u8], cmd: &[u8; 5]) -> bool {
    b.len() >= 5 && b[..5] == cmd[..]
}

#[inline(never)]
fn on_packet(dev: &mut Dev, serial: &mut Serial, buf: &[u8]) {
    if buf.len() >= 4 {
        mk(
            MK_CMD,
            buf[0] as u32 | (buf[1] as u32) << 8 | (buf[2] as u32) << 16 | (buf[3] as u32) << 24,
        );
    }
    mk(MK_CMDCNT, mk_get(MK_CMDCNT) + 1);
    if mk_get(MK_STAGE) < STAGE_CMD {
        mk(MK_STAGE, STAGE_CMD);
    }

    if match5(buf, b"query") {
        write_all(dev, serial, b"firm");
        mk(MK_STAGE, STAGE_QUERY);
    } else if match5(buf, b"getva") {
        write_all(dev, serial, &getva::GETVA_REPLY);
    } else if match5(buf, b"i2crd") {
        /* dev/bring-up verb (ours, not stock):
         *   i2crd <addr7> <wlen 0..=2> <w0> <w1> <rlen 0..=32>
         * -> "OK" + rlen bytes, or "ER" + [stage, stat0.lo, stat0.hi, stat1.lo] */
        let mut reply = [0u8; 34];
        let n = i2c_command(buf, &mut reply);
        write_all(dev, serial, &reply[..n]);
    } else if match5(buf, b"updat") {
        if let Some(ep) = find_bulk_ep() {
            flasher::launch(ep); /* does not return */
        }
        mk(MK_PROBE, 0xBAD_EB); /* no bulk endpoint found — refuse, host times out */
    }
    /* rotat / nextp / lastp / unknown -> silent */
}

/// Parse + run an `i2crd` packet; fill `reply`, return its length.
#[inline(never)]
fn i2c_command(buf: &[u8], reply: &mut [u8; 34]) -> usize {
    if buf.len() < 10 {
        reply[..2].copy_from_slice(b"ER");
        reply[2] = 0x0F; /* bad args */
        return 3;
    }
    let addr7 = buf[5];
    let wlen = buf[6] as usize;
    let wbuf = [buf[7], buf[8]];
    let rlen = buf[9] as usize;
    if wlen > 2 || rlen > 32 {
        reply[..2].copy_from_slice(b"ER");
        reply[2] = 0x0F;
        return 3;
    }
    let mut rbuf = [0u8; 32];
    match i2c::transact(addr7, &wbuf[..wlen], &mut rbuf[..rlen]) {
        Ok(()) => {
            reply[..2].copy_from_slice(b"OK");
            reply[2..2 + rlen].copy_from_slice(&rbuf[..rlen]);
            2 + rlen
        }
        Err(e) => {
            reply[..2].copy_from_slice(b"ER");
            reply[2] = e.stage;
            reply[3] = e.stat0 as u8;
            reply[4] = (e.stat0 >> 8) as u8;
            reply[5] = e.stat1 as u8;
            6
        }
    }
}

/// Blocking-ish write with an absent-host bound (the C had the same bounded wait):
/// push bytes into usbd-serial's buffer and keep polling until it drains.
#[inline(never)]
fn write_all(dev: &mut Dev, serial: &mut Serial, data: &[u8]) {
    let mut rest = data;
    let mut budget = 200_000u32;
    while budget > 0 {
        budget -= 1;
        if !rest.is_empty() {
            match serial.write(rest) {
                Ok(n) => rest = &rest[n..],
                Err(UsbError::WouldBlock) => {}
                Err(_) => return,
            }
        } else {
            match serial.flush() {
                Ok(()) => return,
                Err(UsbError::WouldBlock) => {}
                Err(_) => return,
            }
        }
        dev.poll(&mut [serial]);
    }
}

/// Find the CDC bulk data endpoint: the EPnCS whose type (EP_CTL) is BULK (00)
/// with its RX side enabled (RX_STA != disabled). Unconfigured EPnCS read 0
/// (RX disabled), EP0 is CONTROL, the notification endpoint is INTERRUPT — so
/// this is unambiguous. Read-only peek at registers the USB stack owns.
#[inline(never)]
fn find_bulk_ep() -> Option<u32> {
    let usbd = unsafe { &*pac::Usbd::ptr() };
    for n in 1..8u32 {
        let cs = match n {
            1 => usbd.ep1cs().read().bits(),
            2 => usbd.ep2cs().read().bits(),
            3 => usbd.ep3cs().read().bits(),
            4 => usbd.ep4cs().read().bits(),
            5 => usbd.ep5cs().read().bits(),
            6 => usbd.ep6cs().read().bits(),
            _ => usbd.ep7cs().read().bits(),
        };
        if cs & 0x0600 == 0x0000 && cs & 0x3000 != 0 {
            return Some(n);
        }
    }
    None
}
