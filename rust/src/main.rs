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

mod adc;
mod board;
mod flasher;
mod font;
mod getva;
mod i2c;
mod ina;
mod markers;
mod startup;
mod tft;
mod ui;
mod usb_bus;

use cortex_m_rt::entry;
use gd32f1::gd32f150 as pac;
use markers::*;
use usb_device::device::{StringDescriptors, UsbDeviceBuilder, UsbRev, UsbVidPid};
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

/// Bumped to prove a self-hosted reflash actually took (read at MK_BUILD).
const BUILD_ID: u32 = 23;

type Dev<'a> = UsbDevice<'a, usb_bus::UsbBus>;
type Serial<'a> = SerialPort<'a, usb_bus::UsbBus>;

#[entry]
fn main() -> ! {
    mk(MK_BOOT, 0xB007_B007);
    mk(MK_BUILD, BUILD_ID);
    mk(MK_HEART, 0);
    mk(MK_CMDCNT, 0);
    mk(MK_ENUM, 0);
    mk(MK_I2CERR, 0);
    mk(MK_WINDOWS, 0);

    let p = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
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

    /* TFT init + first frame now, while D+ is still low — the init sequence
     * has ~400 ms of fixed delays that must not stall enumeration. On error
     * the display is left off for this boot (verbs can still retry it). */
    let mut tft_alive = ui::draw(0, 0, 0).is_ok();

    board::pa9_high(&p.gpioa);

    /* SysTick paces the sampler at 250 Hz (48 MHz / 192000), polled — no IRQ.
     * One I2C read per tick; a 24-read window publishes at ~10 Hz, so readings
     * are fresh regardless of how fast (or whether) the host polls getva. */
    let mut syst = cp.SYST;
    syst.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
    syst.set_reload(192_000 - 1);
    syst.clear_current();
    syst.enable_counter();
    let mut sampler = ina::Sampler::new();

    /* 1 Hz energy integration (RAM only, starts at 0; persistence is M2):
     * every 250 ticks accumulate published I and P for one second.
     * mAh = sum(I_1e5 * 1s) / 360000, mWh = sum(P_1e4 * 1s) / 36000. */
    let mut sec_ticks: u32 = 0;
    let mut acc_i_1e5s: u64 = 0;
    let mut acc_p_1e4s: u64 = 0;

    /* screen refresh every 125 ticks (0.5 s); only changed cells are drawn
     * (ui.rs dirty tracking), so an idle refresh costs nothing */
    let mut ui_ticks: u32 = 0;

    let mut buf = [0u8; 64];
    loop {
        mk(MK_HEART, mk_get(MK_HEART) + 1);
        if syst.has_wrapped() {
            if let Some(rd) = sampler.tick() {
                unsafe {
                    let m = &mut *PUBLISHED.0.get();
                    m.v_1e4 = rd.v_1e4;
                    m.i_1e5 = rd.i_1e5;
                    m.dp_1e2 = rd.dp_1e2;
                    m.dm_1e2 = rd.dm_1e2;
                    m.temp_c = rd.temp_c;
                }
            }
            sec_ticks += 1;
            if sec_ticks >= 250 {
                sec_ticks = 0;
                unsafe {
                    let m = &mut *PUBLISHED.0.get();
                    let (p, _) = ina::derive(m.v_1e4, m.i_1e5);
                    acc_i_1e5s += m.i_1e5 as u64;
                    acc_p_1e4s += p as u64;
                    m.mah = (acc_i_1e5s / 360_000) as u32;
                    m.mwh = (acc_p_1e4s / 36_000) as u32;
                }
            }
            ui_ticks += 1;
            if ui_ticks >= 125 && tft_alive {
                ui_ticks = 0;
                let m = unsafe { *PUBLISHED.0.get() };
                let (pw, _) = ina::derive(m.v_1e4, m.i_1e5);
                /* a failed draw disables the refresh for this boot (a wedged
                 * SPI wait would otherwise slow every 0.5 s cycle) */
                tft_alive = ui::draw(m.v_1e4, m.i_1e5, pw).is_ok();
            }
        }
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
        let m = unsafe { *PUBLISHED.0.get() };
        let (p, r) = ina::derive(m.v_1e4, m.i_1e5);
        let fields = getva::Fields {
            v: m.v_1e4,
            i: m.i_1e5,
            p,
            r,
            mah: m.mah,
            mwh: m.mwh,
            temp_c: m.temp_c,
            dp: m.dp_1e2,
            dm: m.dm_1e2,
        };
        let mut reply = [0u8; 192];
        getva::build(&mut reply, &fields);
        write_all(dev, serial, &reply);
    } else if match5(buf, b"i2crd") {
        /* dev/bring-up verb (ours, not stock):
         *   i2crd <addr7> <wlen 0..=2> <w0> <w1> <rlen 0..=32>
         * -> "OK" + rlen bytes, or "ER" + [stage, stat0.lo, stat0.hi, stat1.lo] */
        let mut reply = [0u8; 34];
        let n = i2c_command(buf, &mut reply);
        write_all(dev, serial, &reply[..n]);
    } else if match5(buf, b"adcrd") {
        /* dev verb: adcrd <ch> -> "OK" + u32 LE sum-of-16, or "ER" + stage */
        let mut reply = [0u8; 6];
        let n = if buf.len() >= 6 {
            match adc::read_sum16(buf[5]) {
                Ok(sum) => {
                    reply[..2].copy_from_slice(b"OK");
                    reply[2..6].copy_from_slice(&sum.to_le_bytes());
                    6
                }
                Err(stage) => {
                    reply[..2].copy_from_slice(b"ER");
                    reply[2] = stage;
                    3
                }
            }
        } else {
            reply[..2].copy_from_slice(b"ER");
            reply[2] = 0x0F;
            3
        };
        write_all(dev, serial, &reply[..n]);
    } else if match5(buf, b"tftfl") {
        /* dev verb: tftfl -> test pattern; tftfl <lo> <hi> -> solid RGB565 fill
         * -> "OK", or "ER" + stage (1 SPI TBE timeout, 2 shift-idle timeout) */
        let r = if buf.len() >= 7 {
            let color = buf[5] as u16 | (buf[6] as u16) << 8;
            tft::fill_rect(0, 0, tft::WIDTH - 1, tft::HEIGHT - 1, color)
        } else {
            tft::test_pattern()
        };
        reply_ok_er(dev, serial, r);
    } else if match5(buf, b"tftui") {
        /* dev verb: draw/refresh the main screen from published values */
        let m = unsafe { *PUBLISHED.0.get() };
        let (p, _) = ina::derive(m.v_1e4, m.i_1e5);
        reply_ok_er(dev, serial, ui::draw(m.v_1e4, m.i_1e5, p));
    } else if match5(buf, b"tftbl") {
        /* dev verb: tftbl <lo> <hi> -> backlight PWM compare 0..=999 */
        let r = if buf.len() >= 7 {
            tft::backlight_set(buf[5] as u16 | (buf[6] as u16) << 8)
        } else {
            Err(0x0F)
        };
        reply_ok_er(dev, serial, r);
    } else if match5(buf, b"updat") {
        if let Some(ep) = find_bulk_ep() {
            flasher::launch(ep); /* does not return */
        }
        mk(MK_PROBE, 0xBAD_EB); /* no bulk endpoint found — refuse, host times out */
    }
    /* rotat / nextp / lastp / unknown -> silent */
}

/* Latest published measurement set from the timer-paced sampler + 1 Hz energy
 * integrator; zeros until first window. Single-core, IRQs masked -> Racy ok. */
struct Racy<T>(core::cell::UnsafeCell<T>);
unsafe impl<T> Sync for Racy<T> {}

#[derive(Clone, Copy)]
struct Meas {
    v_1e4: u32,
    i_1e5: u32,
    dp_1e2: u32,
    dm_1e2: u32,
    temp_c: i32,
    mah: u32,
    mwh: u32,
}
static PUBLISHED: Racy<Meas> = Racy(core::cell::UnsafeCell::new(Meas {
    v_1e4: 0,
    i_1e5: 0,
    dp_1e2: 0,
    dm_1e2: 0,
    temp_c: 0,
    mah: 0,
    mwh: 0,
}));

#[inline(never)]
fn reply_ok_er(dev: &mut Dev, serial: &mut Serial, r: Result<(), u8>) {
    match r {
        Ok(()) => write_all(dev, serial, b"OK"),
        Err(stage) => write_all(dev, serial, &[b'E', b'R', stage]),
    }
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
