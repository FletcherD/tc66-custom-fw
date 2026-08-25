//! TC66 custom firmware — M0 on the ecosystem USB stack.
//!
//! usb-device + usbd-serial + stm32-usbd (via the small UsbPeripheral impl in
//! usb_bus.rs) provide a composite device: interface 0 = POWER-Z KM003C
//! vendor emulation (powerz.rs, binds the mainline Linux `powerz` hwmon
//! driver), interfaces 1+2 = CDC-ACM with the bare-5-byte commands:
//!   query -> "firm", getva -> 192-B encrypted packet, rawva -> 46-B
//!   plaintext, updat -> Rust SRAM flasher (flasher.rs), dev verbs,
//!   rotat/nextp/lastp -> silent no-ops.
//!
//! Endpoint layout: EP1 = powerz bulk pair (fixed — the kernel driver
//! hard-codes 0x01/0x81); the CDC endpoints are whatever the stack allocates
//! next (expected: EP2 = interrupt IN, EP3 = bulk OUT+IN). `updat` discovers
//! the CDC bulk endpoint by scanning EPnR (skipping EP1) and the flasher
//! reads its PMA buffers from the BTABLE. Interrupts stay masked
//! (startup.rs); everything runs off dev.poll().

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
mod powerz;
mod startup;
mod tft;
mod ui;
mod usb_bus;

use cortex_m_rt::entry;
use gd32f1::gd32f150 as pac;
use markers::*;
use usb_device::device::{StringDescriptors, UsbDeviceBuilder, UsbRev, UsbVidPid};
use usb_device::prelude::*;
use usbd_serial::SerialPort;

/// Bumped to prove a self-hosted reflash actually took (read at MK_BUILD).
const BUILD_ID: u32 = 36;

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
    board::clock_init(&p.rcu);

    /* Force a fresh enumeration of THIS app: the bootloader handed off with D+
     * still pulled up and the host bound to the bootloader. Disconnect, bring
     * the stack up, reconnect. */
    board::pa9_config_output(&p.rcu, &p.gpioa);
    board::pa9_low(&p.gpioa);
    board::delay_ms(120);

    /* Composite device wearing the POWER-Z KM003C identity so the mainline
     * Linux `powerz` hwmon driver binds: the vendor class MUST be constructed
     * first — the driver hard-codes bulk EPs 0x01/0x81 on interface 00 — and
     * CDC-ACM (our protocol + updat reflash) follows on interfaces 1+2. */
    let usb_alloc = usb_bus::UsbBus::new(usb_bus::Gd32Usb);
    let mut pz = powerz::PowerZ::new(&usb_alloc);
    let mut serial = SerialPort::new(&usb_alloc);
    let mut dev = UsbDeviceBuilder::new(&usb_alloc, UsbVidPid(0x5FC9, 0x0063))
        .strings(&[StringDescriptors::default()
            .manufacturer("ChargerLAB")
            .product("POWER-Z KM003C")
            .serial_number("TC66")])
        .unwrap()
        .composite_with_iads()
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

    /* TIMER5 (basic timer, no pins) free-runs at 10 kHz as the sampler
     * timebase: 40 counts = one 4 ms tick, 250 ticks/s. Elapsed ticks accrue
     * from CNT deltas each loop pass, so time spent blocked (a getva reply is
     * ~21 ms of AES) is caught up, never lost — SysTick's COUNTFLAG latched a
     * single wrap and dropped the rest, stalling sampler + energy clock under
     * fast host polling. 16-bit @ 10 kHz wraps at 6.55 s, far beyond the
     * longest blocking path (write_all's absent-host budget, ~0.5 s). */
    p.rcu.apb1en().modify(|_, w| w.timer5en().enabled());
    p.timer5.psc().write(|w| w.psc().bits(4799)); /* 48 MHz -> 10 kHz */
    p.timer5.car().write(|w| w.car().bits(0xFFFF));
    p.timer5.swevg().write(|w| w.upg().set_bit()); /* latch PSC now */
    p.timer5.ctl0().modify(|_, w| w.cen().enabled());
    getva::init(); /* AES key schedule now, while the stack is shallow */
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
    let mut prev_cnt = p.timer5.cnt().read().cnt().bits();
    let mut tick_acc: u32 = 0;
    loop {
        mk(MK_HEART, mk_get(MK_HEART) + 1);
        let cnt = p.timer5.cnt().read().cnt().bits();
        tick_acc += cnt.wrapping_sub(prev_cnt) as u32;
        prev_cnt = cnt;
        while tick_acc >= 40 {
            tick_acc -= 40;
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
            if let Some((iv, ii)) = sampler.instant() {
                unsafe {
                    let m = &mut *PUBLISHED.0.get();
                    m.inst_v_1e4 = iv;
                    m.inst_i_1e5 = ii;
                }
            }
            /* refresh the pre-built powerz reply each tick (calm context) */
            pz.set_payload(&powerz_payload());
            sec_ticks += 1;
            if sec_ticks >= 250 {
                sec_ticks = 0;
                if let Some(avg) = sampler.drain_second() {
                    unsafe {
                        let m = &mut *PUBLISHED.0.get();
                        m.avg_v_1e4 = avg.v_1e4;
                        m.avg_i_1e5 = avg.i_1e5;
                        m.ori_uv = avg.ori_uv;
                        m.ori_ua = avg.ori_ua;
                    }
                }
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
                /* re-arm before the draw: it's the longest tick-body path,
                 * and an unarmed IN during it risks the powerz 5 ms deadline */
                pz.arm();
                let m = unsafe { *PUBLISHED.0.get() };
                let (pw, _) = ina::derive(m.v_1e4, m.i_1e5);
                /* a failed draw disables the refresh for this boot (a wedged
                 * SPI wait would otherwise slow every 0.5 s cycle) */
                tft_alive = ui::draw(m.v_1e4, m.i_1e5, pw).is_ok();
            }
        }
        /* every pass, not only on USB events: a reply consumed while the
         * loop was busy in tick work must be re-armed at the first
         * opportunity, before the host's next request starts NAK-polling */
        pz.arm();
        if !dev.poll(&mut [&mut pz, &mut serial]) {
            continue;
        }
        if dev.state() == UsbDeviceState::Configured && mk_get(MK_STAGE) < STAGE_CONFIG {
            mk(MK_STAGE, STAGE_CONFIG);
        }
        pz.poll_cmd();
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
        let mut reply = [0u8; 192];
        getva::build(&mut reply, &live_fields());
        write_all(dev, serial, &reply);
    } else if match5(buf, b"rawva") {
        /* dev verb: same fields as getva, plaintext + CRC16, no AES — for
         * fast host streaming (layout in getva.rs; tools/rawva.py) */
        let mut reply = [0u8; getva::RAW_LEN];
        getva::build_raw(&mut reply, &live_fields());
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

/// Snapshot the published measurements + derived values + window seq for a
/// getva/rawva reply.
#[inline(never)]
fn live_fields() -> getva::Fields {
    let m = unsafe { *PUBLISHED.0.get() };
    let (p, r) = ina::derive(m.v_1e4, m.i_1e5);
    getva::Fields {
        v: m.v_1e4,
        i: m.i_1e5,
        p,
        r,
        mah: m.mah,
        mwh: m.mwh,
        temp_c: m.temp_c,
        dp: m.dp_1e2,
        dm: m.dm_1e2,
        seq: mk_get(MK_WINDOWS),
    }
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
    /* powerz fields: latest single conversion, 1 s calibrated average,
     * 1 s nominal-scale ("original") average */
    inst_v_1e4: u32,
    inst_i_1e5: i32,
    avg_v_1e4: u32,
    avg_i_1e5: i32,
    ori_uv: u32,
    ori_ua: i32,
}
static PUBLISHED: Racy<Meas> = Racy(core::cell::UnsafeCell::new(Meas {
    v_1e4: 0,
    i_1e5: 0,
    dp_1e2: 0,
    dm_1e2: 0,
    temp_c: 0,
    mah: 0,
    mwh: 0,
    inst_v_1e4: 0,
    inst_i_1e5: 0,
    avg_v_1e4: 0,
    avg_i_1e5: 0,
    ori_uv: 0,
    ori_ua: 0,
}));

/// Build the 44-byte powerz ADC payload (layout: powerz.rs / HARDWARE.md).
/// All uV/uA are our wire units scaled up (1e-4 V -> x100 uV, 1e-5 A ->
/// x10 uA); Vcc1/Vcc2/Vdd are 0 until the FUSB302 is brought up (M4); temp
/// is clamped at 0 (the kernel driver's byte pair can't encode sub-zero).
#[inline(never)]
fn powerz_payload() -> [u8; powerz::ADC_PAYLOAD_LEN] {
    let m = unsafe { *PUBLISHED.0.get() };
    let mut p = [0u8; powerz::ADC_PAYLOAD_LEN];
    let mut put = |off: usize, v: i32| p[off..off + 4].copy_from_slice(&v.to_le_bytes());
    put(0, m.inst_v_1e4 as i32 * 100);
    put(4, m.inst_i_1e5 * 10);
    put(8, m.avg_v_1e4 as i32 * 100);
    put(12, m.avg_i_1e5 * 10);
    put(16, m.ori_uv as i32);
    put(20, m.ori_ua);
    let t = if m.temp_c < 0 { 0 } else { m.temp_c as u32 };
    p[24] = ((t % 2) * 128) as u8; /* 1/128 degC units */
    p[25] = (t / 2) as u8; /* 2 degC units */
    let mut put16 = |off: usize, v: u32| {
        let c = if v > 0xFFFF { 0xFFFF } else { v as u16 };
        p[off..off + 2].copy_from_slice(&c.to_le_bytes());
    };
    put16(30, m.dp_1e2 * 100); /* 1e-2 V -> 0.1 mV units */
    put16(32, m.dm_1e2 * 100);
    p
}

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
/// (RX disabled), EP0 is CONTROL, the notification endpoint is INTERRUPT.
/// EP1 is skipped: it is the powerz vendor bulk pair (fixed by that protocol),
/// not the CDC data endpoint updat streams through. Read-only peek at
/// registers the USB stack owns.
#[inline(never)]
fn find_bulk_ep() -> Option<u32> {
    let usbd = unsafe { &*pac::Usbd::ptr() };
    for n in 2..8u32 {
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
