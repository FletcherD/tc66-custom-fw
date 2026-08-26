//! TC66 custom firmware — M0 on the ecosystem USB stack.
//!
//! usb-device + usbd-serial + stm32-usbd (via the small UsbPeripheral impl
//! in usb_bus.rs) provide a composite device. Interface 0 = POWER-Z KM003C
//! vendor emulation (powerz.rs, binds the mainline Linux `powerz` hwmon
//! driver) — **the PRIORITY protocol**: its pre-armed reply must always
//! meet the driver's 5 ms URB budget. Interfaces 1+2 = CDC-ACM, kept for
//! COMPATIBILITY with stock TC66 host tools + our dev channel, with the
//! bare-5-byte commands:
//!   query -> "firm", getva -> 192-B encrypted packet, rawva -> 46-B
//!   plaintext, updat -> Rust SRAM flasher (flasher.rs), dev verbs,
//!   rotat/nextp/lastp -> silent no-ops.
//!
//! Endpoint layout: EP1 = powerz bulk pair (fixed — the kernel driver
//! hard-codes 0x01/0x81); the CDC endpoints are whatever the stack allocates
//! next (expected: EP2 = interrupt IN, EP3 = bulk OUT+IN). `updat` discovers
//! the CDC bulk endpoint by scanning EPnR (skipping EP1) and the flasher
//! reads its PMA buffers from the BTABLE.
//!
//! USB is serviced from the USBD_LP ISR (IRQ 37): dev.poll, the powerz
//! command/arm path, and moving CDC packets into CMD_RING all run there, so
//! blocking main-loop work (TFT draws, getva's ~20 ms AES) can never starve
//! the bus. The main loop consumes CMD_RING, builds replies in thread mode,
//! and pushes them into usbd-serial under short critical sections
//! (with_usb). The sampler runs from the TIMER5 interrupt (250 Hz, one
//! interleaved INA226 read per tick — ina.rs), immune to main-loop blocking.
//! Exactly TWO interrupts are ever unmasked (USBD_LP + TIMER5_DAC), both at
//! one NVIC priority so they never nest (the stack-gate charges the max
//! handler chain, not the sum). USBD_LP is enabled late in run(), after
//! every static its ISR touches is initialized; the sampler tick starts
//! only after the grace window. Shared-state soundness arguments live next
//! to each item (UsbPtrs, CmdRing, powerz::REPLY, the ina.rs module doc).
//!
//! The first ~10 s after the unmask are the GRACE WINDOW (pseudo-bootloader
//! escape hatch): the thread dispatches ONLY `query`/`updat` and no
//! measurement feature (sampler/I2C/ADC) has started — a fault in anything
//! later is recovered by power-cycle + `make flash` inside the window, no
//! K1. (The display's DMA first frame runs even earlier, pre-enumeration —
//! bounded-error throughout; K1 is the backstop for a hard fault there.)

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
#[cfg(feature = "dev-verbs")]
mod pd;
mod powerz;
#[cfg(feature = "dev-verbs")]
mod qc;
mod startup;
mod tft;
mod ui;
mod usb_bus;

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::entry;
use gd32f1::gd32f150 as pac;
use markers::*;
use pac::interrupt;
use usb_device::device::{StringDescriptors, UsbDeviceBuilder, UsbRev, UsbVidPid};
use usb_device::prelude::*;
use usbd_serial::SerialPort;

/// Bumped to prove a self-hosted reflash actually took (read at MK_BUILD).
const BUILD_ID: u32 = 80;

type Dev<'a> = UsbDevice<'a, usb_bus::UsbBus>;
/// 64-B read store (one full USB packet — commands are <=10 B) and 96-B
/// write store (1.5 packets; write_all refills as the ISR drains) instead
/// of usbd-serial's 128-B defaults: the buffers live in run()'s frame, and
/// the stack-gate budget is tight (DMA display statics + the ISR drain
/// loop ate the slack).
type Serial<'a> = SerialPort<'a, usb_bus::UsbBus, [u8; 64], [u8; 96]>;
type Pz<'a> = powerz::PowerZ<'a, usb_bus::UsbBus>;

#[entry]
fn main() -> ! {
    /* Deep one-shot init under THIS shallow frame, not run()'s ~2 KB of
     * locals: the stack budget must also carry the sampler ISR chain on top
     * of the deepest thread path (stack-gate). Pure compute — safe before
     * clock init. */
    getva::init();
    run()
}

#[inline(never)]
fn run() -> ! {
    mk(MK_BOOT, 0xB007_B007);
    mk(MK_BUILD, BUILD_ID);
    mk(MK_HEART, 0);
    mk(MK_CMDCNT, 0);
    mk(MK_ENUM, 0);
    mk(MK_I2CERR, 0);
    mk(MK_WINDOWS, 0);
    for i in 0..6 {
        mk(MK_DSP_BASE + 4 * i, 0); /* display/DMA diagnostic block */
    }

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
    let mut serial = SerialPort::new_with_store(&usb_alloc, [0u8; 64], [0u8; 96]);
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

    /* TFT init + first frame now, while D+ is still low — the init
     * sequence has ~400 ms of fixed delays that must not stall
     * enumeration, and the screen is live from power-on instead of after
     * the grace window. flush() pumps the DMA engine to completion
     * (bounded by its per-phase poll budgets); on error the display stays
     * off for this boot (verbs can still retry it). Every wait in this
     * path is bounded and error-returning — a fault here would land
     * before the escape hatch opens, and K1 remains the backstop. */
    ui::draw(0, 0, 0);
    let mut tft_alive = ui::flush().is_ok();

    board::pa9_high(&p.gpioa);

    /* SysTick free-runs over its full 24 bits as the grace-window timebase
     * (CORE-internal — reads never stall during host NAK storms, unlike
     * APB peripherals). 24 bits @ 48 MHz wraps at 349 ms, far beyond any
     * loop pass. The steady-state loop paces itself off the TIMER5 ISR's
     * tick counter instead (ina::ticks). */
    let mut syst = cortex_m::Peripherals::take().unwrap().SYST;
    syst.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
    syst.set_reload(0x00FF_FFFF);
    syst.clear_current();
    syst.enable_counter();

    /* Hand the USB objects to the ISR and open the gate. main() never
     * returns, so the locals live forever and the raw pointers stay valid.
     * Publish them BEFORE unmasking; from here on main touches dev/pz/serial
     * only through with_usb() (critical sections) — the locals are shadowed
     * below so direct access is a compile error. PRIMASK has been set since
     * __pre_init; clearing it last releases any events that accumulated in
     * ISTR since pa9_high (the mask bits in CNTR — CTRM/RESETM/SUSPM/WKUPM —
     * were set by stm32-usbd's enable()). */
    unsafe {
        *USB.0.get() = UsbPtrs {
            dev: &mut dev as *mut Dev as *mut Dev<'static>,
            pz: &mut pz as *mut Pz as *mut Pz<'static>,
            serial: &mut serial as *mut Serial as *mut Serial<'static>,
        };
        mk(MK_USB_STATE, 0);
        mk(MK_USB_IRQS, 0);
        mk(MK_USB_ISTR, 0);
        mk(MK_USB_FULL, 0);
        mk(MK_USB_CMDS, 0);
        mk(MK_USB_SPUR, 0);
        mk(MK_USB_REDRAIN, 0);
        mk(MK_USB_RDMAX, 0);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USBD_LP);
        cortex_m::interrupt::enable();
    }
    #[allow(unused_variables)]
    let (dev, pz, serial) = ((), (), ());

    /* ---- pseudo-bootloader grace window (dev escape hatch) ----
     * 10 s dispatching ONLY `query`->firm and `updat` before the
     * measurement features start (no sampler/I2C/ADC; the display's first
     * frame already ran pre-enumeration). USB runs on the same ISR + ring
     * machinery as normal operation (single code path); the thread does
     * nothing here but drain the ring, so the ring can never stay full. A
     * fault in anything that starts later is escaped by power-cycling and
     * running `make flash` inside this window — no K1. MK_PROBE reads
     * 0x6ACE6ACE while the window is open. Standalone (no host) boots just
     * wait out the 10 s. */
    mk(MK_PROBE, 0x6ACE_6ACE);
    {
        let mut acc: u32 = 0;
        let mut prev = cortex_m::peripheral::SYST::get_current();
        while acc < 480_000_000 {
            /* 10 s of core cycles */
            mk(MK_HEART, mk_get(MK_HEART) + 1);
            let c = cortex_m::peripheral::SYST::get_current();
            acc += prev.wrapping_sub(c) & 0x00FF_FFFF; /* down-counter */
            prev = c;
            /* sweep every pass: during grace nothing else runs, so the
             * escape hatch gets polled-mode robustness on top of the ISR
             * (the clone's IRQ line loses events — see the tick sweep) */
            with_usb(|dev, pz, serial| {
                dev.poll(&mut [&mut *pz, &mut *serial]);
            });
            run_pending_cmds(true);
            with_usb(|_, _, serial| drain_serial(serial));
        }
    }
    mk(MK_PROBE, 0);

    /* Sampler: reprogram TIMER5 as the 250 Hz tick ISR and unmask its NVIC
     * line (ina.rs) — deliberately only now, after the grace window, so a
     * sampler/I2C fault stays escapable by power-cycle + `make flash`. */
    ina::start(&p.rcu, &p.timer5);

    /* 1 Hz energy integration (RAM only, starts at 0; persistence is M2):
     * every 250 ticks accumulate published I and P for one second.
     * mAh = sum(I_1e5 * 1s) / 360000, mWh = sum(P_1e4 * 1s) / 36000. */
    let mut sec_ticks: u32 = 0;
    let mut acc_i_1e5s: u64 = 0;
    let mut acc_p_1e4s: u64 = 0;

    /* screen refresh every 125 ticks (0.5 s); only changed cells are drawn
     * (ui.rs dirty tracking), so an idle refresh costs nothing */
    let mut ui_ticks: u32 = 0;

    /* Screen idle timeout: after 15 s with no measurable input the panel
     * is blanked (DISPOFF + backlight off) — an overnight static screen
     * caused real LCD image persistence. Activity = bus voltage above
     * 2.5 V or current above 1 mA; wake within a second of either. */
    const IDLE_OFF_SECS: u32 = 15;
    let mut idle_secs: u32 = 0;

    let mut last_ticks = ina::ticks();
    loop {
        mk(MK_HEART, mk_get(MK_HEART) + 1);
        /* cal retries + the deferred window-end ADC burst (temp/D±) */
        ina::service();
        /* FUSB302 PD sink engine — no-op until armed by `pdctl 1` */
        #[cfg(feature = "dev-verbs")]
        pd::service();
        let now = ina::ticks();
        let elapsed = now.wrapping_sub(last_ticks);
        last_ticks = now;
        if elapsed > 0 {
            /* refresh the pre-built powerz reply at tick cadence (own frame:
             * its 44-B buffer must not live in run()'s — stack-gate) */
            refresh_powerz_payload();
            sec_ticks += elapsed;
            while sec_ticks >= 250 {
                sec_ticks -= 250;
                /* v/i only, not a whole-Meas snapshot: those cost ~56 B of
                 * main-frame each at opt-1 (stack-gate budget) */
                let (v, i) = ina::snapshot_vi();
                let (pw, _) = ina::derive(v, i);
                acc_i_1e5s += i as u64;
                acc_p_1e4s += pw as u64;
                ina::set_energy(
                    (acc_i_1e5s / 360_000) as u32,
                    (acc_p_1e4s / 36_000) as u32,
                );
                if tft_alive {
                    if v > 25_000 || i > 100 {
                        idle_secs = 0;
                        let _ = tft::screen_on(); /* no-op when already on */
                    } else if idle_secs < IDLE_OFF_SECS {
                        idle_secs += 1;
                        if idle_secs == IDLE_OFF_SECS {
                            let _ = tft::screen_off();
                        }
                    }
                }
            }
            ui_ticks += elapsed;
            if ui_ticks >= 125 && tft_alive {
                ui_ticks = 0;
                /* format-only: marks changed cells dirty; the per-pass
                 * ui::service() below streams the glyphs out via DMA */
                let (v, i) = ina::snapshot_vi();
                let (pw, _) = ina::derive(v, i);
                ui::draw(v, i, pw);
            }
        }
        /* advance the display DMA engine one poll (cheap when idle; at most
         * one glyph-row kick per pass — never blocks the loop). A failed job
         * disables the always-on refresh for this boot; verbs can retry. */
        if tft_alive && ui::service().is_err() {
            tft_alive = false;
        }
        /* NO thread-mode dev.poll sweep here — or anywhere in steady
         * state: the ISR drain loop is the ONE hardware consumer (the
         * structural property that spares stock from QUIRKS §3; the old
         * slow-flash A/B measured second-consumer sweeps making latency
         * 3x worse). The single exception is the grace window, where
         * escape-hatch robustness outranks everything. */
        /* dispatch commands the ISR queued; replies (AES etc.) are built
         * right here in thread mode with interrupts enabled */
        run_pending_cmds(false);
        /* recovery drain: packets that arrived while the ring was full sit
         * in usbd-serial's buffer with no further IRQ coming for them */
        with_usb(|_, _, serial| drain_serial(serial));
        if mk_get(MK_STAGE) < STAGE_CONFIG
            && with_usb(|dev, _, _| dev.state()) == Some(UsbDeviceState::Configured)
        {
            mk(MK_STAGE, STAGE_CONFIG);
        }
    }
}

#[inline(never)]
fn match5(b: &[u8], cmd: &[u8; 5]) -> bool {
    b.len() >= 5 && b[..5] == cmd[..]
}

#[inline(never)]
fn on_packet(buf: &[u8]) {
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
        write_all(b"firm");
        mk(MK_STAGE, STAGE_QUERY);
    } else if match5(buf, b"getva") {
        do_getva();
    } else if match5(buf, b"rawva") {
        do_rawva();
    } else if match5(buf, b"pcprf") {
        #[cfg(feature = "pcprf")]
        do_pcprf();
    } else if match5(buf, b"i2crd") {
        #[cfg(feature = "dev-verbs")]
        do_i2crd(buf);
    } else if match5(buf, b"adcrd") {
        #[cfg(feature = "dev-verbs")]
        do_adcrd(buf);
    } else if match5(buf, b"clkrd") {
        #[cfg(feature = "dev-verbs")]
        do_clkrd();
    } else if match5(buf, b"pdctl") {
        #[cfg(feature = "dev-verbs")]
        {
            /* pdctl 0 = FUSB302 off (SW_RES), pdctl 1 = arm as PD sink,
             * pdctl 2 = send PD hard reset (source restarts caps at its
             * highest rev; VBUS dips) */
            let r = match buf.get(5) {
                Some(0) => pd::off(),
                Some(1) => pd::arm(),
                Some(2) => pd::hard_reset(),
                _ => Err(0x0F),
            };
            reply_ok_er(r);
        }
    } else if match5(buf, b"pdsta") {
        #[cfg(feature = "dev-verbs")]
        do_pdsta();
    } else if match5(buf, b"pdcap") {
        #[cfg(feature = "dev-verbs")]
        do_pdcap();
    } else if match5(buf, b"pdreq") {
        #[cfg(feature = "dev-verbs")]
        {
            /* pdreq <n> [mv u16 LE] [ma u16 LE]: request source PDO n
             * (1-based); mv/ma only for a PPS APDO */
            let r = match buf.get(5) {
                Some(&n) => {
                    let u16at = |i: usize| -> u16 {
                        if buf.len() >= i + 2 {
                            buf[i] as u16 | (buf[i + 1] as u16) << 8
                        } else {
                            0
                        }
                    };
                    pd::request(n, u16at(6), u16at(8))
                }
                None => Err(0x0F),
            };
            reply_ok_er(r);
        }
    } else if match5(buf, b"qctrg") {
        #[cfg(feature = "dev-verbs")]
        {
            /* qctrg <mode>: QC2.0 D+/D- trigger, mode 0/1/2/3 = 5/9/12/20 V
             * (blocks ~1.5 s on the first call — QC entry). Raises OUT V. */
            let r = match buf.get(5) {
                Some(&m) => qc::trigger(m),
                None => Err(0x0F),
            };
            reply_ok_er(r);
        }
    } else if match5(buf, b"qc3ct") {
        #[cfg(feature = "dev-verbs")]
        {
            /* qc3ct <dir> <count>: QC3.0 continuous, dir 0=up 1=down,
             * count 200 mV steps. First call runs entry (~1.5 s). */
            let r = match (buf.get(5), buf.get(6)) {
                (Some(&d), Some(&n)) => qc::qc3(d != 0, n),
                _ => Err(0x0F),
            };
            reply_ok_er(r);
        }
    } else if match5(buf, b"qcoff") {
        #[cfg(feature = "dev-verbs")]
        reply_ok_er(qc::off());
    } else if match5(buf, b"tftfl") {
        #[cfg(feature = "dev-verbs")]
        {
        /* dev verb: tftfl -> test pattern; tftfl <lo> <hi> -> solid RGB565 fill
         * -> "OK", or "ER" + stage (tft.rs: 1 SPI TBE timeout, 2 shift-idle
         * timeout, 3 DMA timeout, 4 DMA error, 5 busy, 6 pump budget) */
        let _ = tft::screen_on(); /* draw verbs wake an idle-off screen */
        let r = if buf.len() >= 7 {
            let color = buf[5] as u16 | (buf[6] as u16) << 8;
            tft::fill_rect(0, 0, tft::WIDTH - 1, tft::HEIGHT - 1, color)
        } else {
            tft::test_pattern()
        };
        reply_ok_er(r);
        }
    } else if match5(buf, b"tftui") {
        #[cfg(feature = "dev-verbs")]
        {
        /* dev verb: draw/refresh the main screen from published values
         * (kick + pump to completion — dev verbs may block) */
        let _ = tft::screen_on();
        let m = ina::snapshot();
        let (p, _) = ina::derive(m.v_1e4, m.i_1e5);
        ui::draw(m.v_1e4, m.i_1e5, p);
        reply_ok_er(ui::flush());
        }
    } else if match5(buf, b"tftbl") {
        #[cfg(feature = "dev-verbs")]
        {
        /* dev verb: tftbl <lo> <hi> -> backlight PWM compare 0..=999 */
        let _ = tft::screen_on();
        let r = if buf.len() >= 7 {
            tft::backlight_set(buf[5] as u16 | (buf[6] as u16) << 8)
        } else {
            Err(0x0F)
        };
        reply_ok_er(r);
        }
    } else if match5(buf, b"updat") {
        /* The flasher erases app flash and takes over the CDC endpoint by
         * hand — the USBD ISR must never run again: mask globally AND in the
         * NVIC before touching anything, so nothing can preempt between
         * endpoint discovery and the SRAM handoff. */
        cortex_m::interrupt::disable();
        cortex_m::peripheral::NVIC::mask(pac::Interrupt::USBD_LP);
        if let Some(ep) = find_bulk_ep() {
            flasher::launch(ep); /* does not return */
        }
        mk(MK_PROBE, 0xBAD_EB); /* no bulk endpoint found — refuse, host times out */
        unsafe {
            cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USBD_LP);
            cortex_m::interrupt::enable();
        }
    }
    /* rotat / nextp / lastp / unknown -> silent */
}

/* Each buffer-carrying verb handler gets its own frame: the stack gate sums
 * every branch's locals when they share on_packet's frame (conservative
 * multi-path accounting), so splitting turns that sum into a max. */

#[inline(never)]
fn do_getva() {
    let mut reply = [0u8; 192];
    getva::build(&mut reply, &live_fields());
    write_all(&reply);
}

/// dev verb: dump the TIMER5-veneer PC-profiler ring (markers.rs
/// MK_PCPRF_BASE): u32 LE total-sample counter + 128 u32 LE PCs. The ISR
/// keeps writing during the copy — the host drops the newest few entries
/// per poll to stay consistent (tools/pcprofile.py).
#[cfg(feature = "pcprf")]
#[inline(never)]
fn do_pcprf() {
    /* 129 words (counter + ring) in three 43-word chunks: one 516-B stack
     * buffer here put the deepest thread chain over the stack-gate budget */
    let mut chunk = [0u8; 4 * 43];
    for c in 0..3 {
        for i in 0..43 {
            let v = mk_get(MK_PCPRF_BASE + 4 * (43 * c + i));
            chunk[4 * i..4 * i + 4].copy_from_slice(&v.to_le_bytes());
        }
        write_all(&chunk);
    }
}

/// dev verb: same fields as getva, plaintext + CRC16, no AES — for fast
/// host streaming (layout in getva.rs; tools/rawva.py)
#[inline(never)]
fn do_rawva() {
    let mut reply = [0u8; getva::RAW_LEN];
    getva::build_raw(&mut reply, &live_fields());
    write_all(&reply);
}

/// dev/bring-up verb (ours, not stock):
///   i2crd <addr7> <wlen 0..=2> <w0> <w1> <rlen 0..=32>
/// -> "OK" + rlen bytes, or "ER" + [stage, stat0.lo, stat0.hi, stat1.lo]
#[cfg(feature = "dev-verbs")]
#[inline(never)]
fn do_i2crd(buf: &[u8]) {
    let mut reply = [0u8; 34];
    let n = i2c_command(buf, &mut reply);
    write_all(&reply[..n]);
}

/// dev verb: adcrd <ch> -> "OK" + u32 LE sum-of-16, or "ER" + stage
#[cfg(feature = "dev-verbs")]
#[inline(never)]
fn do_adcrd(buf: &[u8]) {
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
    write_all(&reply[..n]);
}

/// dev verb: clkrd -> "OK" + RCU_CTL + RCU_CFG0 (u32 LE each). One-off for
/// QUIRKS §1's loose end: the bootloader hands off the clock tree and we
/// never touch the AHB/APB prescalers (CFG0[10:4]) — the UM requires APB1
/// > 24 MHz for the USBD. Read-only register peek.
#[cfg(feature = "dev-verbs")]
#[inline(never)]
fn do_clkrd() {
    let mut reply = [0u8; 10];
    reply[..2].copy_from_slice(b"OK");
    let ctl = unsafe { core::ptr::read_volatile(0x4002_1000 as *const u32) };
    let cfg0 = unsafe { core::ptr::read_volatile(0x4002_1004 as *const u32) };
    reply[2..6].copy_from_slice(&ctl.to_le_bytes());
    reply[6..10].copy_from_slice(&cfg0.to_le_bytes());
    write_all(&reply);
}

/// dev verb: pdsta -> "OK" + 20-byte PD engine state (layout: pd::status)
#[cfg(feature = "dev-verbs")]
#[inline(never)]
fn do_pdsta() {
    let mut reply = [0u8; 22];
    reply[..2].copy_from_slice(b"OK");
    let mut st = [0u8; 20];
    pd::status(&mut st);
    reply[2..22].copy_from_slice(&st);
    write_all(&reply);
}

/// dev verb: pdcap -> "OK" + count + count x 4-byte LE source PDOs
#[cfg(feature = "dev-verbs")]
#[inline(never)]
fn do_pdcap() {
    let mut reply = [0u8; 31];
    let mut pdos = [0u8; 28];
    let n = pd::caps(&mut pdos);
    reply[..2].copy_from_slice(b"OK");
    reply[2] = n;
    reply[3..3 + 4 * n as usize].copy_from_slice(&pdos[..4 * n as usize]);
    write_all(&reply[..3 + 4 * n as usize]);
}

/// Snapshot the published measurements + derived values + window seq for a
/// getva/rawva reply.
#[inline(never)]
fn live_fields() -> getva::Fields {
    let m = ina::snapshot();
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

struct Racy<T>(core::cell::UnsafeCell<T>);
unsafe impl<T> Sync for Racy<T> {}

/* ---- USB object sharing (thread mode <-> USBD_LP ISR) ----
 *
 * The UsbDevice/PowerZ/SerialPort trio is ONE ownership unit living in
 * main()'s frame (main never returns). Raw pointers to them are published
 * here once, before the USBD_LP IRQ is unmasked. Access rules:
 *  - the ISR derives &muts directly (it cannot preempt itself: NVIC never
 *    nests same-priority IRQs, and the only other unmasked IRQ — the TIMER5
 *    sampler tick, same priority — never touches the USB objects);
 *  - thread mode derives &muts only inside with_usb (PRIMASK set), so the
 *    ISR cannot run while they exist;
 *  - after publishing, main's locals are shadowed — the pointers are the
 *    only remaining access path.
 * Together: no two &muts to these objects can ever coexist. */
struct UsbPtrs {
    dev: *mut Dev<'static>,
    pz: *mut Pz<'static>,
    serial: *mut Serial<'static>,
}
static USB: Racy<UsbPtrs> = Racy(core::cell::UnsafeCell::new(UsbPtrs {
    dev: core::ptr::null_mut(),
    pz: core::ptr::null_mut(),
    serial: core::ptr::null_mut(),
}));

/// Run `f` on the USB objects with the USBD_LP ISR held off (PRIMASK).
/// KEEP THE CLOSURES SHORT — a buffer copy or one endpoint write, never AES
/// or TFT work: powerz replies must be re-armable within the Linux driver's
/// 5 ms URB budget. Never nest with_usb calls (double &mut). None before
/// the pointers are published (nothing calls it that early).
#[inline(never)]
fn with_usb<R>(f: impl FnOnce(&mut Dev, &mut Pz, &mut Serial) -> R) -> Option<R> {
    cortex_m::interrupt::free(|_| {
        let u = unsafe { &*USB.0.get() };
        if u.dev.is_null() {
            return None;
        }
        Some(unsafe { f(&mut *u.dev, &mut *u.pz, &mut *u.serial) })
    })
}

/* ---- Command ring (ISR/producer -> main loop/consumer) ----
 *
 * Fixed slots so a CDC packet is handed over whole. Soundness:
 *  - push (drain_serial) runs in the ISR or in thread mode under
 *    interrupt::free — mutually exclusive contexts, so there is exactly one
 *    producer at a time and `head` updates never race;
 *  - pop (run_pending_cmds) runs only in thread-mode main loop — single
 *    consumer for `tail`;
 *  - head/tail are single u32 atomics (naturally atomic on M3; the
 *    Acquire/Release pairs stop compiler reordering vs the slot data);
 *  - a slot is written only while head-tail < SLOTS and read only while
 *    tail < head, so producer and consumer never touch the same slot.
 * When the ring is full the producer simply stops reading usbd-serial: bytes
 * back up in its 128-B buffer and then NAK the endpoint — backpressure, not
 * loss. Main re-runs drain_serial after freeing a slot (no IRQ will fire for
 * already-buffered bytes). */
const CMD_SLOTS: usize = 2;
/* 16 B per slot: commands use at most 10 bytes (i2crd); a host write longer
 * than a slot arrives as fragments, and non-verb fragments are silently
 * dropped by dispatch — our protocol never sends any. Small slots keep bss
 * inside the stack-gate budget. */
const CMD_SLOT_LEN: usize = 16;
struct CmdRing {
    bufs: core::cell::UnsafeCell<[[u8; CMD_SLOT_LEN]; CMD_SLOTS]>,
    lens: core::cell::UnsafeCell<[u8; CMD_SLOTS]>,
    head: AtomicU32,
    tail: AtomicU32,
}
unsafe impl Sync for CmdRing {}
static CMD_RING: CmdRing = CmdRing {
    bufs: core::cell::UnsafeCell::new([[0; CMD_SLOT_LEN]; CMD_SLOTS]),
    lens: core::cell::UnsafeCell::new([0; CMD_SLOTS]),
    head: AtomicU32::new(0),
    tail: AtomicU32::new(0),
};

/// Set when the ISR masked itself because CMD_RING was full; the thread
/// unmasks after freeing a slot. See drain_serial for why masking (not
/// just returning) is mandatory on this level-triggered hardware.
static IRQ_PARKED: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

/// Move buffered CDC packets into CMD_RING (reads straight into the slot —
/// no scratch buffer on the ISR stack). Returns true if it stopped because
/// the ring is full. ISR context, or thread mode under interrupt::free.
///
/// A full ring MUST NOT simply be left for later in ISR context: the unread
/// CDC packet keeps the endpoint's CTR flag asserted, the level-triggered
/// IRQ refires forever, and thread mode never runs again to drain the ring
/// — a livelock (bench-hit: 2.3M IRQs, MK_USB_FULL == MK_USB_IRQS, thread
/// frozen). The ISR parks itself (NVIC mask) instead; the thread unmasks
/// after popping a slot and the still-pending CTR refires the ISR once.
#[inline(never)]
fn drain_serial(serial: &mut Serial) -> bool {
    loop {
        let h = CMD_RING.head.load(Ordering::Relaxed);
        let t = CMD_RING.tail.load(Ordering::Acquire);
        if h.wrapping_sub(t) >= CMD_SLOTS as u32 {
            mk(MK_USB_FULL, mk_get(MK_USB_FULL) + 1);
            return true;
        }
        let i = h as usize % CMD_SLOTS;
        let slot = unsafe { &mut (*CMD_RING.bufs.get())[i] };
        match serial.read(slot) {
            Ok(n) if n > 0 => {
                unsafe { (*CMD_RING.lens.get())[i] = n as u8 };
                CMD_RING.head.store(h.wrapping_add(1), Ordering::Release);
                mk(MK_USB_CMDS, mk_get(MK_USB_CMDS) + 1);
            }
            _ => return false,
        }
    }
}

/// Dispatch every queued command packet. Thread mode only. The slot is
/// copied out and `tail` advanced BEFORE dispatching, so the ring has free
/// space (and a parked ISR is unmasked) while a slow reply (getva AES) is
/// still being built — powerz service continues throughout. With
/// `grace_only`, everything except the escape-hatch verbs (`query`,
/// `updat`) is dropped — the grace-window restriction.
#[inline(never)]
fn run_pending_cmds(grace_only: bool) {
    loop {
        let t = CMD_RING.tail.load(Ordering::Relaxed);
        let h = CMD_RING.head.load(Ordering::Acquire);
        if h == t {
            return;
        }
        let i = t as usize % CMD_SLOTS;
        let len = unsafe { (*CMD_RING.lens.get())[i] } as usize;
        /* commands use at most 10 bytes (i2crd); the copy lets `tail`
         * advance (and the ISR unpark) before slow dispatch, and 16 B
         * keeps main's frame under the stack-gate budget */
        let mut pkt = [0u8; 16];
        let len = len.min(pkt.len());
        let slot: &[u8; CMD_SLOT_LEN] = unsafe { &(*CMD_RING.bufs.get())[i] };
        pkt[..len].copy_from_slice(&slot[..len]);
        CMD_RING.tail.store(t.wrapping_add(1), Ordering::Release);
        if IRQ_PARKED.swap(false, Ordering::AcqRel) {
            /* ring has space again — let the still-pending CTR refire */
            unsafe { cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USBD_LP) };
        }
        if grace_only && !match5(&pkt[..len], b"query") && !match5(&pkt[..len], b"updat") {
            continue;
        }
        on_packet(&pkt[..len]);
    }
}

/// USBD_LP (IRQ 37). Keeps the bus serviced no
/// matter what the main loop is doing. Everything here is bounded and short
/// (stm32-usbd/usbd-serial never busy-wait); the long work (AES replies)
/// stays in thread mode. MK_USB_STATE phases localize a wedge; see markers.rs.
#[interrupt]
fn USBD_LP() {
    mk(MK_USB_IRQS, mk_get(MK_USB_IRQS) + 1);
    mk(MK_USB_STATE, 1);
    /* USBD_INTF (ISTR) snapshot before poll clears it: base+0x44 */
    mk(MK_USB_ISTR, unsafe { core::ptr::read_volatile(0x4000_5C44 as *const u32) });
    let u = unsafe { &*USB.0.get() };
    if u.dev.is_null() {
        mk(MK_USB_SPUR, mk_get(MK_USB_SPUR) + 1);
        mk(MK_USB_STATE, 0);
        return;
    }
    let (dev, pz, serial) = unsafe { (&mut *u.dev, &mut *u.pz, &mut *u.serial) };
    /* Stock-firmware drain-loop shape (QUIRKS §2): service until no
     * endpoint shows a pending transfer event, so one IRQ entry never
     * strands a second event behind a possibly-lost re-fire. The stock ISR
     * loops on `ISTR & CTR`; ours loops on the EPxCS ST flags instead —
     * EPxCS reads are side-effect-free (rc_w0), while a bare ISTR read
     * measurably consumes events on this clone (builds 50-52 A/B). Bounded:
     * a flag poll cannot clear (e.g. usbd-serial's buffer full holding
     * RX_ST as backpressure) must not spin the ISR — the level IRQ (or the
     * next event) refires us for anything left behind. */
    let mut passes = 0u32;
    loop {
        dev.poll(&mut [&mut *pz, &mut *serial]);
        mk(MK_USB_STATE, 2);
        /* powerz: answer Connect/Disconnect, then make sure the pre-built
         * ADC reply is armed (endpoint_in_complete re-arms inside poll;
         * this covers the first arm and queued-command-reply handoffs) */
        pz.poll_cmd();
        pz.arm();
        mk(MK_USB_STATE, 3);
        if drain_serial(serial) {
            /* ring full: park (see drain_serial) — the thread unmasks after
             * it frees a slot, and the pending CTR refires us exactly once */
            IRQ_PARKED.store(true, Ordering::Release);
            cortex_m::peripheral::NVIC::mask(pac::Interrupt::USBD_LP);
            break;
        }
        passes += 1;
        if !ep_events_pending() {
            break;
        }
        if passes >= 4 {
            /* still pending after 4 passes: something poll won't clear this
             * entry (backpressure) — leave it for the re-fire, count it */
            mk(MK_USB_RDMAX, mk_get(MK_USB_RDMAX) + 1);
            break;
        }
    }
    if passes > 1 {
        /* extra same-entry service passes the drain loop earned */
        mk(MK_USB_REDRAIN, mk_get(MK_USB_REDRAIN) + passes - 1);
    }
    mk(MK_USB_STATE, 4);
    mk(MK_USB_STATE, 0);
}

/// Any endpoint with a pending transfer event (EPxCS RX_ST bit 15 /
/// TX_ST bit 7)? Read-only scan — EPxCS ST flags are rc_w0, so reads are
/// side-effect-free (unlike ISTR on this clone). Unconfigured EPxCS read 0.
#[inline(never)]
fn ep_events_pending() -> bool {
    let usbd = unsafe { &*pac::Usbd::ptr() };
    for n in 0..8u32 {
        let cs = match n {
            0 => usbd.ep0cs().read().bits(),
            1 => usbd.ep1cs().read().bits(),
            2 => usbd.ep2cs().read().bits(),
            3 => usbd.ep3cs().read().bits(),
            4 => usbd.ep4cs().read().bits(),
            5 => usbd.ep5cs().read().bits(),
            6 => usbd.ep6cs().read().bits(),
            _ => usbd.ep7cs().read().bits(),
        };
        if cs & 0x8080 != 0 {
            return true;
        }
    }
    false
}

/// Build + publish the powerz reply (thread mode; the copy into
/// powerz::REPLY runs inside the critical section).
#[inline(never)]
fn refresh_powerz_payload() {
    let pl = powerz_payload();
    with_usb(|_, pz, _| pz.set_payload(&pl));
}

/// Build the 44-byte powerz ADC payload (layout: powerz.rs / HARDWARE.md).
/// All uV/uA are our wire units scaled up (1e-4 V -> x100 uV, 1e-5 A ->
/// x10 uA); Vcc1/Vcc2 come from the PD engine's CC sweep (live while armed,
/// else 0), Vdd from the Vrefint ratio; temp is clamped at 0 (the kernel
/// driver's byte pair can't encode sub-zero).
#[inline(never)]
fn powerz_payload() -> [u8; powerz::ADC_PAYLOAD_LEN] {
    let m = ina::snapshot();
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
    /* Vcc1/Vcc2: CC-pin voltages from the PD engine's MDAC sweep — live
     * while the engine is armed (`pdctl 1`), 0 otherwise. Vdd: logic rail
     * via the Vrefint ratio (each window's ADC burst). */
    #[cfg(feature = "dev-verbs")]
    let (c1, c2) = pd::cc_mv();
    #[cfg(not(feature = "dev-verbs"))]
    let (c1, c2) = (0u16, 0u16);
    put16(26, c1 as u32 * 10); /* mV -> 0.1 mV units */
    put16(28, c2 as u32 * 10);
    put16(34, m.vdd_mv * 10);
    p
}

#[cfg(feature = "dev-verbs")]
#[inline(never)]
fn reply_ok_er(r: Result<(), u8>) {
    match r {
        Ok(()) => write_all(b"OK"),
        Err(stage) => write_all(&[b'E', b'R', stage]),
    }
}

/// Parse + run an `i2crd` packet; fill `reply`, return its length.
#[cfg(feature = "dev-verbs")]
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
    /* thread-mode I2C must never race the sampler ISR's transactions —
     * pause the tick for the duration (ina.rs bus rule) */
    match ina::pause(|| i2c::transact(addr7, &wbuf[..wlen], &mut rbuf[..rlen])) {
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

/// Blocking-ish write with an absent-host bound (the C had the same bounded
/// wait): feed usbd-serial's buffer under short critical sections — one
/// buffer copy / endpoint write each, never held across slow work. The USBD
/// ISR kicks and continues the actual bus traffic (serial.write starts a
/// packet itself; endpoint_in_complete flushes the rest), so WouldBlock here
/// just means "wait for the ISR to drain some".
#[inline(never)]
fn write_all(data: &[u8]) {
    let mut sent = 0usize;
    let mut budget = 200_000u32;
    while budget > 0 {
        budget -= 1;
        /* No thread-mode dev.poll sweep here anymore: it existed because
         * the slow-flash single-pass ISR lost completion events (~20-30%)
         * and reply drains stalled in ~50 ms steps. The EPxCS drain loop
         * in the ISR (QUIRKS §2) services those events at the source, and
         * removing the sweep leaves exactly ONE hardware consumer in
         * steady state — the structural property that spares the stock
         * firmware from QUIRKS §3. A/B on build 62 confirmed no drain
         * stalls and unchanged latency without it. */
        let done = with_usb(|_, _, serial| {
            if sent < data.len() {
                match serial.write(&data[sent..]) {
                    Ok(n) => {
                        sent += n;
                        false
                    }
                    Err(UsbError::WouldBlock) => false,
                    Err(_) => true,
                }
            } else {
                match serial.flush() {
                    Ok(()) => true,
                    Err(UsbError::WouldBlock) => false,
                    Err(_) => true,
                }
            }
        });
        if done != Some(false) {
            return; /* finished, hard error, or USB not published yet */
        }
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
