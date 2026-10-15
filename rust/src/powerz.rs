//! ChargerLAB POWER-Z KM003C emulation (vendor interface 0).
//!
//! The mainline Linux `powerz` hwmon driver binds VID:PID 5FC9:0063 interface
//! 00 and speaks one transaction: 4-byte GetData(ADC) command to bulk EP 0x01,
//! reply on bulk EP 0x81 — 4-byte data header + 4-byte extended header +
//! 44-byte ADC payload. The endpoint ADDRESSES are hard-coded in the driver,
//! so this class must be constructed FIRST (interface 0, endpoint index 1).
//!
//! Control header:  [type:7|ext:1][tid][attr15<<1 LE]  (GetData attr 1 = ADC
//! arrives as 0c tid 02 00). Data header: [0x41][tid][len-words LE]. Extended
//! header: attr | chunk<<16 | size<<22, LE u32. Protocol reference:
//! github.com/okhsunrog/km003c-protocol-research (see docs/HARDWARE.md).

use usb_device::bus::{InterfaceNumber, UsbBus, UsbBusAllocator};
use usb_device::class::UsbClass;
use usb_device::descriptor::DescriptorWriter;
use usb_device::endpoint::{EndpointAddress, EndpointIn, EndpointOut, EndpointType};
use usb_device::Result;
use usb_device::UsbDirection;

const CMD_CONNECT: u8 = 0x02;
const CMD_DISCONNECT: u8 = 0x03;
const RSP_ACCEPT: u8 = 0x05;
const RSP_REJECT: u8 = 0x06;
const CMD_GET_DATA: u8 = 0x0C;
const CMD_ENABLE_PD_MONITOR: u8 = 0x10;
const CMD_DISABLE_PD_MONITOR: u8 = 0x11;
const RSP_PUT_DATA: u8 = 0x41;
const ATT_ADC: u16 = 0x0001;
const ATT_PD: u16 = 0x0010; /* KM003C PdPacket (PD capture) */
const ATT_PD_TRACE: u16 = 0x0020; /* KM003C PD state trace */
const EXT_NEXT: u32 = 1 << 15; /* extended header: another logical packet follows */

/// Set by EnablePdMonitor (USB ISR); the main loop starts the sniffer if
/// the PD engine is off.
pub static PD_MONITOR_REQ: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

pub const ADC_PAYLOAD_LEN: usize = 44;
const ADC_REPLY_LEN: usize = 8 + ADC_PAYLOAD_LEN;
/* one PD reply per poll fits a single 64-B bulk packet: 8 header + up to 56
 * payload (12-B preamble + one or two events); the host polls to drain more */
const PD_REPLY_MAX: usize = 64;
/* ADC + PD (GetData 0x0011): 8 + 44 ADC + 4 + PD payload, sent as two bulk
 * packets; capped at 127 so the transfer always ends on a short packet
 * (no zero-length packet needed) */
const COMBINED_MAX: usize = 127;

pub struct PowerZ<'a, B: UsbBus> {
    iface: InterfaceNumber,
    ep_out: EndpointOut<'a, B>,
    ep_in: EndpointIn<'a, B>,
    /* one pending 4-byte command response (Accept/Reject), retried until
     * the IN endpoint takes it */
    pending: [u8; 4],
    pending_len: usize,
    /* pending PD_REPLY length (PD capture / trace / ADC+PD); 0 = none —
     * sent in 64-B packets, pd_sent bytes of it so far */
    pd_pending: usize,
    pd_sent: usize,
}

/* The pre-built PutData(ADC) reply lives in a static, NOT in the struct:
 * main()'s stack frame is within ~100 B of overflowing the deliberately
 * fenced RAM region (memory.x), and a boot-path overflow hardfaults before
 * the D+ pull-up — an invisible dead board needing K1 recovery.
 *
 * Soundness (multi-word buffer shared with the USBD_LP ISR): REPLY is only
 * reached through PowerZ's methods, and the PowerZ object itself is only
 * reachable from the ISR or from thread mode under interrupt::free
 * (with_usb in main.rs). Those contexts are mutually exclusive, so a
 * set_payload copy (thread, in a critical section) can never interleave
 * with an arm/endpoint_in_complete read (ISR) — the armed packet is always
 * a whole, untorn payload. */
struct RacyCell<T>(core::cell::UnsafeCell<T>);
unsafe impl<T> Sync for RacyCell<T> {}
/* header pre-baked so the first arms (grace window, before any set_payload)
 * already send a well-formed PutData reply with zero fields */
const fn reply_template() -> [u8; ADC_REPLY_LEN] {
    let mut r = [0u8; ADC_REPLY_LEN];
    r[0] = RSP_PUT_DATA;
    let words = ((4 + ADC_PAYLOAD_LEN) / 4) as u16;
    r[2] = words as u8;
    r[3] = (words >> 8) as u8;
    let ext: u32 = ATT_ADC as u32 | (ADC_PAYLOAD_LEN as u32) << 22;
    r[4] = ext as u8;
    r[5] = (ext >> 8) as u8;
    r[6] = (ext >> 16) as u8;
    r[7] = (ext >> 24) as u8;
    r
}
static REPLY: RacyCell<[u8; ADC_REPLY_LEN]> =
    RacyCell(core::cell::UnsafeCell::new(reply_template()));
/* PD-capture PutData reply, built on demand in poll_cmd (same exclusive
 * powerz context as REPLY). A static, not a struct field, for the same
 * stack-budget reason. */
static PD_REPLY: RacyCell<[u8; COMBINED_MAX]> =
    RacyCell(core::cell::UnsafeCell::new([0; COMBINED_MAX]));

fn ext_header(att: u16, next: bool, size: usize) -> [u8; 4] {
    (att as u32 | if next { EXT_NEXT } else { 0 } | (size as u32) << 22).to_le_bytes()
}

impl<'a, B: UsbBus> PowerZ<'a, B> {
    pub fn new(alloc: &'a UsbBusAllocator<B>) -> PowerZ<'a, B> {
        PowerZ {
            iface: alloc.interface(),
            ep_out: alloc
                .alloc(
                    Some(EndpointAddress::from_parts(1, UsbDirection::Out)),
                    EndpointType::Bulk,
                    64,
                    0,
                )
                .unwrap(),
            ep_in: alloc
                .alloc(
                    Some(EndpointAddress::from_parts(1, UsbDirection::In)),
                    EndpointType::Bulk,
                    64,
                    0,
                )
                .unwrap(),
            pending: [0; 4],
            pending_len: 0,
            pd_pending: 0,
            pd_sent: 0,
        }
    }

    /// Retry any stashed reply (PD capture first, then a command response);
    /// true once nothing is pending. PD replies take priority over the
    /// re-armed ADC packet so a GetData(ATT_PD) is answered with PD data.
    fn flush(&mut self) -> bool {
        while self.pd_sent < self.pd_pending {
            let r = unsafe { &*PD_REPLY.0.get() };
            let end = (self.pd_sent + 64).min(self.pd_pending);
            match self.ep_in.write(&r[self.pd_sent..end]) {
                Ok(_) => self.pd_sent = end,
                Err(_) => return false,
            }
        }
        self.pd_pending = 0;
        self.pd_sent = 0;
        if self.pending_len == 0 {
            return true;
        }
        match self.ep_in.write(&self.pending[..self.pending_len]) {
            Ok(_) => {
                self.pending_len = 0;
                true
            }
            Err(_) => false,
        }
    }

    fn queue(&mut self, data: &[u8]) {
        self.pending[..data.len()].copy_from_slice(data);
        self.pending_len = data.len();
        self.flush();
    }

    /// Refresh the pre-built PutData(ADC) reply (tid 0 — the kernel driver
    /// reads the struct at fixed offsets and ignores headers). Called once
    /// per tick from calm context; `arm` then only has to write it.
    pub fn set_payload(&mut self, payload: &[u8; ADC_PAYLOAD_LEN]) {
        let reply = unsafe { &mut *REPLY.0.get() };
        reply[0] = RSP_PUT_DATA;
        let words = ((4 + ADC_PAYLOAD_LEN) / 4) as u16; /* ext header + payload */
        reply[2] = words as u8;
        reply[3] = (words >> 8) as u8;
        let ext: u32 = ATT_ADC as u32 | (ADC_PAYLOAD_LEN as u32) << 22;
        reply[4..8].copy_from_slice(&ext.to_le_bytes());
        reply[8..].copy_from_slice(payload);
    }

    /// Keep the pre-built ADC reply armed on the IN endpoint. Pre-arming
    /// means the host's IN completes immediately instead of NAK-polling us:
    /// sustained IN NAK traffic on this USBD stalls the CPU so badly
    /// (~20-100x on APB access and straight-line flash fetch) that a
    /// build-on-demand reply blows the powerz driver's 5 ms URB timeout.
    /// The armed packet is at most one host poll interval stale; every read
    /// after the first in a burst gets tick-fresh data. A queued command
    /// response takes priority.
    ///
    /// The write is UNCONDITIONAL (WouldBlock while armed is the cheap
    /// no-op): tracking armed-ness in software does not work, because
    /// endpoint_in_complete events are silently dropped ~20% of the time on
    /// this stack/clone (bench-measured 320 callbacks for ~400 consumed
    /// replies) — a stale "armed" latch left the endpoint chronically empty
    /// and 54% of host reads timing out. Do not reintroduce a skip flag.
    pub fn arm(&mut self) {
        if !self.flush() {
            return;
        }
        let reply = unsafe { &*REPLY.0.get() };
        let _ = self.ep_in.write(reply); /* WouldBlock while armed = cheap no-op */
    }

    /// Service the OUT endpoint. GetData(ADC) needs no action (the pre-armed
    /// reply serves it); PD, ADC+PD and PD-trace GetData build a reply;
    /// Connect/Disconnect/Enable-/DisablePdMonitor get Accept (Enable also
    /// requests the sniffer), the rest Reject — those queue behind the
    /// armed ADC packet, which only matters for hosts beyond the kernel
    /// driver.
    pub fn poll_cmd(&mut self) {
        let mut buf = [0u8; 64];
        match self.ep_out.read(&mut buf) {
            Ok(n) if n >= 4 => {}
            _ => return,
        }
        let cmd = buf[0] & 0x7F;
        let tid = buf[1];
        let attr = (buf[2] as u16 | (buf[3] as u16) << 8) >> 1;
        match cmd {
            CMD_GET_DATA if attr == ATT_PD => self.build_pd(tid, false),
            CMD_GET_DATA if attr == ATT_ADC | ATT_PD => self.build_pd(tid, true),
            CMD_GET_DATA if attr == ATT_PD_TRACE => self.build_trace(tid),
            CMD_GET_DATA if attr & ATT_ADC != 0 => {} /* pre-armed reply serves it */
            CMD_CONNECT | CMD_DISCONNECT | CMD_DISABLE_PD_MONITOR => {
                self.queue(&[RSP_ACCEPT, tid, 0, 0]);
            }
            CMD_ENABLE_PD_MONITOR => {
                PD_MONITOR_REQ.store(true, core::sync::atomic::Ordering::Relaxed);
                self.queue(&[RSP_ACCEPT, tid, 0, 0]);
            }
            _ => {
                self.queue(&[RSP_REJECT, tid, 0, 0]);
            }
        }
    }

    /// Queue the `len`-byte PutData reply built in PD_REPLY (main header
    /// filled here). A reply still being sent is never overwritten — its
    /// events are already drained from the ring; the host gets it instead.
    fn send_built(&mut self, tid: u8, len: usize) {
        let reply = unsafe { &mut *PD_REPLY.0.get() };
        let words = ((len - 4 + 3) / 4) as u16; /* everything after the main header */
        reply[0] = RSP_PUT_DATA;
        reply[1] = tid;
        reply[2] = words as u8;
        reply[3] = (words >> 8) as u8;
        self.pd_pending = len;
        self.pd_sent = 0;
        self.flush();
    }

    /// Build a PutData(ATT_PD) reply from the pd.rs capture ring — after the
    /// current ADC payload as a chained logical packet when `with_adc`
    /// (GetData 0x0011) — and queue it. The host may read the pre-armed ADC
    /// packet first, then this on its next IN (filter by the ext-header
    /// attribute) — so the ADC fast-path is untouched. Empty ring still
    /// returns the 12-byte measurement preamble.
    fn build_pd(&mut self, tid: u8, with_adc: bool) {
        #[cfg(feature = "triggers")]
        {
            if self.pd_pending > 0 {
                return;
            }
            let reply = unsafe { &mut *PD_REPLY.0.get() };
            let mut off = 4;
            let mut cap = PD_REPLY_MAX;
            if with_adc {
                let adc = unsafe { &*REPLY.0.get() };
                reply[4..8].copy_from_slice(&ext_header(ATT_ADC, true, ADC_PAYLOAD_LEN));
                reply[8..ADC_REPLY_LEN].copy_from_slice(&adc[8..]);
                off = ADC_REPLY_LEN;
                cap = COMBINED_MAX;
            }
            let payload = crate::pd::drain_pd_packet(&mut reply[off + 4..cap]);
            reply[off..off + 4].copy_from_slice(&ext_header(ATT_PD, false, payload));
            self.send_built(tid, off + 4 + payload);
        }
        #[cfg(not(feature = "triggers"))]
        self.queue(&[RSP_REJECT, tid, 0, 0]); /* no PD engine without triggers */
    }

    /// PD state trace: both queues empty (count bytes 0). The KM003C's
    /// state/protocol codes are firmware-specific and undocumented.
    fn build_trace(&mut self, tid: u8) {
        if self.pd_pending > 0 {
            return;
        }
        let reply = unsafe { &mut *PD_REPLY.0.get() };
        reply[4..8].copy_from_slice(&ext_header(ATT_PD_TRACE, false, 2));
        reply[8] = 0;
        reply[9] = 0;
        self.send_built(tid, 10);
    }
}

impl<B: UsbBus> UsbClass<B> for PowerZ<'_, B> {
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> Result<()> {
        writer.iad(self.iface, 1, 0xFF, 0x00, 0x00, None)?;
        writer.interface(self.iface, 0xFF, 0x00, 0x00)?;
        writer.endpoint(&self.ep_out)?;
        writer.endpoint(&self.ep_in)?;
        Ok(())
    }

    fn reset(&mut self) {
        /* bus reset re-initializes the endpoint registers: a queued command
         * response is gone from the hardware — forget it */
        self.pending_len = 0;
        self.pd_pending = 0;
        self.pd_sent = 0;
    }

    fn endpoint_out(&mut self, addr: EndpointAddress) {
        if addr == self.ep_out.address() {
            /* serve + re-arm the command endpoint the moment the packet
             * lands (inside dev.poll): the host retries a NAKed command
             * write for only 5 ms, and waiting for a main-loop pass to
             * read it loses that race under storm-degraded loop speed */
            self.poll_cmd();
        }
    }

    fn endpoint_in_complete(&mut self, addr: EndpointAddress) {
        if addr == self.ep_in.address() && self.flush() {
            /* no command response queued — re-arm the ADC reply right
             * here, before the host's next request can start NAKing (this
             * callback is best-effort: see arm() on why the per-pass
             * unconditional write is the actual guarantee) */
            let reply = unsafe { &*REPLY.0.get() };
            let _ = self.ep_in.write(reply);
        }
    }
}
