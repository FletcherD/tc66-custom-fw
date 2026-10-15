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
const CMD_START_GRAPH: u8 = 0x0E;
const CMD_STOP_GRAPH: u8 = 0x0F;
const CMD_ENABLE_PD_MONITOR: u8 = 0x10;
const CMD_DISABLE_PD_MONITOR: u8 = 0x11;
const RSP_NOT_READABLE: u8 = 0x27;
const RSP_PUT_DATA: u8 = 0x41;
const CMD_MEMORY_READ: u8 = 0x44;
const CMD_STREAMING_AUTH: u8 = 0x4C;
const ATT_ADC: u16 = 0x0001;
const ATT_ADC_QUEUE: u16 = 0x0002; /* KM003C AdcQueue (StartGraph stream) */
/* AdcQueue wire record: seq, marker, Vbus uV, Ibus uA, CC1, CC2, D+, D- */
const AQ_RECORD: usize = 20;
/* a vendor session (Connect) ends after this long without a command */
const SESSION_IDLE_US: u32 = 2_000_000;
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
/* AdcQueue replies: where the PdPacket / trace parts are staged in PD_REPLY
 * (after the ≤56-B ADC + AdcQueue header part) */
const SUFFIX: usize = 56;

pub struct PowerZ<'a, B: UsbBus> {
    iface: InterfaceNumber,
    ep_out: EndpointOut<'a, B>,
    ep_in: EndpointIn<'a, B>,
    /* one pending 4-byte command response (Accept/Reject), retried until
     * the IN endpoint takes it */
    pending: [u8; 4],
    pending_len: usize,
    /* pending PD_REPLY length (PD capture / trace / ADC+PD / session
     * replies); 0 = none — sent in 64-B packets, pd_sent bytes of it so
     * far; pd_split ends one transfer early (MemoryRead confirmation, then
     * its data as a second transfer) */
    pd_pending: usize,
    pd_sent: usize,
    pd_split: usize,
    /* AdcQueue reply in flight: aq_pre header bytes in PD_REPLY, then
     * aq_n records from ring index aq_first, built per 64-B packet;
     * aq_len total, aq_sent so far (0/0 = none) */
    aq_len: usize,
    aq_sent: usize,
    aq_pre: usize,
    aq_rec_end: usize,
    aq_first: u32,
    aq_zlp: bool,
    /* a host sent Connect (km003c-rs, Mtools — never the kernel driver):
     * stop pre-arming the ADC reply and answer every request on demand,
     * so no stale packet sits between a request and its reply */
    session: bool,
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

/// Fill PD_REPLY's PutData main header for a `len`-byte reply.
fn put_data_header(tid: u8, len: usize) {
    let reply = unsafe { &mut *PD_REPLY.0.get() };
    let words = ((len - 4 + 3) / 4) as u16; /* everything after the main header */
    reply[0] = RSP_PUT_DATA;
    reply[1] = tid;
    reply[2] = words as u8;
    reply[3] = (words >> 8) as u8;
}

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
            pd_split: 0,
            aq_len: 0,
            aq_sent: 0,
            aq_pre: 0,
            aq_rec_end: 0,
            aq_first: 0,
            aq_zlp: false,
            session: false,
        }
    }

    /// Retry any stashed reply (PD capture first, then a command response);
    /// true once nothing is pending. PD replies take priority over the
    /// re-armed ADC packet so a GetData(ATT_PD) is answered with PD data.
    fn flush(&mut self) -> bool {
        while self.pd_sent < self.pd_pending {
            let r = unsafe { &*PD_REPLY.0.get() };
            let mut end = (self.pd_sent + 64).min(self.pd_pending);
            if self.pd_split > self.pd_sent {
                end = end.min(self.pd_split);
            }
            match self.ep_in.write(&r[self.pd_sent..end]) {
                Ok(_) => self.pd_sent = end,
                Err(_) => return false,
            }
        }
        self.pd_pending = 0;
        self.pd_sent = 0;
        self.pd_split = 0;
        while self.aq_sent < self.aq_len {
            let mut pkt = [0u8; 64];
            let n = (self.aq_len - self.aq_sent).min(64);
            self.aq_fill(&mut pkt[..n]);
            match self.ep_in.write(&pkt[..n]) {
                Ok(_) => self.aq_sent += n,
                Err(_) => return false,
            }
        }
        if self.aq_zlp {
            match self.ep_in.write(&[]) {
                Ok(_) => self.aq_zlp = false,
                Err(_) => return false,
            }
        }
        self.aq_len = 0;
        self.aq_sent = 0;
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
        if self.session && crate::adcq::idle_us() > SESSION_IDLE_US {
            self.session = false; /* host went away without Disconnect */
        }
        if !self.flush() || self.session {
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
        crate::adcq::host_seen();
        match cmd {
            CMD_GET_DATA if attr == ATT_PD => self.build_pd(tid, false),
            CMD_GET_DATA if attr == ATT_ADC | ATT_PD => self.build_pd(tid, true),
            CMD_GET_DATA if attr == ATT_PD_TRACE => self.build_trace(tid),
            CMD_GET_DATA
                if attr & ATT_ADC_QUEUE != 0
                    && attr & !(ATT_ADC | ATT_ADC_QUEUE | ATT_PD | ATT_PD_TRACE) == 0 =>
            {
                self.build_aq(tid, attr)
            }
            /* outside a session the pre-armed reply serves it */
            CMD_GET_DATA if attr & ATT_ADC != 0 => {
                if self.session && !self.busy() {
                    let r = unsafe { &mut *PD_REPLY.0.get() };
                    r[..ADC_REPLY_LEN].copy_from_slice(unsafe { &*REPLY.0.get() });
                    self.send_built(tid, ADC_REPLY_LEN);
                }
            }
            CMD_CONNECT => {
                self.session = true;
                self.queue(&[RSP_ACCEPT, tid, 0, 0]);
            }
            CMD_DISCONNECT => {
                self.session = false;
                crate::adcq::request(crate::adcq::OFF);
                self.queue(&[RSP_ACCEPT, tid, 0, 0]);
            }
            CMD_DISABLE_PD_MONITOR => {
                self.queue(&[RSP_ACCEPT, tid, 0, 0]);
            }
            CMD_START_GRAPH if attr < 4 => {
                crate::adcq::request(attr as u8 + 1);
                self.queue(&[RSP_ACCEPT, tid, 0, 0]);
            }
            CMD_STOP_GRAPH if crate::adcq::requested() != crate::adcq::OFF => {
                crate::adcq::request(crate::adcq::OFF);
                self.queue(&[RSP_ACCEPT, tid, 0, 0]);
            }
            /* StreamingAuth: any credential gets level 1 (result word
             * 0x0203, response id 0); the payload is echoed, not
             * re-encrypted (km003c-rs never checks it; Mtools would) */
            CMD_STREAMING_AUTH if buf[2..4] == [0x00, 0x02] && !self.busy() => {
                let r = unsafe { &mut *PD_REPLY.0.get() };
                r[..4].copy_from_slice(&[CMD_STREAMING_AUTH, 0, 0x03, 0x02]);
                r[4..36].copy_from_slice(&buf[4..36]);
                self.send_raw(36, 0);
            }
            /* MemoryRead: only the HardwareID block, matched by its fixed
             * request ciphertext (km003c_consts.rs) — confirmation, then the
             * encrypted block as a second transfer */
            CMD_MEMORY_READ
                if buf[2..4] == [0x01, 0x01]
                    && buf[4..20] == crate::km003c_consts::HWID_REQ_BLOCK0
                    && !self.busy() =>
            {
                let r = unsafe { &mut *PD_REPLY.0.get() };
                r[..4].copy_from_slice(&[CMD_MEMORY_READ | 0x80, tid, 0x01, 0x01]);
                r[4..20].copy_from_slice(&crate::km003c_consts::HWID_CONFIRM);
                r[20..36].copy_from_slice(&crate::km003c_consts::HWID_DATA);
                self.send_raw(36, 20);
            }
            CMD_MEMORY_READ => {
                self.queue(&[RSP_NOT_READABLE, tid, 0, 0]);
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
        put_data_header(tid, len);
        self.send_raw(len, 0);
    }

    /// Queue `len` bytes of PD_REPLY as they are, ending the first transfer
    /// after `split` bytes when nonzero.
    fn send_raw(&mut self, len: usize, split: usize) {
        self.pd_pending = len;
        self.pd_sent = 0;
        self.pd_split = split;
        self.flush();
    }

    /// A multi-packet reply is still in flight (PD_REPLY is in use).
    fn busy(&self) -> bool {
        self.pd_pending > 0 || self.aq_len > 0
    }

    /// GetData with AdcQueue in the mask (`attr`: AdcQueue plus any of ADC,
    /// PdPacket, PD trace — km003c-rs's GUI asks for AdcQueue + PdPacket):
    /// the logical packets in attribute order, ADC + the AdcQueue ext header
    /// in PD_REPLY[..aq_pre], up to `adcq::MAX_TAKE` records expanded from
    /// the ring, then PdPacket / trace from PD_REPLY[SUFFIX..]. Empty parts
    /// are left out (km003c-rs only checks that each part was requested);
    /// AdcQueue alone with nothing buffered is the bare 4-B header, the real
    /// device's empty reply. A total that is a multiple of 64 B ends with a
    /// zero-length packet.
    fn build_aq(&mut self, tid: u8, attr: u16) {
        if self.busy() {
            return;
        }
        let reply = unsafe { &mut *PD_REPLY.0.get() };
        let (first, n) = crate::adcq::take(crate::adcq::MAX_TAKE);
        let mut suf = SUFFIX;
        #[cfg(feature = "triggers")]
        if attr & ATT_PD != 0 {
            let payload = crate::pd::drain_pd_packet(&mut reply[SUFFIX + 4..COMBINED_MAX - 6]);
            reply[SUFFIX..SUFFIX + 4]
                .copy_from_slice(&ext_header(ATT_PD, attr & ATT_PD_TRACE != 0, payload));
            suf += 4 + payload;
        }
        if attr & ATT_PD_TRACE != 0 {
            reply[suf..suf + 4].copy_from_slice(&ext_header(ATT_PD_TRACE, false, 2));
            reply[suf + 4] = 0;
            reply[suf + 5] = 0;
            suf += 6;
        }
        let mut pre = 4;
        if attr & ATT_ADC != 0 {
            let adc = unsafe { &*REPLY.0.get() };
            reply[8..ADC_REPLY_LEN].copy_from_slice(&adc[8..]);
            let more = n > 0 || suf > SUFFIX;
            reply[4..8].copy_from_slice(&ext_header(ATT_ADC, more, ADC_PAYLOAD_LEN));
            pre = ADC_REPLY_LEN;
        }
        if n > 0 {
            /* chunk = record count, size = bytes per record */
            let ext = ATT_ADC_QUEUE as u32
                | ((suf > SUFFIX) as u32) << 15
                | n << 16
                | (AQ_RECORD as u32) << 22;
            reply[pre..pre + 4].copy_from_slice(&ext.to_le_bytes());
            pre += 4;
        }
        self.aq_pre = pre;
        self.aq_first = first;
        self.aq_rec_end = pre + AQ_RECORD * n as usize;
        self.aq_len = self.aq_rec_end + suf - SUFFIX;
        self.aq_zlp = self.aq_len % 64 == 0;
        self.aq_sent = 0;
        put_data_header(tid, self.aq_len);
        self.flush();
    }

    /// Bytes `aq_sent..aq_sent + out.len()` of the AdcQueue reply: the
    /// header from PD_REPLY, records expanded from the ring, then the
    /// PdPacket / trace suffix from PD_REPLY[SUFFIX..].
    fn aq_fill(&self, out: &mut [u8]) {
        let reply = unsafe { &*PD_REPLY.0.get() };
        let adc = unsafe { &*REPLY.0.get() };
        let mut p = self.aq_sent;
        let mut o = 0;
        while o < out.len() {
            if p < self.aq_pre || p >= self.aq_rec_end {
                let q = if p < self.aq_pre { p } else { SUFFIX + p - self.aq_rec_end };
                out[o] = reply[q];
                o += 1;
                p += 1;
                continue;
            }
            let k = (p - self.aq_pre) / AQ_RECORD;
            let f = (p - self.aq_pre) % AQ_RECORD;
            let (s, seq) = crate::adcq::get(self.aq_first.wrapping_add(k as u32));
            let mut rec = [0u8; AQ_RECORD];
            rec[0..2].copy_from_slice(&seq.to_le_bytes());
            rec[4..8].copy_from_slice(&s.v_uv.to_le_bytes());
            rec[8..12].copy_from_slice(&s.i_ua.to_le_bytes());
            /* CC1, CC2, D+, D- from the ADC payload (0.1 mV): kept at 2
             * samples/s, 1 mV at the faster rates (the KM003C's units) */
            let div = if crate::adcq::active() == 1 { 1 } else { 10 };
            for c in 0..4 {
                let a = 8 + 26 + 2 * c;
                let v = u16::from_le_bytes([adc[a], adc[a + 1]]) / div;
                rec[12 + 2 * c..14 + 2 * c].copy_from_slice(&v.to_le_bytes());
            }
            let take = (AQ_RECORD - f).min(out.len() - o);
            out[o..o + take].copy_from_slice(&rec[f..f + take]);
            o += take;
            p += take;
        }
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
        self.pd_split = 0;
        self.aq_len = 0;
        self.aq_sent = 0;
        self.aq_zlp = false;
        self.session = false;
        crate::adcq::request(crate::adcq::OFF);
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
        if addr == self.ep_in.address() && self.flush() && !self.session {
            /* no command response queued — re-arm the ADC reply right
             * here, before the host's next request can start NAKing (this
             * callback is best-effort: see arm() on why the per-pass
             * unconditional write is the actual guarantee) */
            let reply = unsafe { &*REPLY.0.get() };
            let _ = self.ep_in.write(reply);
        }
    }
}
