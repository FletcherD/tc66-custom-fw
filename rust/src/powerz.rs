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
//! github.com/okhsunrog/km003c-protocol-research (see HARDWARE.md).

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
const RSP_PUT_DATA: u8 = 0x41;
const ATT_ADC: u16 = 0x0001;

pub const ADC_PAYLOAD_LEN: usize = 44;
const ADC_REPLY_LEN: usize = 8 + ADC_PAYLOAD_LEN;

pub struct PowerZ<'a, B: UsbBus> {
    iface: InterfaceNumber,
    ep_out: EndpointOut<'a, B>,
    ep_in: EndpointIn<'a, B>,
    /* one pending 4-byte command response (Accept/Reject), retried until
     * the IN endpoint takes it */
    pending: [u8; 4],
    pending_len: usize,
}

/* The pre-built PutData(ADC) reply lives in a static, NOT in the struct:
 * main()'s stack frame is within ~100 B of overflowing the deliberately
 * fenced RAM region (memory.x), and a boot-path overflow hardfaults before
 * the D+ pull-up — an invisible dead board needing K1 recovery. Single-core,
 * IRQs masked -> Racy ok. */
struct RacyCell<T>(core::cell::UnsafeCell<T>);
unsafe impl<T> Sync for RacyCell<T> {}
static REPLY: RacyCell<[u8; ADC_REPLY_LEN]> = RacyCell(core::cell::UnsafeCell::new([0; ADC_REPLY_LEN]));

/* TEMP diagnosis counters (payload tail via dbg_get): 0=in_complete,
 * 1=arm write ok, 2=arm write WouldBlock, 3=flush-false in arm,
 * 4=pending queued, 5=last non-GetData cmd byte, 6=GetData count */
static DBG: RacyCell<[u16; 7]> = RacyCell(core::cell::UnsafeCell::new([0; 7]));
pub fn dbg_bump(i: usize) {
    unsafe { (*DBG.0.get())[i] = (*DBG.0.get())[i].wrapping_add(1) };
}
pub fn dbg_set(i: usize, v: u16) {
    unsafe { (*DBG.0.get())[i] = v };
}
pub fn dbg_get() -> [u16; 7] {
    unsafe { *DBG.0.get() }
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
        }
    }

    /// Retry a stashed reply; true once nothing is pending.
    fn flush(&mut self) -> bool {
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
            dbg_bump(3);
            return;
        }
        let reply = unsafe { &*REPLY.0.get() };
        match self.ep_in.write(reply) {
            Ok(_) => dbg_bump(1),
            Err(_) => dbg_bump(2),
        }
    }

    /// Service the OUT endpoint. GetData needs no action (the pre-armed
    /// reply serves it); Connect/Disconnect get Accept, the rest Reject —
    /// those queue behind the armed ADC packet, which only matters for
    /// hosts beyond the kernel driver (Tier 2 work).
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
            CMD_GET_DATA if attr & ATT_ADC != 0 => dbg_bump(6),
            CMD_CONNECT | CMD_DISCONNECT => {
                dbg_bump(4);
                dbg_set(5, cmd as u16);
                self.queue(&[RSP_ACCEPT, tid, 0, 0]);
            }
            _ => {
                dbg_bump(4);
                dbg_set(5, cmd as u16 | 0x8000);
                self.queue(&[RSP_REJECT, tid, 0, 0]);
            }
        }
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
        dbg_bump(0);
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
