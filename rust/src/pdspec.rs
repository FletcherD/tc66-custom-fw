//! USB Power Delivery field layouts as named constants (`triggers`).
//!
//! A field is `F { shift, width, unit }`: `width` bits at bit `shift`, one
//! raw step = `unit` (mV, mA, mW as named). `get` reads the raw value, `val`
//! the scaled one, `put` places a raw value. All `const fn` — they compile to
//! the same shifts and masks as hand-written code.
//!
//! Source: USB PD specification Revision 3.2 Version 1.2 (usb.org document
//! library); table numbers refer to it. Revision 2.0 V1.3 differs only in
//! VDO layouts we read through fields common to both. The host-side
//! counterpart (all tables) is tools/lib/pd_spec.py.

#[derive(Clone, Copy)]
pub struct F {
    pub shift: u32,
    pub width: u32,
    pub unit: u32,
}

impl F {
    pub const fn mask(self) -> u32 {
        (1 << self.width) - 1
    }
    /// Raw field value.
    pub const fn get(self, v: u32) -> u32 {
        v >> self.shift & self.mask()
    }
    /// Field value × unit.
    pub const fn val(self, v: u32) -> u32 {
        self.get(v) * self.unit
    }
    /// `raw` placed in the field (truncated to its width).
    pub const fn put(self, raw: u32) -> u32 {
        (raw & self.mask()) << self.shift
    }
}

const fn f(shift: u32, width: u32) -> F {
    F { shift, width, unit: 1 }
}

const fn fu(shift: u32, width: u32, unit: u32) -> F {
    F { shift, width, unit }
}

/* ---- Message Header (Table 6.2) ---- */
pub const HDR_EXTENDED: F = f(15, 1);
pub const HDR_NUM_OBJECTS: F = f(12, 3);
pub const HDR_MSG_ID: F = f(9, 3);
pub const HDR_POWER_ROLE: F = f(8, 1); /* SOP: 1 = source */
pub const HDR_CABLE_PLUG: F = f(8, 1); /* SOP'/SOP'': 1 = sent by a cable plug */
pub const HDR_SPEC_REV: F = f(6, 2);
pub const HDR_MSG_TYPE: F = f(0, 5);
pub const REV_2_0: u32 = 1;
pub const REV_3_X: u32 = 2;

/* ---- message types (Tables 6.4 / 6.5) ---- */
pub const CTRL_GOODCRC: u32 = 1;
pub const CTRL_ACCEPT: u32 = 3;
pub const CTRL_REJECT: u32 = 4;
pub const CTRL_PING: u32 = 5;
pub const CTRL_PS_RDY: u32 = 6;
pub const CTRL_GET_SOURCE_CAP: u32 = 7;
pub const CTRL_GET_SINK_CAP: u32 = 8;
pub const CTRL_WAIT: u32 = 12;
pub const CTRL_SOFT_RESET: u32 = 13;
pub const CTRL_NOT_SUPPORTED: u32 = 16; /* PD 3.x */
pub const DATA_SOURCE_CAP: u32 = 1;
pub const DATA_REQUEST: u32 = 2;
pub const DATA_SINK_CAP: u32 = 4;
pub const DATA_VENDOR_DEFINED: u32 = 15;

/* ---- Power Data Objects (Tables 6.6 - 6.16) ---- */
pub const PDO_TYPE: F = f(30, 2);
pub const PDO_FIXED: u32 = 0;
pub const PDO_BATTERY: u32 = 1;
pub const PDO_VARIABLE: u32 = 2;
pub const PDO_APDO: u32 = 3;
pub const APDO_TYPE: F = f(28, 2);
pub const APDO_PPS: u32 = 0;
pub const APDO_SPR_AVS: u32 = 2;
/* fixed: voltage + max current; battery / variable: min voltage at the same
 * bits as the fixed voltage, max voltage above it */
pub const FIXED_VOLTAGE: F = fu(10, 10, 50); /* mV */
pub const FIXED_MAX_CURRENT: F = fu(0, 10, 10); /* mA */
pub const RANGE_MIN_VOLTAGE: F = fu(10, 10, 50); /* mV, battery / variable */
pub const RANGE_MAX_VOLTAGE: F = fu(20, 10, 50); /* mV */
pub const VARIABLE_MAX_CURRENT: F = fu(0, 10, 10); /* mA */
pub const BATTERY_MAX_POWER: F = fu(0, 10, 250); /* mW */
pub const PPS_MAX_VOLTAGE: F = fu(17, 8, 100); /* mV */
pub const PPS_MIN_VOLTAGE: F = fu(8, 8, 100); /* mV */
pub const PPS_MAX_CURRENT: F = fu(0, 7, 50); /* mA */
pub const SPR_AVS_MAX_CURRENT_15V: F = fu(10, 10, 10); /* mA, 9-15 V */

/* ---- Request Data Objects (Tables 6.19 - 6.22) ---- */
pub const RDO_OBJECT_POSITION: F = f(28, 4);
pub const RDO_NO_USB_SUSPEND: F = f(24, 1);
pub const RDO_OPERATING_CURRENT: F = fu(10, 10, 10); /* mA, fixed / variable */
pub const RDO_MAX_CURRENT: F = fu(0, 10, 10); /* mA */
pub const RDO_OPERATING_POWER: F = fu(10, 10, 250); /* mW, battery */
pub const RDO_PPS_VOLTAGE: F = fu(9, 12, 20); /* mV */
pub const RDO_AVS_VOLTAGE: F = fu(9, 12, 25); /* mV */
pub const RDO_APDO_CURRENT: F = fu(0, 7, 50); /* mA, PPS / AVS */

/* ---- Structured VDM header (Table 6.33) ---- */
pub const VDM_STRUCTURED: F = f(15, 1);
pub const VDM_COMMAND_TYPE: F = f(6, 2);
pub const VDM_ACK: u32 = 1;
pub const VDM_COMMAND: F = f(0, 5);
pub const CMD_DISCOVER_IDENTITY: u32 = 1;
/* Discover Identity ACK objects: VDM header, ID Header, Cert Stat, Product,
 * then the product type VDO(s) */
pub const DISCID_VDM_HEADER: usize = 0;
pub const DISCID_ID_HEADER: usize = 1;
pub const DISCID_PRODUCT_TYPE_VDO: usize = 4;
pub const DISCID_CABLE_MIN_OBJECTS: usize = DISCID_PRODUCT_TYPE_VDO + 1;

/* ---- ID Header VDO (Table 6.34) ---- */
pub const IDH_PRODUCT_TYPE: F = f(27, 3); /* SOP': cable plug / VPD */
pub const IDH_VID: F = f(0, 16);
pub const CABLE_PASSIVE: u32 = 3;
pub const CABLE_ACTIVE: u32 = 4;

/* ---- Passive / active cable VDO (Tables 6.42 / 6.43) ---- */
pub const CABLE_MAX_VBUS: F = f(9, 2); /* PD 3.x: 0 20 V, 1/2 (deprecated) 30/40 V, 3 50 V */
pub const CABLE_MAX_VBUS_BASE_V: u32 = 20;
pub const CABLE_MAX_VBUS_STEP_V: u32 = 10;
pub const CABLE_VBUS_CURRENT: F = f(5, 2);
pub const CABLE_CURRENT_3A: u32 = 1;
pub const CABLE_CURRENT_5A: u32 = 2;
pub const CABLE_USB_SPEED: F = f(0, 3);
