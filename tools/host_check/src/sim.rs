//! Stand-ins for the firmware modules pdinfo.rs and detect.rs call, driven by
//! one scripted state (`set`/`get`).

use std::sync::Mutex;

#[derive(Clone)]
pub struct Sim {
    pub mode: u8,
    pub src: bool,
    pub snk: bool,
    pub cc: u8,
    pub cc_mv: u16,
    pub v_1e4: u32,
    pub i_1e5: u32,
    pub dp_1e2: u32,
    pub dm_1e2: u32,
    pub qc_class: u16,
    pub afc_ok: bool,
    pub fcp: Result<[u8; 2], u8>,
    pub arm: Result<(), u8>,
    pub status: [u8; 20],
    pub caps: Vec<u32>,
    pub ticks: u32,
}

pub const DEFAULT: Sim = Sim {
    mode: pd::MODE_SNIFF,
    src: true,
    snk: true,
    cc: 1,
    cc_mv: 1680,
    v_1e4: 90_210, /* 9.021 V */
    i_1e5: 123_400, /* 1.234 A */
    dp_1e2: 0,
    dm_1e2: 0,
    qc_class: 0,
    afc_ok: false,
    fcp: Err(0x41),
    arm: Ok(()),
    status: [0; 20],
    caps: Vec::new(),
    ticks: 0,
};

static SIM: Mutex<Sim> = Mutex::new(DEFAULT);

pub fn set(s: Sim) {
    *SIM.lock().unwrap() = s;
}

pub fn get() -> Sim {
    SIM.lock().unwrap().clone()
}

pub fn advance(ticks: u32) {
    let mut s = SIM.lock().unwrap();
    s.ticks = s.ticks.wrapping_add(ticks);
}

#[allow(dead_code)]
pub mod pd {
    pub const MODE_OFF: u8 = 0;
    pub const MODE_SINK: u8 = 1;
    pub const MODE_SNIFF: u8 = 2;
    pub struct Link {
        pub mode: u8,
        pub src: bool,
        pub snk: bool,
        pub cc: u8,
        pub cc_mv: u16,
    }
    pub fn link() -> Link {
        let s = super::get();
        Link { mode: s.mode, src: s.src, snk: s.snk, cc: s.cc, cc_mv: s.cc_mv }
    }
    pub fn is_off() -> bool {
        super::get().mode == MODE_OFF
    }
    pub fn off() -> Result<(), u8> {
        Ok(())
    }
    pub fn arm() -> Result<(), u8> {
        super::get().arm
    }
    pub fn status(out: &mut [u8; 20]) {
        *out = super::get().status;
    }
    pub fn caps(out: &mut [u8; 28]) -> u8 {
        let c = super::get().caps;
        for (i, p) in c.iter().enumerate() {
            out[4 * i..4 * i + 4].copy_from_slice(&p.to_le_bytes());
        }
        c.len() as u8
    }
}

pub mod ina {
    pub struct Meas {
        pub v_1e4: u32,
        pub i_1e5: u32,
        pub dp_1e2: u32,
        pub dm_1e2: u32,
    }
    pub fn snapshot_vi() -> (u32, u32) {
        let s = super::get();
        (s.v_1e4, s.i_1e5)
    }
    pub fn snapshot() -> Meas {
        let s = super::get();
        Meas { v_1e4: s.v_1e4, i_1e5: s.i_1e5, dp_1e2: s.dp_1e2, dm_1e2: s.dm_1e2 }
    }
    pub fn ticks() -> u32 {
        super::get().ticks
    }
}

#[allow(dead_code)]
pub mod qc {
    pub fn off() -> Result<(), u8> {
        Ok(())
    }
    pub fn hvdcp_begin() {}
    pub fn line_mv(_ch: u8) -> u16 {
        0
    }
    pub fn hvdcp_class(_early: u16, _late: u16) -> u16 {
        super::get().qc_class
    }
    pub fn accp_release() {}
    pub fn accp_hold() {}
    pub fn accp_entered() {}
    pub fn afc(_cmd: u8, _reps: u8) -> Result<(), u8> {
        if super::get().afc_ok {
            Ok(())
        } else {
            Err(0x40)
        }
    }
    pub fn fcp_read(_reg: u8, out: &mut [u8]) -> Result<(), u8> {
        let r = super::get().fcp?;
        out.copy_from_slice(&r);
        Ok(())
    }
}

pub mod pdscr {
    pub fn top() {}
    pub fn page() -> (u8, u8) {
        (0, 1)
    }
    pub fn next_page() {}
}
