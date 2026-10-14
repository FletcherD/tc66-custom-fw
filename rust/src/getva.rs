//! Real `getva` reply builder (M1).
//!
//! Wire format (docs/HARDWARE.md): 192-B plaintext = 3 x 64-B blocks pac1/pac2/pac3,
//! little-endian u32 fields, **CRC16/MODBUS of bytes 0..60 stored LE at 60..64
//! of each block** (verified against the known-good static packet), then
//! AES-256-ECB encrypted with the meter's fixed key.
//!
//! TEMPLATE below is the decrypted known-good static packet (fw "1.18");
//! build() patches every live field (serial, run count, V/I/P/R, both
//! energy groups, temp, D±, seq) and recomputes checksums.

use crate::aes256;

/* compile-time key schedule, next to the cipher in fast flash */
#[link_section = ".fasttext.getva_rk"]
static RK: aes256::RoundKeys = aes256::expand_key(&KEY);

const KEY: [u8; 32] = [
    0x58, 0x21, 0xfa, 0x56, 0x01, 0xb2, 0xf0, 0x26, 0x87, 0xff, 0x12, 0x04,
    0x62, 0x2a, 0x4f, 0xb0, 0x86, 0xf4, 0x02, 0x60, 0x81, 0x6f, 0x9a, 0x0b,
    0xa7, 0xf1, 0x06, 0x61, 0x9a, 0xb8, 0x72, 0x88,
];

const TEMPLATE: [u8; 192] = [
    0x70, 0x61, 0x63, 0x31, 0x54, 0x43, 0x36, 0x36, 0x31, 0x2e, 0x31, 0x38,
    0xee, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x50, 0xc3, 0x00, 0x00, 0x50, 0xc3, 0x00, 0x00, 0xa8, 0x61, 0x00, 0x00,
    0x6c, 0xb5, 0x00, 0x00, 0x70, 0x61, 0x63, 0x32, 0x64, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x00, 0x00, 0x00,
    0x49, 0x01, 0x00, 0x00, 0x49, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x7a, 0x82, 0x00, 0x00, 0x70, 0x61, 0x63, 0x33,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x05, 0x00, 0x00,
];

pub fn crc16_modbus(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        crc ^= b as u16;
        for _ in 0..8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ 0xA001 } else { crc >> 1 };
        }
    }
    crc
}

fn put_u32(buf: &mut [u8; 192], off: usize, v: u32) {
    buf[off..off + 4].copy_from_slice(&v.to_le_bytes());
}

/// Live fields in wire units: v = 1e-4 V, i = 1e-5 A, p = 1e-4 W,
/// r = 1e-1 ohm, energy in whole mAh/mWh, temp in deg C, d± in 1e-2 V.
pub struct Fields {
    pub v: u32,
    pub i: u32,
    pub p: u32,
    pub r: u32,
    /// active energy group (rawva)
    pub mah: u32,
    pub mwh: u32,
    /// both groups: g0 mAh, g0 mWh, g1 mAh, g1 mWh (getva 72..88)
    pub groups: [u32; 4],
    /// per-unit serial (flash config page) and boot count (EEPROM 0x202c)
    pub serial: u32,
    pub runs: u32,
    pub temp_c: i32,
    pub dp: u32,
    pub dm: u32,
    /// sampler window counter (our extension, pac3+4; stock apps ignore pac3)
    pub seq: u32,
}

/// `rawva` reply: the same live fields as getva, plaintext, no AES. All LE u32 unless noted:
///   0 magic "raw1", 4 seq, 8 v_1e4, 12 i_1e5, 16 p_1e4, 20 r_1e1,
///   24 mah, 28 mwh, 32 temp_c (i32), 36 dp_1e2, 40 dm_1e2,
///   44 CRC16/MODBUS of bytes 0..44 (LE u16). 46 B total.
pub const RAW_LEN: usize = 46;

#[inline(never)]
pub fn build_raw(out: &mut [u8; RAW_LEN], f: &Fields) {
    out[..4].copy_from_slice(b"raw1");
    for (i, v) in [
        f.seq, f.v, f.i, f.p, f.r, f.mah, f.mwh, f.temp_c as u32, f.dp, f.dm,
    ]
    .iter()
    .enumerate()
    {
        out[4 + 4 * i..8 + 4 * i].copy_from_slice(&v.to_le_bytes());
    }
    let crc = crc16_modbus(&out[..44]);
    out[44..46].copy_from_slice(&crc.to_le_bytes());
}

/// Fill the template with live values, checksum, encrypt.
#[inline(never)]
pub fn build(out: &mut [u8; 192], f: &Fields) {
    *out = TEMPLATE;
    put_u32(out, 48, f.v);
    put_u32(out, 52, f.i);
    put_u32(out, 56, f.p);
    put_u32(out, 68, f.r);
    put_u32(out, 12, f.serial);
    put_u32(out, 44, f.runs);
    for (k, &g) in f.groups.iter().enumerate() {
        put_u32(out, 72 + 4 * k, g);
    }
    put_u32(out, 88, if f.temp_c < 0 { 1 } else { 0 }); /* temp sign */
    put_u32(out, 92, f.temp_c.unsigned_abs());
    put_u32(out, 96, f.dp);
    put_u32(out, 100, f.dm);
    put_u32(out, 132, f.seq);
    for b in 0..3 {
        let crc = crc16_modbus(&out[b * 64..b * 64 + 60]) as u32;
        let bytes = crc.to_le_bytes();
        out[b * 64 + 60..b * 64 + 64].copy_from_slice(&bytes);
    }
    for blk in out.chunks_exact_mut(16) {
        if let Ok(blk) = blk.try_into() {
            aes256::encrypt_block(&RK, blk);
        }
    }
}
