//! Host-side proofs for the firmware's pure modules, compiled here
//! verbatim from rust/src:
//! - ntc.rs: fixed-point NTC conversion bit-identical to the stock f32 model
//!   over every possible ADC sum16 (also reported against exact f64).
//! - aes256.rs: S-box = the GF(2^8) definition, FIPS-197 C.3 vector, and
//!   random blocks under the meter's getva key vs the RustCrypto `aes` crate.
//! - pdinfo.rs: the PD parser + screen text replayed over a negotiation
//!   (Source_Cap, Request, PS_RDY, PPS, Reject, cable identity, hard reset);
//!   the frames' meaning cross-checked with tools/pd_decode.py.
//! - pdinfo.rs + detect.rs: every screen line kind (text + color) over a
//!   scripted set of states, compared with text.golden (textdump.rs;
//!   `cargo run --release -- --bless` rewrites it after an intended change).

#[path = "../../../rust/src/ntc.rs"]
mod ntc;
#[allow(dead_code)]
#[path = "../../../rust/src/aes256.rs"]
mod aes256;
#[path = "../../../rust/src/text.rs"]
mod text;
#[path = "../../../rust/src/pdinfo.rs"]
mod pdinfo;
#[allow(dead_code)]
#[path = "../../../rust/src/pdspec.rs"]
mod pdspec;
#[allow(dead_code)]
#[path = "../../../rust/src/pdsta.rs"]
mod pdsta;

#[allow(dead_code)]
#[path = "../../../rust/src/detect.rs"]
mod detect;
mod sim;
mod textdump;
use sim::{ina, pd, pdscr, qc};

use aes::cipher::{BlockEncrypt, KeyInit};

/// The stock model in f32 + libm logf, as the firmware computed it before
/// the fixed-point port.
fn stock_f32(sum16: u32) -> i32 {
    let avg = sum16 as f32 / 16.0;
    if avg < 1.0 || avg > 4094.0 {
        return 0;
    }
    let r = avg * 11500.0 / (4095.0 - avg);
    let inv_t = 1.0 / 298.15 + libm::logf(r / 10_000.0) / 3450.0;
    (1.0 / inv_t - 273.15) as i32
}

fn exact(sum16: u32) -> f64 {
    let avg = sum16 as f64 / 16.0;
    if !(1.0..=4094.0).contains(&avg) {
        return 0.0;
    }
    let r = avg * 11500.0 / (4095.0 - avg);
    1.0 / (1.0 / 298.15 + (r / 10_000.0).ln() / 3450.0) - 273.15
}

fn check_ntc() -> bool {
    let (mut vs_stock, mut vs_exact) = (0u32, 0u32);
    for s in 0..=65535u32 {
        let v = ntc::temp_c(s);
        if v != stock_f32(s) {
            vs_stock += 1;
            println!("  sum16={s}: fixed={v} stock_f32={} exact={:.6}", stock_f32(s), exact(s));
        }
        if v as f64 != exact(s).trunc() {
            vs_exact += 1;
        }
    }
    println!("ntc: sum16 0..=65535 ({}..={} C): {vs_stock} differ from stock f32, {vs_exact} from trunc(exact)",
             ntc::temp_c(65504), ntc::temp_c(16));
    vs_stock == 0
}

fn gmul(mut a: u8, mut b: u8) -> u8 {
    let mut p = 0;
    while b != 0 {
        if b & 1 != 0 { p ^= a; }
        a = (a << 1) ^ if a & 0x80 != 0 { 0x1b } else { 0 };
        b >>= 1;
    }
    p
}

fn sbox_ref(x: u8) -> u8 {
    let inv = if x == 0 { 0 } else { (1..=255u8).find(|&y| gmul(x, y) == 1).unwrap() };
    inv ^ inv.rotate_left(1) ^ inv.rotate_left(2) ^ inv.rotate_left(3) ^ inv.rotate_left(4) ^ 0x63
}

/// The meter's getva key (rust/src/getva.rs KEY).
const GETVA_KEY: [u8; 32] = [
    0x58, 0x21, 0xfa, 0x56, 0x01, 0xb2, 0xf0, 0x26, 0x87, 0xff, 0x12, 0x04,
    0x62, 0x2a, 0x4f, 0xb0, 0x86, 0xf4, 0x02, 0x60, 0x81, 0x6f, 0x9a, 0x0b,
    0xa7, 0xf1, 0x06, 0x61, 0x9a, 0xb8, 0x72, 0x88,
];

fn check_aes() -> bool {
    let mut ok = true;
    let bad_sbox = (0..=255u8).filter(|&x| aes256::sbox(x) != sbox_ref(x)).count();
    println!("aes: S-box entries wrong: {bad_sbox}");
    ok &= bad_sbox == 0;

    /* FIPS-197 Appendix C.3 */
    let key: [u8; 32] = core::array::from_fn(|i| i as u8);
    let mut blk: [u8; 16] = core::array::from_fn(|i| (i as u8) * 0x11);
    aes256::encrypt_block(&aes256::expand_key(&key), &mut blk);
    let want = [0x8e, 0xa2, 0xb7, 0xca, 0x51, 0x67, 0x45, 0xbf, 0xea, 0xfc, 0x49, 0x90, 0x4b, 0x49, 0x60, 0x89];
    println!("aes: FIPS-197 C.3 vector {}", if blk == want { "ok" } else { "WRONG" });
    ok &= blk == want;

    let rk = aes256::expand_key(&GETVA_KEY);
    let reference = aes::Aes256Enc::new(&GETVA_KEY.into());
    let mut x = 0x2545_f491_4f6c_dd1du64; /* xorshift, fixed seed */
    let mut bad = 0;
    for _ in 0..100_000 {
        let mut b = [0u8; 16];
        for chunk in b.chunks_mut(8) {
            x ^= x << 13; x ^= x >> 7; x ^= x << 17;
            chunk.copy_from_slice(&x.to_le_bytes());
        }
        let mut r = b.into();
        reference.encrypt_block(&mut r);
        aes256::encrypt_block(&rk, &mut b);
        bad += (b[..] != r[..]) as u32;
    }
    println!("aes: 100000 random blocks under the getva key: {bad} differ from the aes crate");
    ok & (bad == 0)
}

fn pd_text() -> Vec<String> {
    (0..pdinfo::lines())
        .map(|i| {
            let mut l = [b' '; text::COLS];
            pdinfo::line(i, &mut l);
            String::from_utf8_lossy(&l).trim_end().to_string()
        })
        .collect()
}

fn pd_expect(step: &str, want: &[&str]) -> bool {
    let got = pd_text();
    let ok = got == want;
    println!("pdinfo: {step}: {}", if ok { "ok" } else { "WRONG" });
    if !ok {
        for (i, (g, w)) in got.iter().zip(want.iter().chain(std::iter::repeat(&"<none>"))).enumerate() {
            println!("  {i:2} got {g:<22} want {w}");
        }
    }
    ok
}

fn pd_msg(hdr: u16, objs: &[u32], sop: u8) {
    let b: Vec<u8> = objs.iter().flat_map(|o| o.to_le_bytes()).collect();
    pdinfo::on_msg(hdr, &b, sop);
}

/// Apple-style source: 5/9/15/20 V 3 A + PPS 3.3-11 V 3 A, PD 3.0 headers;
/// header = type | rev 2 << 6 | power role (source) 1 << 8 | nobj << 12.
fn check_pdinfo() -> bool {
    const CAPS: [u32; 5] = [0x0801_912C, 0x0002_D12C, 0x0004_B12C, 0x0006_412C, 0xC0DC_213C];
    const SRC_CAP: u16 = 0x51A1; /* data 1, DFP, 5 objects */
    const REQUEST: u16 = 0x1082; /* data 2 from the sink */
    const GOODCRC: u16 = 0x0081;
    const PS_RDY: u16 = 0x0186;
    const REJECT: u16 = 0x0184;
    const SOP1_ID_ACK: u16 = 0x518F; /* SOP' VDM from the cable plug, 5 objects */
    let mut ok = true;
    pdinfo::reset();
    ok &= pd_expect("idle", &[
        "SNIFF       src snk", "no contract", "VBUS 9.021V 1.234A", "CC1 1.68V Rp 3.0A",
        "no e-marker seen", "msg 0 crc 0 hr 0", "no source caps",
    ]);
    pd_msg(SRC_CAP, &CAPS, 0);
    pd_msg(SRC_CAP, &CAPS, 0); /* retry: overwrites */
    pd_msg(GOODCRC, &[], 0);
    pd_msg(REQUEST, &[0x2003_212C], 0); /* PDO 2, 2 A op / 3 A max */
    ok &= pd_expect("caps + request (Accept missed)", &[
        "SNIFF PD3.x src snk", "?#2 9.00V 2.00A", "VBUS 9.021V 1.234A", "CC1 1.68V Rp 3.0A",
        "no e-marker seen", "msg 4 crc 0 hr 0",
        " 1  5.00V 3.00A", " 2  9.00V 3.00A", " 3 15.00V 3.00A", " 4 20.00V 3.00A",
        " 5 PPS 3.3-11.0V 3.00A",
    ]);
    pd_msg(PS_RDY, &[], 0);
    let contract9 = [
        "SNIFF PD3.x src snk", ">#2 9.00V 2.00A", "VBUS 9.021V 1.234A", "CC1 1.68V Rp 3.0A",
        "no e-marker seen", "msg 5 crc 0 hr 0",
        " 1  5.00V 3.00A", ">2  9.00V 3.00A", " 3 15.00V 3.00A", " 4 20.00V 3.00A",
        " 5 PPS 3.3-11.0V 3.00A",
    ];
    ok &= pd_expect("PS_RDY -> contract", &contract9);
    pd_msg(REQUEST, &[0x5003_8628], 0); /* PPS 9.02 V 2 A */
    pd_msg(REJECT, &[], 0);
    pdinfo::on_event(0x32);
    let mut want = contract9;
    want[5] = "msg 7 crc 1 hr 0";
    ok &= pd_expect("PPS request rejected", &want);
    pd_msg(REQUEST, &[0x5003_8628], 0);
    pd_msg(PS_RDY, &[], 0);
    pd_msg(SOP1_ID_ACK, &[0xFF00_A041, 0x1800_05AC, 0, 0x1781_0000, 0x0008_0040], 1);
    ok &= pd_expect("PPS contract + cable", &[
        "SNIFF PD3.x src snk", ">#5 PPS 9.02V 2.00A", "VBUS 9.021V 1.234A", "CC1 1.68V Rp 3.0A",
        "CBL 5A 20V USB2 05AC", "msg 10 crc 1 hr 0",
        " 1  5.00V 3.00A", " 2  9.00V 3.00A", " 3 15.00V 3.00A", " 4 20.00V 3.00A",
        ">5 PPS 3.3-11.0V 3.00A",
    ]);
    pdinfo::on_event(0x33);
    ok &= pd_expect("hard reset", &[
        "SNIFF       src snk", "no contract", "VBUS 9.021V 1.234A", "CC1 1.68V Rp 3.0A",
        "CBL 5A 20V USB2 05AC", "msg 10 crc 1 hr 1", "no source caps",
    ]);
    pdinfo::source_gone();
    ok &= pd_expect("source gone", &[
        "SNIFF       src snk", "no contract", "VBUS 9.021V 1.234A", "CC1 1.68V Rp 3.0A",
        "no e-marker seen", "msg 10 crc 1 hr 1", "no source caps",
    ]);
    ok
}

fn main() {
    if std::env::args().any(|a| a == "--bless") {
        textdump::bless();
        return;
    }
    let ok = check_ntc() & check_aes() & check_pdinfo() & textdump::check();
    std::process::exit(!ok as i32);
}
