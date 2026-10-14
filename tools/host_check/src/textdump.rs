//! Every PD-screen (pdinfo.rs) and detection-screen (detect.rs) line kind,
//! text and color, over a scripted set of states — compared byte for byte
//! with text.golden. The golden file is the spec of what the screens show:
//! after an intended text change, `cargo run --release -- --bless` rewrites
//! it and the diff is reviewed with the change.

use crate::sim::{self, Sim, DEFAULT};
use crate::{detect, pd, pdinfo, text};
use std::fmt::Write;

const GOLDEN: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/text.golden");

fn dump_lines(out: &mut String, title: &str, n: usize, line: fn(usize, &mut [u8; text::COLS]) -> u16) {
    writeln!(out, "## {title}").unwrap();
    /* one line past the end: must come back blank with color 0 */
    for i in 0..=n {
        let mut l = [b'#'; text::COLS];
        let c = line(i, &mut l);
        writeln!(out, "{c:04X} |{}|", String::from_utf8_lossy(&l)).unwrap();
    }
}

fn pd_dump(out: &mut String, title: &str) {
    dump_lines(out, &format!("pdinfo: {title}"), pdinfo::lines(), pdinfo::line);
}

fn det_dump(out: &mut String, title: &str) {
    dump_lines(out, &format!("detect: {title}"), detect::lines(), detect::line);
}

fn msg(hdr: u16, objs: &[u32], sop: u8) {
    let b: Vec<u8> = objs.iter().flat_map(|o| o.to_le_bytes()).collect();
    pdinfo::on_msg(hdr, &b, sop);
}

/* Source_Capabilities header: type 1 | rev << 6 | source << 8 | n << 12 */
fn src_cap(rev: u16, caps: &[u32]) {
    msg(1 | rev << 6 | 1 << 8 | (caps.len() as u16) << 12, caps, 0);
}
const REQUEST: u16 = 0x1082;
const PS_RDY: u16 = 0x0186;

/// Fixed 5 V 3 A, VAR 5-12 V 2 A, BAT 5-20 V 60 W, PPS 3.3-11 V 3 A, SPR AVS
/// 3 A / 2.25 A, an EPR-AVS-type APDO, fixed 9 V 3 A.
const ALL_CAPS: [u32; 7] = [
    0x0801_912C,
    2 << 30 | 240 << 20 | 100 << 10 | 200, /* PD Table 6.6: 10b variable */
    1 << 30 | 400 << 20 | 100 << 10 | 240, /* 01b battery */
    0xC0DC_213C,
    3 << 30 | 2 << 28 | 300 << 10 | 225,
    3 << 30 | 1 << 28 | 0x012_3456,
    0x0002_D12C,
];

fn pd_cable(rev: u16, vdo: u32) {
    msg(0x510F | rev << 6, &[0xFF00_A041, 0x1800_05AC, 0, 0x1781_0000, vdo], 1);
}

fn pdinfo_all(out: &mut String) {
    sim::set(DEFAULT);
    pdinfo::reset();
    pd_dump(out, "reset, sniff, sink on CC1");
    for (name, mode, src, snk, cc, cc_mv) in [
        ("off", pd::MODE_OFF, false, false, 0, 0),
        ("sniff, no source", pd::MODE_SNIFF, false, false, 0, 0),
        ("sniff, open CC2", pd::MODE_SNIFF, true, false, 2, 2700),
        ("sniff, Rp USB", pd::MODE_SNIFF, true, true, 1, 659),
        ("sniff, Rp 1.5A", pd::MODE_SNIFF, true, true, 1, 660),
        ("sniff, Rp 3.0A", pd::MODE_SNIFF, true, true, 1, 1230),
        ("sink", pd::MODE_SINK, true, true, 1, 1570),
    ] {
        sim::set(Sim { mode, src, snk, cc, cc_mv, ..DEFAULT });
        pd_dump(out, name);
    }
    sim::set(DEFAULT);
    for rev in 0..3 {
        src_cap(rev, &ALL_CAPS);
        pd_dump(out, &format!("all PDO kinds, caps rev {rev}"));
    }
    for (pos, rdo) in [
        (1, 0x0004_B12C),                     /* fixed, 3 A op */
        (2, 0x0003_2096),                     /* VAR, 2 A op */
        (3, 0x0003_C0F0),                     /* BAT, 60 W op */
        (4, 0x0003_843C),                     /* PPS 9.00 V 3 A */
        (5, 0x0004_B028),                     /* AVS 15.00 V 2 A */
        (6, 0x0005_A028),                     /* other APDO */
        (8, 0x0004_B12C),                     /* not among the caps */
    ] {
        msg(REQUEST, &[(pos as u32) << 28 | rdo], 0);
        pd_dump(out, &format!("request {pos} pending"));
        msg(PS_RDY, &[], 0);
        pd_dump(out, &format!("contract {pos}"));
    }
    for (rev, vdo) in [(1, 0x0000_0020), (2, 0x0000_0241), (2, 0x0000_0442), (2, 0x0000_0663), (2, 0x0000_0004), (2, 0x0000_0047)] {
        pd_cable(rev, vdo);
        pd_dump(out, &format!("cable rev {rev} vdo {vdo:08X}"));
    }
    for _ in 0..1000 {
        pdinfo::on_event(0x32);
    }
    for _ in 0..100 {
        pdinfo::on_event(0x33);
    }
    for _ in 0..10_000 {
        msg(PS_RDY, &[], 0);
    }
    pd_dump(out, "counters clamped, after hard resets");
    pdinfo::source_gone();
    pd_dump(out, "source gone");
}

/// Run detection to the end, dumping the screen whenever the head line
/// changes (the running steps) and once at the end.
fn det_run(out: &mut String, title: &str, s: Sim) {
    sim::set(s);
    let r = detect::start();
    writeln!(out, "# {title}: start {r:?}").unwrap();
    let mut last = [0u8; text::COLS];
    for _ in 0..100 {
        let mut head = [0u8; text::COLS];
        detect::line(0, &mut head);
        if head != last {
            last = head;
            det_dump(out, &format!("{title}, running"));
        }
        if !detect::busy() {
            break;
        }
        sim::advance(1100);
        detect::service();
    }
    det_dump(out, &format!("{title}, done"));
}

/// Status0 BC_LVL, contract flag, rev for the PD step: `pdsta` layout.
fn status(state: u8, contract: u8, status0: u8, rev: u8) -> [u8; 22] {
    let mut p = [0u8; 22];
    p[0] = state;
    p[1] = 1;
    p[2] = contract;
    p[7] = status0;
    p[13] = rev;
    p
}

fn detect_all(out: &mut String) {
    let idle = Sim { mode: pd::MODE_OFF, src: false, snk: false, cc: 0, i_1e5: 1_000, v_1e4: 51_600, ..DEFAULT };
    sim::set(idle.clone());
    det_dump(out, "never run");
    det_run(out, "refused: sink armed", Sim { mode: pd::MODE_SINK, ..idle.clone() });
    det_run(out, "refused: sink seen", Sim { mode: pd::MODE_SNIFF, snk: true, ..idle.clone() });
    det_run(out, "refused: load", Sim { i_1e5: 4_500, ..idle.clone() });

    /* load appears between start and the passive step */
    sim::set(idle.clone());
    let r = detect::start();
    sim::set(Sim { i_1e5: 3_300, ..idle.clone() });
    for _ in 0..3 {
        sim::advance(1100);
        detect::service();
    }
    writeln!(out, "# load during passive: start {r:?}").unwrap();
    det_dump(out, "load during passive");

    let four = ALL_CAPS[..1].iter().chain(&[0x0002_D12C, 0x0004_B12C, 0x0006_41D6]).copied().collect::<Vec<_>>();
    for (i, (dp, dm)) in [(200, 200), (200, 270), (270, 200), (270, 270)].into_iter().enumerate() {
        det_run(out, &format!("Apple {i}"), Sim { dp_1e2: dp, dm_1e2: dm, qc_class: 2, arm: Err(3), ..idle.clone() });
    }
    det_run(out, "Samsung, AFC, FCP, no source", Sim {
        dp_1e2: 120, dm_1e2: 120, qc_class: 2, afc_ok: true, fcp: Ok([2, 36]),
        status: status(1, 0, 0, 0), ..idle.clone()
    });
    det_run(out, "tied, QC, PD 3.0 Rp 3A", Sim {
        dp_1e2: 60, dm_1e2: 60, qc_class: 2, fcp: Err(0x21),
        status: status(5, 1, 3, 2), caps: four.clone(), ..idle.clone()
    });
    det_run(out, "DCP, Rp USB, no PD", Sim {
        qc_class: 1, status: status(5, 0, 1, 1), ..idle.clone()
    });
    det_run(out, "QC, Rp 1.5A, PD 1.0, all PDO kinds", Sim {
        qc_class: 2, afc_ok: true, status: status(5, 1, 2, 0), caps: ALL_CAPS.to_vec(), ..idle.clone()
    });
    det_run(out, "Rp unknown (CC activity)", Sim {
        status: status(5, 1, 0x43, 2), caps: four, ..idle.clone()
    });
    det_run(out, "plain USB", Sim { status: status(1, 0, 0, 0), ..idle });
}

fn render() -> String {
    let mut out = String::new();
    pdinfo_all(&mut out);
    detect_all(&mut out);
    out
}

pub fn bless() {
    std::fs::write(GOLDEN, render()).unwrap();
    println!("text: wrote {GOLDEN}");
}

pub fn check() -> bool {
    let got = render();
    let want = std::fs::read_to_string(GOLDEN).unwrap_or_default();
    if got == want {
        println!("text: {} screen lines identical to text.golden", got.lines().filter(|l| !l.starts_with('#')).count());
        return true;
    }
    println!("text: differs from text.golden:");
    for (i, (g, w)) in got.lines().zip(want.lines().chain(std::iter::repeat(""))).enumerate() {
        if g != w {
            println!("  line {}: got  {g}\n  {:>w$}want {w}", i + 1, "", w = 6 + (i + 1).to_string().len());
        }
    }
    if want.lines().count() > got.lines().count() {
        println!("  ({} golden lines missing)", want.lines().count() - got.lines().count());
    }
    false
}
