//! Screen text of the PD (pdinfo.rs) and detection (detect.rs) screens: the
//! bounded line writer `W` and the packed string table its `t` writes from.
//!
//! Every screen string is an entry of `TEXT` (length-prefixed, packed at
//! compile time) written by its id: `w.t(VBUS)`. A call site then loads an
//! 8-bit immediate instead of a 32-bit address plus a length — about 8 B
//! less per string on Thumb-2. Ids computed as `FIRST + k` rely on the
//! entries being consecutive in the order listed (marked "in order").

/// Text columns per line (the PD screen's width in 7-px cells).
pub const COLS: usize = 22;

macro_rules! table {
    ($($id:ident = $s:literal,)*) => {
        #[allow(non_camel_case_types, clippy::upper_case_acronyms)]
        #[repr(u8)]
        enum Id { $($id,)* }
        $(#[allow(dead_code)] pub const $id: u8 = Id::$id as u8;)*
        const LIST: &[&[u8]] = &[$($s,)*];
    };
}

table! {
    /* shared */
    VBUS = b"VBUS ",
    MA = b"mA",
    CC = b"CC",
    PD10 = b"PD1.0", /* in order: header spec rev 0, 1, 2 */
    PD20 = b"PD2.0",
    PD3X = b"PD3.x",

    /* PD screen (pdinfo.rs) */
    PD_OFF = b"PD off   K2: sniff",
    SNIFF = b"SNIFF ",
    SINK = b"SINK  ",
    NO_REV = b"     ",
    SRC = b" src",
    SNK = b" snk",
    NO_CONTRACT = b"no contract",
    V_SP = b"V ",
    CC_NO_SOURCE = b"CC  no source",
    RP_USB = b" Rp USB", /* in order: Rp from the CC voltage */
    RP_1A5 = b" Rp 1.5A",
    RP_3A = b" Rp 3.0A",
    OPEN = b" open",
    NO_EMARKER = b"no e-marker seen",
    CBL = b"CBL ",
    CBL_NO_A = b"--",
    CBL_3A = b"3A",
    CBL_5A = b"5A",
    USB2 = b" USB2", /* in order: cable VDO speed field 0..4, other */
    GEN1 = b" Gen1",
    GEN2 = b" Gen2",
    GEN3 = b" Gen3",
    GEN4 = b" Gen4",
    SPEED_UNKNOWN = b" ?",
    MSG = b"msg ",
    CRC = b" crc ",
    HR = b" hr ",
    NO_CAPS = b"no source caps",
    VAR = b"VAR ",
    BAT = b"BAT ",
    PPS = b"PPS ",
    AVS = b"AVS ",
    AVS_SPR = b"AVS 9-20V ",
    APDO = b"APDO ",
    UNKNOWN_PDO = b"? ",

    /* detection screen (detect.rs) */
    REFUSED = b"refused: ",
    REF_SINK_ARMED = b"PD sink on",
    REF_SINK_SEEN = b"sink on port",
    REF_LOAD = b"load ",
    DETECTING = b"detecting: ",
    STEP_DPDM = b"D+/D-",
    STEP_QC = b"QC",
    STEP_AFC = b"AFC",
    STEP_FCP = b"FCP",
    DET_DONE = b"charger detect done",
    DET_START = b"K2: detect charger",
    FOUND_APPLE = b"Apple ", /* in order: detect::line's `found` flags */
    FOUND_SAMSUNG = b"Samsung ",
    FOUND_QC = b"QC ",
    FOUND_DCP = b"DCP ",
    FOUND_AFC = b"AFC ",
    FOUND_FCP = b"FCP ",
    FOUND_PD = b"PD",
    PLAIN_USB = b"plain USB only",
    LOAD = b" load ",
    NO_DIVIDER = b"no D+/D- divider",
    SAMSUNG_DIV = b"Samsung divider 2A",
    TIED = b"D+=D- tied, biased",
    APPLE_DIV = b"Apple divider ",
    APPLE_0A5 = b"0.5A", /* in order: Apple divider 1..4 */
    APPLE_1A = b"1A",
    APPLE_2A1 = b"2.1A",
    APPLE_2A4 = b"2.4A",
    QC_NA = b"QC n/a (divider)",
    QC_NO_SHORT = b"no D+/D- short", /* in order: qc::hvdcp_class 0, 1, 2 */
    QC_DCP = b"DCP, no QC",
    QC_HVDCP = b"QC (HVDCP)",
    AFC_YES = b"AFC answered",
    AFC_NO = b"AFC no answer",
    RAISED = b" raised, ",
    FCP_NO_REPLY = b"FCP no reply",
    FCP_NACK = b"FCP NACK",
    PD_ERROR = b"PD error ",
    NO_TYPEC = b"no Type-C source",
    BC_UNKNOWN = b" Rp ?    ", /* in order: Status0 BC_LVL 0..3 */
    BC_USB = b" Rp USB  ",
    BC_1A5 = b" Rp 1.5A ",
    BC_3A = b" Rp 3.0A ",
    NO_PD = b"no PD",
    AGAIN = b"K2: run again",
}

const fn packed_len(list: &[&[u8]]) -> usize {
    let mut n = 0;
    let mut i = 0;
    while i < list.len() {
        n += 1 + list[i].len();
        i += 1;
    }
    n
}

const fn pack<const N: usize>(list: &[&[u8]]) -> [u8; N] {
    assert!(list.len() <= 256, "ids are u8");
    let mut out = [0u8; N];
    let mut p = 0;
    let mut i = 0;
    while i < list.len() {
        let s = list[i];
        assert!(s.len() <= COLS);
        out[p] = s.len() as u8;
        p += 1;
        let mut j = 0;
        while j < s.len() {
            out[p] = s[j];
            p += 1;
            j += 1;
        }
        i += 1;
    }
    out
}

static TEXT: [u8; packed_len(LIST)] = pack(LIST);

/// Bounded line writer: characters past COLS are dropped.
pub struct W {
    pub b: [u8; COLS],
    n: usize,
}

impl W {
    pub fn new() -> W {
        W { b: [b' '; COLS], n: 0 }
    }

    #[inline(never)]
    pub fn ch(&mut self, c: u8) {
        if let Some(x) = self.b.get_mut(self.n) {
            *x = c;
        }
        self.n += 1;
    }

    /// Table entry `id` (nothing for an id past the table).
    #[inline(never)]
    pub fn t(&mut self, id: u8) {
        let mut id = id;
        let mut it = TEXT.iter();
        while let Some(&n) = it.next() {
            for &c in it.by_ref().take(n as usize) {
                if id == 0 {
                    self.ch(c);
                }
            }
            if id == 0 {
                return;
            }
            id -= 1;
        }
    }

    /// Decimal, right-aligned in at least `width` chars.
    #[inline(never)]
    pub fn uint(&mut self, v: u32, width: usize) {
        let mut tmp = [0u8; 10];
        let mut k = 0;
        let mut v = v;
        for t in tmp.iter_mut() {
            *t = b'0' + (v % 10) as u8;
            k += 1;
            v /= 10;
            if v == 0 {
                break;
            }
        }
        for _ in k..width {
            self.ch(b' ');
        }
        for &t in tmp.iter().take(k).rev() {
            self.ch(t);
        }
    }

    /// `v` in units of 10^-frac (frac 1..=3), right-aligned in `width`.
    #[inline(never)]
    pub fn fixed(&mut self, v: u32, frac: usize, width: usize) {
        let mut d = [0u8; 3];
        let mut v = v;
        for x in d.iter_mut().take(frac) {
            *x = b'0' + (v % 10) as u8;
            v /= 10;
        }
        self.uint(v, width.saturating_sub(frac + 1));
        self.ch(b'.');
        for &x in d.iter().take(frac).rev() {
            self.ch(x);
        }
    }

    #[inline(never)]
    pub fn hex16(&mut self, v: u16) {
        for sh in [12, 8, 4, 0] {
            self.ch(b"0123456789ABCDEF"[(v >> sh & 0xF) as usize]);
        }
    }

    /// mV as "x.xxV" (2 decimals)
    #[inline(never)]
    pub fn volts(&mut self, mv: u32, width: usize) {
        self.fixed(mv / 10, 2, width);
        self.ch(b'V');
    }

    /// mA as "x.xxA"
    #[inline(never)]
    pub fn amps(&mut self, ma: u32) {
        self.fixed(ma / 10, 2, 0);
        self.ch(b'A');
    }

    /// A PD header's spec revision field.
    pub fn rev(&mut self, rev: u8) {
        self.t(PD10 + rev.min(2));
    }
}
