# tools/

Host-side Python for the TC66 custom firmware. Run any script with
`uv run tools/<name>.py --help` (uv scripts declare their own dependencies).

- **`tools/*.py` — for users of the meter:**
  - `tc66flash.py`: flash a release, go back to stock, back up the EEPROM (standalone).
  - `detect.py`: charger protocol detection at 5 V.
  - `pd.py`: PD sink / sniffer control, voltage requests, status, the PD screen text.
  - `pdcap.py`: live PD capture with decoding (needs `udev/70-tc66.rules`); starts the sniffer
    if the PD engine is off; `--save` / `--replay`.
  - `pdseq.py`: a capture around a sequence of `pd.py` steps.
  - `qc.py`: Quick Charge triggers.
  - `rawva.py` / `getva.py`: measurements.
  - `gtrec.py`: the on-meter datalog.
  - `cfg.py`: settings.
  - `eedump.py`: EEPROM dump / decode.
  - `ble.py`: readings over Bluetooth.
- **`tools/dev/` — firmware development and bench diagnostics:**
  - build gates (`stack_gate.py`, `fast_gate.py`), the flash / release plumbing the
    Makefile calls (`app_update.py`, `encrypt.py`, `flash_image.py`);
  - fonts (`mkfont.py` + `font/`), host-side tests (`host_check/`, `pd_decode_test.py`);
  - recovery (`recovery/`, `k1boot.py`);
  - profiling and timing (`pcprofile.py`, `verbtime.py`, `uitime.py`, `wdgtest.py`);
  - flash size (`inline_report.py`: per-function bytes spent on inlined copies, from the
    DWARF — candidates for `#[inline(never)]`);
  - dev-verb front ends and bench diagnostics (`i2cread.py`, `adcread.py`, `tft.py`,
    `tftgrab.py`, `fusb_intn.py`, `gpio_cc_sweep.py`, `snapshot.py` (GPIO/ADC/register snapshot + diff, dev-board), `km003c_check.py` (KM003C-protocol replies), `pdlog_check.py`,
    `accp_decode.py`, `scope_capture.py`).
- **`tools/lib/` — modules imported by the scripts, not run directly:**
  - `tc66lib.py`: meter discovery, `open_meter`, bootloader helpers.
  - `trigger_guard.py`: voltage limit + confirmation for trigger tools.
  - `pd_spec.py`: USB PD field layouts as named constants.
  - `pd_decode.py`: PD message decoder.
  - `pd_engine.py`: the meter's PD engine interface (`pdsta` / `pdcap` layouts, `xfer` /
    `ok_er` / `read_status`).
  - `km003c.py`: KM003C capture framing.
  - `rawva_proto.py`: the `rawva` reply.

`tools/udev/70-tc66.rules` gives non-root access to the meter's USB device
(KM003C vendor interface, used by `pdcap.py`); install steps are in the file.

Scripts put `tools/lib` on `sys.path` before importing from it (`"lib"` from
`tools/`, `"../lib"` from `tools/dev/`). Code that more than one script needs
goes in `lib/`, not in a runnable script.
