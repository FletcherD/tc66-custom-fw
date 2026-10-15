#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = ["pyelftools>=0.31", "rust-demangler>=1.0"]
# ///
"""Flash spent on inlined copies, per function, from the ELF's DWARF.

Every DW_TAG_inlined_subroutine is one inlined copy of a function. Grouping
them by origin gives each function's inline site count and the bytes its
copies occupy. A function inlined at N sites with S bytes total would cost
roughly S/N (one out-of-line body, 0 if one already exists) + N calls if
un-inlined; `est_save` is that difference. Nested copies are counted in
their parent too, so the totals overlap — read it as a candidate list, then
confirm with a real build.
"""
import argparse
import collections
import re
import sys

from elftools.elf.elffile import ELFFile
import rust_demangler

CALL_COST = 6  # bl (4) + some argument shuffling
FLASH = (0x08000000, 0x08010000)


def demangle(name):
    try:
        s = rust_demangler.demangle(name)
    except Exception:
        return name
    return re.sub(r"::h[0-9a-f]{16}$", "", s)


def die_name(die, cache):
    if die.offset in cache:
        return cache[die.offset]
    name = None
    for attr in ("DW_AT_linkage_name", "DW_AT_MIPS_linkage_name"):
        if attr in die.attributes:
            name = demangle(die.attributes[attr].value.decode())
            break
    if name is None:
        if "DW_AT_abstract_origin" in die.attributes:
            name = die_name(die.get_DIE_from_attribute("DW_AT_abstract_origin"), cache)
        elif "DW_AT_specification" in die.attributes:
            name = die_name(die.get_DIE_from_attribute("DW_AT_specification"), cache)
        elif "DW_AT_name" in die.attributes:
            name = die.attributes["DW_AT_name"].value.decode()
        else:
            name = f"<die {die.offset:#x}>"
    cache[die.offset] = name
    return name


def linked(die):
    lo = die.attributes.get("DW_AT_low_pc")
    return lo is not None and FLASH[0] <= lo.value < FLASH[1]


def die_size(die, dwarf, cu):
    a = die.attributes
    if "DW_AT_low_pc" in a and "DW_AT_high_pc" in a:
        hi = a["DW_AT_high_pc"]
        lo = a["DW_AT_low_pc"].value
        return hi.value if hi.form != "DW_FORM_addr" else hi.value - lo
    if "DW_AT_ranges" in a:
        rl = dwarf.range_lists()
        if rl is None:
            return 0
        total = 0
        for r in rl.get_range_list_at_offset(a["DW_AT_ranges"].value, cu=cu):
            if hasattr(r, "begin_offset"):
                total += r.end_offset - r.begin_offset
        return total
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("elf", nargs="?", default="target/thumbv7m-none-eabi/release/tc66-fw",
                    help="ELF built with debug info")
    ap.add_argument("-n", "--top", type=int, default=40, help="rows to print")
    ap.add_argument("--min-sites", type=int, default=2, help="minimum inline sites")
    ap.add_argument("--filter", default="", help="regex on the function name")
    args = ap.parse_args()

    with open(args.elf, "rb") as f:
        elf = ELFFile(f)
        dwarf = elf.get_dwarf_info()
        outline = set()
        for sec in elf.iter_sections():
            if sec.name == ".symtab":
                for s in sec.iter_symbols():
                    if s["st_info"]["type"] == "STT_FUNC" and s["st_size"] > 0:
                        outline.add(demangle(s.name))
        cache = {}
        sites = collections.defaultdict(list)
        for cu in dwarf.iter_CUs():
            for die in cu.iter_DIEs():
                # gc-sections drops dead functions but keeps their DWARF with a
                # tombstoned (0) address: count only subprograms in flash
                if die.tag != "DW_TAG_subprogram" or not linked(die):
                    continue
                stack = list(die.iter_children())
                while stack:
                    d = stack.pop()
                    stack.extend(d.iter_children())
                    if d.tag == "DW_TAG_inlined_subroutine":
                        size = die_size(d, dwarf, cu)
                        if size:
                            sites[die_name(d, cache)].append(size)

    rows = []
    flt = re.compile(args.filter) if args.filter else None
    for name, sizes in sites.items():
        n, total = len(sizes), sum(sizes)
        if n < args.min_sites or (flt and not flt.search(name)):
            continue
        body = 0 if name in outline else max(sizes)
        rows.append((total - body - n * CALL_COST, n, total, max(sizes), name in outline, name))
    rows.sort(reverse=True)
    print(f"{'est_save':>8} {'sites':>5} {'total':>6} {'max':>5} ool  function")
    for save, n, total, mx, ool, name in rows[: args.top]:
        print(f"{save:8d} {n:5d} {total:6d} {mx:5d} {'y' if ool else ' ':>3}  {name}")


if __name__ == "__main__":
    sys.exit(main())
