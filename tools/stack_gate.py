#!/usr/bin/env python3
"""Static worst-case stack-depth gate (make stack-gate).

Build 35 died to a silent boot-path stack overflow: main()'s frame grew past
the fenced RAM region (memory.x) and the overflow hardfaulted inside ui::draw
BEFORE the D+ pull-up — an invisible dead board needing K1 recovery. This gate
computes a conservative worst-case stack depth from the disassembly and fails
the build when the margin gets thin, so that never ships again.

Method: parse `objdump -d` (.text only — .flasher runs post-erase in its own
world and is covered by flasher-gate), sum each function's SP-decrements
(push/stmdb/sub sp — summing over the whole body overestimates multi-path
frames, which is the safe direction), build the direct call graph (bl, plus
cross-function b/b.w tail branches, treated as calls = overestimate), and
DFS-memoize the deepest chain from Reset. Indirect calls (blx rN — the
usb-device trait-object dispatch in dev.poll) are charged the max subtree
depth over every UsbClass impl method, floored at INDIRECT_FLOOR. Recursion
in the direct graph is a hard error.

Interrupt/exception charges: the gate reads the ELF .vector_table and
resolves every populated slot to its handler function. Device IRQs (slots
16+) can preempt any thread-mode chain, so the worst case charges one
hardware exception frame (32 B) + the DEEPEST device-IRQ handler subtree on
top of the thread worst case. ASSUMPTION (do not break it): all enabled IRQs
share one NVIC priority, so device IRQs never nest — the gate charges the
max handler chain, not the sum. System-exception slots (2..15 — faults; they
spin and never return) are charged the same way on top of everything.
A populated vector that doesn't resolve to a disassembled .text function is
a hard error — never silently uncharged.

The stack budget is STACK_TOP - __ebss from nm. The gate requires
worst_case + irq_charge + fault_charge + MARGIN <= budget.

Usage: stack_gate.py <elf> [--margin N]
"""
import re
import subprocess
import sys

INDIRECT_FLOOR = 192  # min charge for a blx rN if UsbClass scan finds less
EXC_FRAME = 32        # one hardware exception stacking (r0-r3,r12,lr,pc,xpsr)
VECTORS = 16 + 52     # 16 system slots + the GD32F150's 52 device IRQ slots
DEFAULT_MARGIN = 192  # required headroom beyond the computed worst case

# The RUNTIME stack top: __pre_init raises MSP here (startup.rs), just below
# the SRAM marker window at 0x20001000. The linker's _stack_start stays at
# 0x20000E40 because the flash header's initial SP must keep the
# bootloader-proven value. KEEP IN SYNC with startup.rs __pre_init.
STACK_TOP = 0x2000_1000

PUSH_RE = re.compile(r"\b(?:push|push\.w|stmdb\s+sp!,)\s*\{([^}]*)\}")
SUBSP_RE = re.compile(r"\bsub(?:\.w)?\s+sp,(?:\s*sp,)?\s*#(\d+)")
VPUSH_RE = re.compile(r"\bvpush\s*\{([^}]*)\}")
CALL_RE = re.compile(r"\b(?:bl|b\.n|b\.w)\s+([0-9a-f]+)\s+<([^+>]+)(\+0x[0-9a-f]+)?>")
BLX_REG_RE = re.compile(r"\bblx\s+r\d")
FUNC_RE = re.compile(r"^([0-9a-f]+) <(.+)>:$")
SECTION_RE = re.compile(r"^Disassembly of section (\S+):")


def parse(disasm):
    funcs = {}  # name -> {frame, calls:set, indirect:bool, addr}
    cur = None
    section = None
    for line in disasm.splitlines():
        m = SECTION_RE.match(line)
        if m:
            section = m.group(1)
            continue
        m = FUNC_RE.match(line)
        if m:
            if section == ".text":
                cur = {"frame": 0, "calls": set(), "indirect": False,
                       "addr": int(m.group(1), 16), "name": m.group(2)}
                funcs[m.group(2)] = cur
            else:
                cur = None
            continue
        if cur is None:
            continue
        m = PUSH_RE.search(line)
        if m:
            cur["frame"] += 4 * len([r for r in m.group(1).split(",") if r.strip()])
            continue
        m = SUBSP_RE.search(line)
        if m:
            cur["frame"] += int(m.group(1))
            continue
        m = VPUSH_RE.search(line)
        if m:
            cur["frame"] += 8 * len([r for r in m.group(1).split(",") if r.strip()])
            continue
        if BLX_REG_RE.search(line):
            cur["indirect"] = True
            continue
        m = CALL_RE.search(line)
        if m and m.group(2) != cur["name"]:
            # cross-function bl or tail branch; b.n/b.w to <sym+off> is a
            # mid-function jump — only whole-symbol targets are calls
            if m.group(3) is None:
                cur["calls"].add(m.group(2))
    return funcs


def worst_depth(funcs, indirect_cost):
    memo = {}
    onstack = []

    def depth(name):
        f = funcs.get(name)
        if f is None:
            return 0, [name + "?"]  # target outside .text (e.g. veneer) — leaf
        if name in memo:
            return memo[name]
        if name in onstack:
            cyc = onstack[onstack.index(name):] + [name]
            sys.exit("stack-gate: recursion in call graph: " + " -> ".join(cyc))
        onstack.append(name)
        best, chain = 0, []
        for c in f["calls"]:
            d, ch = depth(c)
            if d > best:
                best, chain = d, ch
        if f["indirect"] and indirect_cost > best:
            best, chain = indirect_cost, ["<indirect: UsbClass max>"]
        onstack.pop()
        memo[name] = (f["frame"] + best, [f"{name} [{f['frame']}]"] + chain)
        return memo[name]

    return depth


def vector_handlers(elf, funcs):
    """Resolve .vector_table slots to handler names.

    Returns (sys_handlers, irq_handlers): names for populated system-
    exception slots (2..15) and device-IRQ slots (16..). Reset (slot 1) is
    the graph entry, not a preemptor. A populated slot that doesn't map to a
    disassembled .text symbol is fatal — it would go uncharged otherwise.
    """
    dump = subprocess.run(["arm-none-eabi-objdump", "-s", "-j", ".vector_table",
                           elf], capture_output=True, text=True, check=True).stdout
    data = bytearray()
    for line in dump.splitlines():
        m = re.match(r"^ [0-9a-f]+ ((?:[0-9a-f]{2,8} ?){1,4})", line)
        if m:
            data += bytes.fromhex(m.group(1).replace(" ", ""))
    words = [int.from_bytes(data[i:i + 4], "little")
             for i in range(0, len(data), 4)]
    if len(words) < VECTORS:
        sys.exit(f"stack-gate: .vector_table has {len(words)} slots, "
                 f"expected {VECTORS}")
    addr2name = {f["addr"]: n for n, f in funcs.items()}

    def resolve(slot, w):
        name = addr2name.get(w & ~1)
        if name is None:
            sys.exit(f"stack-gate: vector slot {slot} -> {w:#x} does not "
                     "resolve to a .text function; extend the gate")
        return name

    sys_h = {resolve(i, w) for i, w in enumerate(words[2:16], 2) if w}
    irq_h = {resolve(i, w) for i, w in enumerate(words[16:VECTORS], 16) if w}
    return sys_h, irq_h


def main():
    elf = sys.argv[1]
    margin = DEFAULT_MARGIN
    if "--margin" in sys.argv:
        margin = int(sys.argv[sys.argv.index("--margin") + 1])

    disasm = subprocess.run(["arm-none-eabi-objdump", "-d", elf],
                            capture_output=True, text=True, check=True).stdout
    nm = subprocess.run(["arm-none-eabi-nm", elf],
                        capture_output=True, text=True, check=True).stdout
    syms = {}
    for line in nm.splitlines():
        parts = line.split()
        if len(parts) == 3:
            syms[parts[2]] = int(parts[0], 16)
    assert syms["_stack_start"] <= STACK_TOP, "linker stack above runtime top?"
    assert syms["__ebss"] < STACK_TOP, "__ebss above stack top"
    budget = STACK_TOP - syms["__ebss"]

    funcs = parse(disasm)

    # charge for indirect calls: deepest UsbClass impl method subtree
    probe = worst_depth(funcs, 0)
    usb_impls = [n for n in funcs if "UsbClass" in n]
    indirect_cost = max([probe(n)[0] for n in usb_impls] + [INDIRECT_FLOOR])

    depth = worst_depth(funcs, indirect_cost)
    entry = next((n for n in ("Reset", "main") if n in funcs), None)
    if entry is None:
        sys.exit("stack-gate: no Reset/main symbol found")
    worst, chain = depth(entry)

    # Preemption charges from the real vector table (see module docstring):
    # deepest device-IRQ handler can land on the thread worst case; a fault
    # (spins, never returns) can then land on top of that.
    sys_handlers, irq_handlers = vector_handlers(elf, funcs)
    irq_worst, irq_name = max(((depth(h)[0], h) for h in irq_handlers),
                              default=(0, "-"))
    fault_worst, fault_name = max(((depth(h)[0], h) for h in sys_handlers),
                                  default=(0, "-"))
    irq_charge = (EXC_FRAME + irq_worst) if irq_handlers else 0
    fault_charge = EXC_FRAME + fault_worst

    if "--top" in sys.argv:
        tot = sorted(((depth(n)[0], n) for n in funcs), reverse=True)
        for d, n in tot[:15]:
            print(f"{d:6d}  {n}")
        print("worst chain from", entry)
        for hop in chain[:12]:
            print("  " + hop)
        print(f"irq handlers: {sorted(irq_handlers)}")
        if irq_handlers:
            print(f"deepest irq chain ({irq_name}):")
            for hop in depth(irq_name)[1][:8]:
                print("  " + hop)
        return
    need = worst + irq_charge + fault_charge + margin

    print(f"stack-gate: worst {worst} B + irq {irq_charge} B "
          f"({EXC_FRAME} frame + {irq_worst} {irq_name.split('_RN')[0] or 'chain'})"
          f" + fault {fault_charge} B + {margin} margin = {need} B of "
          f"{budget} B ({budget - need + margin} B headroom) "
          f"[indirect charge {indirect_cost} B]")
    if need > budget:
        print("worst chain:")
        for hop in chain[:12]:
            print("  " + hop)
        if irq_handlers:
            print(f"deepest irq chain ({irq_name}):")
            for hop in depth(irq_name)[1][:8]:
                print("  " + hop)
        sys.exit(f"stack-gate FAIL: need {need} B > budget {budget} B "
                 f"(STACK_TOP-__ebss). Shrink deep-chain or handler frames "
                 f"(or shallow boot-time init); see FIRMWARE.md.")
    print("stack-gate ok")


if __name__ == "__main__":
    main()
