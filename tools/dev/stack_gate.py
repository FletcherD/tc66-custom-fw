#!/usr/bin/env python3
"""Static worst-case stack-depth gate (make stack-gate).

Build 35 died to a silent boot-path stack overflow: main()'s frame grew past
the fenced RAM region (memory.x) and the overflow hardfaulted inside ui::draw
BEFORE the D+ pull-up — an invisible dead board needing K1 recovery. This gate
computes a conservative worst-case stack depth from the disassembly and fails
the build when the margin gets thin, so that never ships again.

Method: parse `objdump -d` (every section except .flasher — that runs
post-erase in its own world and is covered by flasher-gate), sum each
function's SP-decrements (push/stmdb/sub sp — summing over the whole body
overestimates multi-path frames, which is the safe direction), build the
direct call graph (bl, plus cross-function b/b.w tail branches, treated as
calls = overestimate), and DFS-memoize the deepest chain from Reset.
Indirect calls (blx rN — the usb-device trait-object dispatch in dev.poll)
are charged the max subtree depth over every UsbClass impl method, floored
at INDIRECT_FLOOR. Recursion in the direct graph is a hard error.

Interrupts run code now (the USBD_LP ISR), so the worst case must include a
handler preempting the deepest thread-mode chain. Handlers are discovered
from the .vector_table section: entries [2..16] are system exceptions
(faults — ours only record and spin), entries [16..] are IRQs. Every
nonzero entry must resolve to a disassembled function or the gate fails.
Two phases are checked, both against the budget:

  boot  (PRIMASK set from __pre_init until main() unmasks; only a fault can
         stack):   worst_full + EXC_FRAME + exc_worst + MARGIN
  run   (USBD_LP unmasked; an IRQ can preempt the deepest chain and a fault
         can preempt the ISR):
                   worst_run + EXC_FRAME + irq_worst + EXC_FRAME
                   + exc_worst + MARGIN

worst_run prunes PRE_IRQ_ROOTS — deep one-shot chains that provably run
only before the unmask (today: none). The gate verifies each pruned root
has exactly one caller (the main body) and is unreachable from any handler,
but the *ordering* (called before NVIC::unmask) is a code invariant — keep
the call before the unmask or delete it from PRE_IRQ_ROOTS.

The stack budget is STACK_TOP - __ebss from nm.

Usage: stack_gate.py <elf> [--margin N] [--top]
"""
import re
import subprocess
import sys

INDIRECT_FLOOR = 192  # min charge for a blx rN if UsbClass scan finds less
EXC_FRAME = 32        # one hardware exception stacking (r0-r3,r12,lr,pc,xpsr)
DEFAULT_MARGIN = 192  # required headroom beyond the computed worst case

# Deep one-shot chains that run only while the USBD IRQ is still masked
# (before NVIC::unmask in main.rs) — pruned from the run-phase thread worst.
# Substrings of the v0-mangled names (crate-hash independent).
PRE_IRQ_ROOTS = []
# ...and the only caller each is allowed to have (the main body): if the
# call moves anywhere else, fail loudly instead of under-counting.
PRE_IRQ_CALLER = "cortex_m_rt_main"

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
            if section != ".flasher":
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

    # Collapse linker-generated long-branch veneers (fast.x's section
    # placement makes lld emit them): a thunk is a stackless trampoline to
    # exactly one target, so rewrite callers to call the target directly.
    # Leaving them in would double-count PRE_IRQ_ROOTS matches and hide the
    # real caller behind the veneer.
    thunks = {}
    prefix = "__Thumbv7ABSLongThunk"
    for name, f in funcs.items():
        if name.startswith(prefix):
            assert f["frame"] == 0, f"thunk {name} has a stack frame"
            # the body is movw/movt+bx (no parseable branch): the target is
            # encoded in the symbol name (lld separates with an underscore:
            # __Thumbv7ABSLongThunk_<target>)
            target = name[len(prefix):]
            if target not in funcs and target[1:] in funcs:
                target = target[1:]
            thunks[name] = target
    for name in thunks:
        del funcs[name]
    for f in funcs.values():
        f["calls"] = {thunks.get(c, c) for c in f["calls"]}
    return funcs


def worst_depth(funcs, indirect_cost, skip=()):
    memo = {}
    onstack = []

    def depth(name):
        f = funcs.get(name)
        if f is None:
            return 0, [name + "?"]  # target outside parsed code (veneer) — leaf
        if any(s in name for s in skip):
            return 0, [name + " <pre-IRQ only: pruned>"]
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
    """Map the vector table to handler function names.

    Returns (exc_names, irq_names): system-exception handlers (entries 2..16)
    and IRQ handlers (entries 16..). Every nonzero entry must resolve to a
    disassembled function — an unknown handler cannot be stack-bounded.
    """
    out = subprocess.run(["arm-none-eabi-objdump", "-s", "-j", ".vector_table",
                          elf], capture_output=True, text=True, check=True).stdout
    words = []
    for line in out.splitlines():
        m = re.match(r"^ ([0-9a-f]+) ((?:[0-9a-f]{8} ?){1,4})", line)
        if m:
            for tok in m.group(2).split():
                words.append(int.from_bytes(bytes.fromhex(tok), "little"))
    if len(words) < 16:
        sys.exit("stack-gate: .vector_table too short (%d words)" % len(words))
    by_addr = {f["addr"]: name for name, f in funcs.items()}

    def resolve(idx, w):
        name = by_addr.get(w & ~1)
        if name is None:
            sys.exit("stack-gate: vector[%d] = %#x resolves to no disassembled"
                     " function — cannot bound its stack" % (idx, w))
        return name

    exc = {resolve(i, w) for i, w in enumerate(words[2:16], 2) if w}
    irq = {resolve(i, w) for i, w in enumerate(words[16:], 16) if w}
    return exc, irq


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
    worst_full, chain_full = depth(entry)

    # ---- interrupt handlers, from the vector table ----
    exc_names, irq_names = vector_handlers(elf, funcs)
    exc_worst, exc_chain = max((depth(n) for n in exc_names),
                               default=(0, []))
    irq_worst, irq_chain = max((depth(n) for n in irq_names),
                               default=(0, []))

    # ---- validate the PRE_IRQ_ROOTS pruning ----
    reachable_from_handlers = set()

    def walk(name):
        if name in reachable_from_handlers or name not in funcs:
            return
        reachable_from_handlers.add(name)
        for c in funcs[name]["calls"]:
            walk(c)

    for h in exc_names | irq_names:
        walk(h)
    for root in PRE_IRQ_ROOTS:
        matches = [n for n in funcs if root in n]
        if len(matches) != 1:
            sys.exit(f"stack-gate: PRE_IRQ_ROOTS {root!r} matched "
                     f"{len(matches)} functions (want exactly 1): {matches}")
        name = matches[0]
        callers = [n for n, f in funcs.items() if name in f["calls"]]
        if len(callers) != 1 or PRE_IRQ_CALLER not in callers[0]:
            sys.exit(f"stack-gate: pre-IRQ root {name} must be called exactly "
                     f"once, from *{PRE_IRQ_CALLER}* (before NVIC::unmask) — "
                     f"callers: {callers}")
        if name in reachable_from_handlers:
            sys.exit(f"stack-gate: pre-IRQ root {name} is reachable from an "
                     f"interrupt handler — pruning it would be unsound")

    depth_run = worst_depth(funcs, indirect_cost, skip=PRE_IRQ_ROOTS)
    worst_run, chain_run = depth_run(entry)

    if "--top" in sys.argv:
        tot = sorted(((depth(n)[0], n) for n in funcs), reverse=True)
        for d, n in tot[:15]:
            print(f"{d:6d}  {n}")
        for label, ch in (("thread full (boot phase)", chain_full),
                          ("thread pruned (run phase)", chain_run),
                          ("worst IRQ handler", irq_chain),
                          ("worst exception handler", exc_chain)):
            print(f"-- {label}:")
            for hop in ch[:12]:
                print("  " + hop)
        return

    # boot phase: IRQs masked since __pre_init; only a fault can stack.
    need_boot = worst_full + EXC_FRAME + exc_worst + margin
    # run phase: USBD IRQ unmasked; it preempts the deepest (pruned) thread
    # chain, and a fault can preempt the ISR. No IRQ nesting (single IRQ,
    # one priority); faults spin, so one of each is the true worst case.
    need_run = worst_run + EXC_FRAME + irq_worst + EXC_FRAME + exc_worst \
        + margin
    need = max(need_boot, need_run)

    print(f"stack-gate: boot {worst_full}+{EXC_FRAME}+{exc_worst} = "
          f"{need_boot - margin} B | run {worst_run}+{EXC_FRAME}+{irq_worst}"
          f"+{EXC_FRAME}+{exc_worst} = {need_run - margin} B | +{margin} "
          f"margin vs budget {budget} B "
          f"({budget - max(need_boot, need_run) + margin} B headroom) "
          f"[indirect charge {indirect_cost} B]")
    if need > budget:
        for label, ch in (("thread (boot)", chain_full),
                          ("thread (run)", chain_run),
                          ("IRQ handler", irq_chain),
                          ("exception handler", exc_chain)):
            print(f"{label} chain:")
            for hop in ch[:12]:
                print("  " + hop)
        sys.exit(f"stack-gate FAIL: need {need} B > budget {budget} B "
                 f"(STACK_TOP-__ebss). Shrink deep-chain frames, keep the "
                 f"ISR shallow, or move one-shot init pre-unmask; see "
                 f"docs/FIRMWARE.md.")
    print("stack-gate ok")


if __name__ == "__main__":
    main()
