"""Guardrails shared by the charge-trigger host tools (pd.py, qc.py).

A granted trigger raises the voltage on the meter's OUT port to whatever
was requested — up to 20 V on anything plugged in downstream. Every
voltage-raising command therefore (1) refuses targets above a maximum
(--max-v, default DEFAULT_MAX_V) and (2) asks for an explicit confirmation
unless --yes is given.
"""
import argparse
import sys

DEFAULT_MAX_V = 9.0


def parse(doc):
    """Split the guard flags off argv; returns (guard args, remaining argv)."""
    ap = argparse.ArgumentParser(
        add_help=False, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--max-v", type=float, default=DEFAULT_MAX_V,
                    help="refuse any request above this OUT voltage (V)")
    ap.add_argument("--yes", action="store_true",
                    help="skip the confirmation prompt")
    ap.add_argument("-h", "--help", action="store_true")
    g, rest = ap.parse_known_args()
    if g.help or not rest:
        sys.exit(doc + "\n" + ap.format_help().split("\n", 2)[2])
    return g, rest


def confirm(g, volts, what):
    """Gate a request that may put `volts` on OUT. Exits on refusal."""
    if volts > g.max_v + 1e-6:
        sys.exit(f"refusing {what}: {volts:.2f} V exceeds --max-v {g.max_v:.2f} V "
                 "(raise --max-v if everything downstream is rated for it)")
    if g.yes:
        return
    print(f"{what} will put up to {volts:.2f} V on the meter's OUT port.\n"
          "Disconnect anything downstream that is not rated for it.")
    if not sys.stdin.isatty():
        sys.exit("no terminal to confirm on — pass --yes")
    if input("Type 'yes' to continue: ").strip().lower() != "yes":
        sys.exit("aborted")
