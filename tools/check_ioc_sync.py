#!/usr/bin/env python3
"""Check that every .ioc's declared peripherals match its generated main.c.

CubeMX generates one MX_<PERIPH>_Init() per enabled peripheral. If someone
edits the .ioc without regenerating, or hand-writes an init that the .ioc does
not know about, the two drift. The dangerous direction is an init that exists
in main.c but not in the .ioc: the next person who regenerates from CubeMX
silently loses that code, because it sits outside the USER CODE guards.

This runs on a plain checkout with no CubeMX installed. It does not verify pin
assignments or clock configuration; only CubeMX itself can do that fully.

Known, accepted drift is listed in tools/ioc-sync-ignore.txt as
"<ioc path>: <SYMBOL>[,<SYMBOL>...]  # reason".

Usage: tools/check_ioc_sync.py [repo_root]
Exit 1 if any unaccepted drift is found.
"""
import re
import sys
from pathlib import Path

# Core/system entries and always-generated inits that never pair 1:1 with an
# Mcu.IP entry.
SKIP = {
    "CORTEX_M33_NS", "CORTEX_M33", "DEBUG", "SYS", "RCC", "PWR", "NVIC",
    "NVIC1", "LPBAM", "LPBAMQUEUE", "MEMORYMAP", "CRS",
    # MX_GPIO_Init is emitted for every project but GPIO is never an Mcu.IP.
    "GPIO",
}
SKIP_PREFIX = ("NUCLEO", "STM32", "B-U5", "LPBAM")


def read(path):
    try:
        return path.read_text(encoding="utf-8", errors="ignore")
    except OSError:
        return None


def ioc_peripherals(text):
    out = set()
    for m in re.finditer(r"^Mcu\.IP\d+=(.+)$", text, re.M):
        p = m.group(1).strip()
        if p in SKIP or any(p.startswith(x) for x in SKIP_PREFIX):
            continue
        out.add(p)
    return out


def main_c_inits(text):
    return {i for i in re.findall(r"\bMX_([A-Z0-9_]+?)_Init\s*\(", text)
            if i not in SKIP}


def related(a, b):
    """USART1 pairs with MX_USART1_UART_Init, so allow prefix/containment."""
    return a == b or b.startswith(a + "_") or a in b


def load_ignore(root):
    """{ioc_path: {SYMBOL, ...}} of accepted drift."""
    f = root / "tools" / "ioc-sync-ignore.txt"
    out = {}
    if not f.exists():
        return out
    for line in f.read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        path, syms = line.split(":", 1)
        out[path.strip()] = {s.strip() for s in syms.split(",") if s.strip()}
    return out


def main():
    root = Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
    ignore = load_ignore(root)
    iocs = sorted(p for p in root.rglob("*.ioc") if ".git" not in p.parts)
    if not iocs:
        print("no .ioc files found", file=sys.stderr)
        return 1

    bad = 0
    for ioc in iocs:
        rel = ioc.relative_to(root).as_posix()
        txt = read(ioc)
        main_c = read(ioc.parent / "Core" / "Src" / "main.c")
        if txt is None or main_c is None:
            print(f"SKIP  {rel} (no Core/Src/main.c)")
            continue

        periphs = ioc_peripherals(txt)
        inits = main_c_inits(main_c)
        allowed = ignore.get(rel, set())

        missing = {p for p in periphs
                   if not any(related(p, i) for i in inits)} - allowed
        extra = {i for i in inits
                 if not any(related(p, i) for p in periphs)} - allowed

        ver = re.search(r"^MxCube\.Version=(.+)$", txt, re.M)
        ver = ver.group(1) if ver else "?"
        if missing or extra:
            bad += 1
            print(f"DRIFT {rel}  (CubeMX {ver})")
            if missing:
                print(f"        declared in .ioc but never initialised: "
                      f"{', '.join(sorted(missing))}")
            if extra:
                print(f"        initialised in main.c but absent from .ioc: "
                      f"{', '.join(sorted(extra))}")
                print("        ^ regenerating from CubeMX would delete this")
        else:
            note = f", {len(allowed)} accepted" if allowed else ""
            print(f"OK    {rel}  (CubeMX {ver}, {len(periphs)} peripherals{note})")

    print(f"\n{bad} project(s) out of sync")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
