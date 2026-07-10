#!/usr/bin/env python3
"""Live plot of the rudder controller's decoded PLRS-IMU attitude.

Reads the firmware's `imu` struct directly over SWD via openocd, without halting
the core (background AHB-AP memory reads), and plots heel / yaw rate / heading in
real time. No firmware change and no serial link needed; the ST-Link already
present is the only connection.

Data path:  openocd (telnet :4444)  --mdw-->  imu struct  -->  matplotlib

The five fields of interest are consecutive in memory, so a single
`mdw <base> 9` fetches them all. Only the base address (&imu.heading_deg) is
resolved from the ELF at startup, so the tool survives the global moving on a
rebuild as long as the struct layout is unchanged.

Run with the system python: `python3 imu_live.py`. Not `uv run` -- uv's
standalone Python + PyPI matplotlib wheel cannot reach the Wayland display on
NixOS (no libwayland on its path), so it falls back to a headless backend. The
nixpkgs matplotlib is linked against the system libs and just works.
"""

from __future__ import annotations

import argparse
import atexit
import os
import re
import shutil
import socket
import struct
import subprocess
import sys
import time
from collections import deque

# Word offsets within the read, relative to &imu.heading_deg. Fixed by the
# PLRS_IMU struct layout (see PLRS_IMU.h), independent of where imu lands.
OFF_HEADING = 0  # float, deg
OFF_HEEL = 3     # float, deg
OFF_YAW = 4      # float, deg/s
OFF_LAST_RX = 6  # uint32, HAL tick ms of last good frame
OFF_DROPS = 8    # uint32, cumulative dropped frames
READ_WORDS = 9

# Relative to this script, so it resolves no matter the CWD it's launched from.
ELF_DEFAULT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "Debug", "rudder-controller.elf")


def resolve_base(elf: str, gdb: str) -> int:
    """Resolve &imu.heading_deg from the ELF (one brief core halt at startup)."""
    out = subprocess.run(
        [gdb, "-q", elf, "-batch",
         "-ex", "target extended-remote :3333",
         "-ex", "printf \"BASE %p\\n\", &imu.heading_deg",
         "-ex", "detach"],
        capture_output=True, text=True, timeout=15,
    ).stdout
    m = re.search(r"BASE 0x([0-9a-fA-F]+)", out)
    if not m:
        sys.exit(f"could not resolve &imu.heading_deg from {elf}:\n{out}")
    return int(m.group(1), 16)


class OpenOcdTelnet:
    """Persistent openocd telnet session for repeated non-intrusive reads."""

    def __init__(self, host: str = "localhost", port: int = 4444) -> None:
        self.sock = socket.create_connection((host, port), timeout=5)
        self.sock.settimeout(5)
        self._drain_until_prompt()

    def _drain_until_prompt(self) -> str:
        buf = b""
        while b"> " not in buf:
            chunk = self.sock.recv(4096)
            if not chunk:
                break
            buf += chunk
        return buf.decode(errors="replace")

    def command(self, cmd: str) -> str:
        self.sock.sendall(cmd.encode() + b"\n")
        return self._drain_until_prompt()

    def read_words(self, addr: int, count: int) -> list[int]:
        resp = self.command(f"mdw 0x{addr:08x} {count}")
        words: list[int] = []
        for line in resp.splitlines():
            if ":" not in line:
                continue
            _, _, data = line.partition(":")
            for tok in data.split():
                if re.fullmatch(r"[0-9a-fA-F]{8}", tok):
                    words.append(int(tok, 16))
        return words


def word_to_float(word: int) -> float:
    return struct.unpack(">f", word.to_bytes(4, "big"))[0]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--elf", default=ELF_DEFAULT)
    ap.add_argument("--gdb", default="arm-none-eabi-gdb")
    ap.add_argument("--hz", type=float, default=15.0, help="poll rate")
    ap.add_argument("--window", type=float, default=15.0, help="seconds shown")
    ap.add_argument("--stale-ms", type=int, default=250,
                    help="link deemed stale if last_rx_ms stops advancing")
    ap.add_argument("--no-openocd", action="store_true",
                    help="attach to an already-running openocd")
    ap.add_argument("--selftest", type=float, default=0.0,
                    help="headless: print N seconds of samples, no plot")
    args = ap.parse_args()

    if not args.no_openocd:
        openocd = shutil.which("openocd")
        if openocd is None:
            sys.exit("openocd not found on PATH (or pass --no-openocd)")
        proc = subprocess.Popen(
            [openocd, "-f", "interface/stlink.cfg", "-f", "target/stm32u5x.cfg"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        atexit.register(proc.terminate)
        time.sleep(2)

    base = resolve_base(args.elf, args.gdb)
    ocd = OpenOcdTelnet()
    ocd.command("resume")  # gdb detach may have left the core halted

    if args.selftest:
        deadline = time.time() + args.selftest
        while time.time() < deadline:
            w = ocd.read_words(base, READ_WORDS)
            print(f"heading={word_to_float(w[OFF_HEADING]):7.2f}  "
                  f"heel={word_to_float(w[OFF_HEEL]):7.2f}  "
                  f"yaw={word_to_float(w[OFF_YAW]):7.2f}  "
                  f"last_rx_ms={w[OFF_LAST_RX]:>8}  drops={w[OFF_DROPS]}")
            time.sleep(1.0 / args.hz)
        return

    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    t0 = time.time()
    t: deque[float] = deque()
    heel: deque[float] = deque()
    yaw: deque[float] = deque()
    heading: deque[float] = deque()
    state = {"last_rx": -1, "last_rx_wall": t0, "drops": 0}

    fig, (ax_h, ax_y, ax_hd) = plt.subplots(3, 1, sharex=True, figsize=(9, 7))
    fig.suptitle("Rudder controller: decoded PLRS-IMU attitude (live over SWD)")
    (ln_heel,) = ax_h.plot([], [], color="tab:blue")
    (ln_yaw,) = ax_y.plot([], [], color="tab:green")
    (ln_hd,) = ax_hd.plot([], [], color="tab:red")
    ax_h.set_ylabel("heel (deg)")
    ax_y.set_ylabel("yaw rate (deg/s)")
    ax_hd.set_ylabel("heading (deg)")
    ax_hd.set_xlabel("time (s)")
    status = ax_h.text(0.01, 0.95, "", transform=ax_h.transAxes, va="top",
                       family="monospace")
    for ax in (ax_h, ax_y, ax_hd):
        ax.grid(True, alpha=0.3)

    def update(_frame):
        now = time.time()
        w = ocd.read_words(base, READ_WORDS)
        if len(w) < READ_WORDS:
            return ln_heel, ln_yaw, ln_hd, status
        last_rx = w[OFF_LAST_RX]
        if last_rx != state["last_rx"]:
            state["last_rx"] = last_rx
            state["last_rx_wall"] = now
        state["drops"] = w[OFF_DROPS]

        t.append(now - t0)
        heel.append(word_to_float(w[OFF_HEEL]))
        yaw.append(word_to_float(w[OFF_YAW]))
        heading.append(word_to_float(w[OFF_HEADING]))
        while t and t[0] < t[-1] - args.window:
            t.popleft(); heel.popleft(); yaw.popleft(); heading.popleft()

        ln_heel.set_data(t, heel)
        ln_yaw.set_data(t, yaw)
        ln_hd.set_data(t, heading)
        for ax in (ax_h, ax_y, ax_hd):
            ax.set_xlim(max(0, t[-1] - args.window), max(args.window, t[-1]))
            ax.relim(); ax.autoscale_view(scalex=False)

        stale = (now - state["last_rx_wall"]) * 1000 > args.stale_ms
        status.set_text(
            ("LINK STALE" if stale else "LINK LIVE")
            + f"   last_rx_ms={last_rx}   drops={state['drops']}")
        status.set_color("tab:red" if stale else "tab:green")
        return ln_heel, ln_yaw, ln_hd, status

    _anim = FuncAnimation(fig, update, interval=1000.0 / args.hz,
                          blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
