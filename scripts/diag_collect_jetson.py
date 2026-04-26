#!/usr/bin/env python3
# Collect Jetson system state right after a diagnostic probe failed.
# Snapshots kernel/proc views, cpu/gpu freq, thermal zones, dmesg tail, and
# tegrastats. Writes the report to stdout, or appends to --append-to.
import argparse
import datetime as dt
import os
import shutil
import subprocess
import sys
from pathlib import Path


def run(cmd, timeout=8):
    try:
        out = subprocess.run(
            cmd,
            shell=isinstance(cmd, str),
            timeout=timeout,
            capture_output=True,
            text=True,
            check=False,
        )
        return out.stdout + out.stderr
    except subprocess.TimeoutExpired:
        return f"<timeout after {timeout}s>\n"
    except FileNotFoundError as exc:
        return f"<missing: {exc}>\n"
    except Exception as exc:  # noqa: BLE001
        return f"<error: {exc}>\n"


def read_text(path):
    try:
        return Path(path).read_text()
    except Exception as exc:  # noqa: BLE001
        return f"<error: {exc}>\n"


def section(title, body):
    return f"\n===== {title} =====\n{body}\n"


def collect():
    out = []
    out.append(section("collected_at_utc", dt.datetime.utcnow().isoformat()))

    out.append(section("uname -a", run(["uname", "-a"])))
    out.append(section("nvpmodel -q", run(["nvpmodel", "-q"])))
    out.append(section("/proc/loadavg", read_text("/proc/loadavg")))
    out.append(section("/proc/meminfo (head)", "".join(read_text("/proc/meminfo").splitlines(keepends=True)[:20])))

    # CPU frequencies
    cpu_block = []
    for cpu_dir in sorted(Path("/sys/devices/system/cpu").glob("cpu[0-9]*")):
        cur = cpu_dir / "cpufreq/scaling_cur_freq"
        if cur.exists():
            cpu_block.append(f"{cpu_dir.name} cur_freq={read_text(cur).strip()}")
    out.append(section("cpu freqs", "\n".join(cpu_block) if cpu_block else "<none>"))

    # GPU frequencies (devfreq)
    gpu_block = []
    for p in Path("/sys/class/devfreq").glob("*"):
        cur = p / "cur_freq"
        if cur.exists():
            gpu_block.append(f"{p.name} cur_freq={read_text(cur).strip()}")
    out.append(section("devfreq gpu freqs", "\n".join(gpu_block) if gpu_block else "<none>"))

    # Thermal zones
    therm_block = []
    for p in sorted(Path("/sys/devices/virtual/thermal").glob("thermal_zone*")):
        type_p = p / "type"
        temp_p = p / "temp"
        t = read_text(type_p).strip()
        v = read_text(temp_p).strip()
        try:
            v_c = f"{int(v) / 1000.0:.1f}C"
        except ValueError:
            v_c = v
        therm_block.append(f"{p.name} type={t} temp={v_c}")
    out.append(section("thermal zones", "\n".join(therm_block) if therm_block else "<none>"))

    # tegrastats one-shot (3 samples)
    if shutil.which("tegrastats"):
        out.append(section("tegrastats (3 lines @ 200ms)",
                           run("tegrastats --interval 200 2>&1 | head -n 3", timeout=4)))
    else:
        out.append(section("tegrastats", "<not installed>\n"))

    # dmesg tail (last 10 minutes if --since supported)
    out.append(section("dmesg tail",
                       run("dmesg --since '10 min ago' 2>/dev/null || dmesg | tail -n 200")))

    # Stacks of all processes named 'auto_battlebot' or 'diag_*'
    proc_block = []
    for pid_dir in Path("/proc").iterdir():
        if not pid_dir.name.isdigit():
            continue
        comm = read_text(pid_dir / "comm").strip()
        if comm == "auto_battlebot" or comm.startswith("diag_"):
            stack = read_text(pid_dir / "stack")
            wchan = read_text(pid_dir / "wchan").strip()
            proc_block.append(f"--- pid={pid_dir.name} comm={comm} wchan={wchan} ---\n{stack}")
    out.append(section("auto_battlebot/diag_* stacks",
                       "\n".join(proc_block) if proc_block else "<none running>"))

    return "".join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tag", default="", help="Free-form tag included in the header")
    ap.add_argument("--append-to", default="", help="Append the report to this file")
    args = ap.parse_args()

    header = f"\n##### diag_collect_jetson tag={args.tag or '<none>'} pid={os.getpid()} #####\n"
    body = header + collect()

    if args.append_to:
        with open(args.append_to, "a") as f:
            f.write(body)
    else:
        sys.stdout.write(body)


if __name__ == "__main__":
    main()
