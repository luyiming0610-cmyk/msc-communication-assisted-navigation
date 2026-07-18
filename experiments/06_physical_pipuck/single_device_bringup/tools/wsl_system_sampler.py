#!/usr/bin/env python3
"""Same-shape local system sampler as pi_system_sampler.py, run on the WSL
side. Local only, no network calls per sample.

Usage: wsl_system_sampler.py OUTPUT_CSV_PATH [INTERVAL_S] [DURATION_S]
"""
import sys
import time


def read_cpu_times():
    with open("/proc/stat") as fh:
        line = fh.readline()
    parts = line.split()
    values = [int(v) for v in parts[1:9]]
    idle = values[3] + values[4]
    total = sum(values)
    return idle, total


def read_mem_mb():
    total_kb = available_kb = None
    with open("/proc/meminfo") as fh:
        for line in fh:
            if line.startswith("MemTotal:"):
                total_kb = int(line.split()[1])
            elif line.startswith("MemAvailable:"):
                available_kb = int(line.split()[1])
    if total_kb is None or available_kb is None:
        return None, None
    return total_kb / 1024.0, available_kb / 1024.0


def main():
    if len(sys.argv) < 2:
        print("Usage: wsl_system_sampler.py OUTPUT_CSV_PATH [INTERVAL_S] [DURATION_S]", file=sys.stderr)
        return 2
    output_path = sys.argv[1]
    interval_s = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    duration_s = float(sys.argv[3]) if len(sys.argv) > 3 else None

    prev_idle, prev_total = read_cpu_times()

    with open(output_path, "w") as fh:
        fh.write("unix_time_s,cpu_percent,mem_used_mb,mem_available_mb\n")
        fh.flush()
        start = time.time()
        try:
            while True:
                time.sleep(interval_s)
                now = time.time()

                idle, total = read_cpu_times()
                d_idle = idle - prev_idle
                d_total = total - prev_total
                cpu_percent = (1.0 - d_idle / d_total) * 100.0 if d_total > 0 else 0.0
                prev_idle, prev_total = idle, total

                mem_total_mb, mem_available_mb = read_mem_mb()
                mem_used_mb = (
                    (mem_total_mb - mem_available_mb)
                    if mem_total_mb is not None and mem_available_mb is not None
                    else None
                )

                fh.write(
                    f"{now:.3f},{cpu_percent:.2f},"
                    f"{'' if mem_used_mb is None else f'{mem_used_mb:.2f}'},"
                    f"{'' if mem_available_mb is None else f'{mem_available_mb:.2f}'}\n"
                )
                fh.flush()

                if duration_s is not None and (now - start) >= duration_s:
                    break
        except KeyboardInterrupt:
            pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
