#!/usr/bin/env python3
"""Lightweight, dependency-free (stdlib only) local system sampler for the
Raspberry Pi side of physical_single_device_transport_diagnostic_pilot01.

Runs entirely locally on the Pi -- writes to a local CSV once per second.
Does NOT open any new SSH/network connection per sample (that would pollute
the RTT measurement being taken by the bridge at the same time). Copy the
resulting CSV off the Pi only after the run has cleanly stopped.

Usage:
    python3 pi_system_sampler.py OUTPUT_CSV_PATH [INTERVAL_S] [DURATION_S]

DURATION_S is optional; omit to run until Ctrl+C (SIGINT), which is the
expected use here since it should be stopped after the ROS-side run.
"""
import sys
import time


def read_cpu_times():
    with open("/proc/stat") as fh:
        line = fh.readline()
    parts = line.split()
    # user nice system idle iowait irq softirq steal
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


def read_net_bytes(iface="wlan0"):
    try:
        with open(f"/sys/class/net/{iface}/statistics/rx_bytes") as fh:
            rx = int(fh.read().strip())
        with open(f"/sys/class/net/{iface}/statistics/tx_bytes") as fh:
            tx = int(fh.read().strip())
        return rx, tx
    except OSError:
        return None, None


def read_wifi_signal(iface="wlan0"):
    """Parses /proc/net/wireless (no subprocess needed, avoids spawning
    iwconfig every sample). Format: link quality and signal level (dBm)."""
    try:
        with open("/proc/net/wireless") as fh:
            lines = fh.readlines()
    except OSError:
        return None, None
    for line in lines[2:]:
        parts = line.split()
        if not parts:
            continue
        if parts[0].rstrip(":") == iface:
            try:
                link_quality = float(parts[2])
                signal_dbm = float(parts[3])
                return link_quality, signal_dbm
            except (IndexError, ValueError):
                return None, None
    return None, None


def main():
    if len(sys.argv) < 2:
        print("Usage: pi_system_sampler.py OUTPUT_CSV_PATH [INTERVAL_S] [DURATION_S]", file=sys.stderr)
        return 2
    output_path = sys.argv[1]
    interval_s = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    duration_s = float(sys.argv[3]) if len(sys.argv) > 3 else None

    prev_idle, prev_total = read_cpu_times()
    prev_rx, prev_tx = read_net_bytes()

    with open(output_path, "w") as fh:
        fh.write(
            "unix_time_s,cpu_percent,mem_used_mb,mem_available_mb,"
            "net_rx_bytes_delta,net_tx_bytes_delta,"
            "wifi_link_quality,wifi_signal_dbm\n"
        )
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
                    else ""
                )

                rx, tx = read_net_bytes()
                rx_delta = (rx - prev_rx) if rx is not None and prev_rx is not None else ""
                tx_delta = (tx - prev_tx) if tx is not None and prev_tx is not None else ""
                if rx is not None:
                    prev_rx = rx
                if tx is not None:
                    prev_tx = tx

                link_quality, signal_dbm = read_wifi_signal()

                fh.write(
                    f"{now:.3f},{cpu_percent:.2f},"
                    f"{mem_used_mb if mem_used_mb == '' else f'{mem_used_mb:.2f}'},"
                    f"{mem_available_mb if mem_available_mb is None else f'{mem_available_mb:.2f}'},"
                    f"{rx_delta},{tx_delta},"
                    f"{link_quality if link_quality is not None else ''},"
                    f"{signal_dbm if signal_dbm is not None else ''}\n"
                )
                fh.flush()

                if duration_s is not None and (now - start) >= duration_s:
                    break
        except KeyboardInterrupt:
            pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
