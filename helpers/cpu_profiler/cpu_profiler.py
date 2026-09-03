#!/usr/bin/env python3
"""
ReloBot CPU & Memory Profiler
=============================
Collects time-series CPU and memory utilization statistics across all Docker containers,
ROS 2 nodes, and composable container threads on Linux / Raspberry Pi.

Usage:
  ./profile_cpu.sh [DURATION_SECONDS]
  python3 helpers/cpu_profiler/cpu_profiler.py --duration 30 --interval 1.0 --output /tmp/cpu_report.json
"""

import os
import sys
import time
import json
import math
import argparse
import subprocess
from collections import defaultdict

def get_clk_tck():
    return os.sysconf(os.sysconf_names["SC_CLK_TCK"])

def get_num_cpus():
    return os.cpu_count() or 4

def get_system_cpu():
    with open("/proc/stat", "r") as f:
        for line in f:
            if line.startswith("cpu "):
                parts = [int(x) for x in line.split()[1:]]
                # user, nice, system, idle, iowait, irq, softirq, steal
                idle = parts[3] + (parts[4] if len(parts) > 4 else 0)
                total = sum(parts)
                return total, idle
    return 0, 0

def read_proc_stat(stat_path):
    try:
        with open(stat_path, "r") as f:
            content = f.read()
        rparen = content.rfind(")")
        if rparen == -1:
            return None
        comm = content[content.find("(")+1:rparen]
        fields = content[rparen+2:].split()
        # fields after comm:
        # 11: utime, 12: stime, 17: num_threads, 19: starttime, 21: vsize, 22: rss
        utime = int(fields[11])
        stime = int(fields[12])
        rss_pages = int(fields[21])
        return {
            "comm": comm,
            "utime": utime,
            "stime": stime,
            "total_time": utime + stime,
            "rss_pages": rss_pages
        }
    except (FileNotFoundError, ProcessLookupError, PermissionError):
        return None

def parse_ros_node_name(cmd):
    if "__node:=" in cmd:
        for token in cmd.split():
            if "__node:=" in token or token.startswith("-r__node:="):
                return token.split("__node:=")[-1]
            if token.startswith("__node:="):
                return token.split(":=")[-1]
    
    tokens = cmd.split()
    exe = tokens[0] if tokens else ""
    base = os.path.basename(exe)
    if "python" in base:
        for t in tokens[1:]:
            if not t.startswith("-") and ("/" in t or t.endswith(".py")):
                return os.path.basename(t)
    if base in ["ros2", "python3", "bash", "sh"]:
        for t in tokens[1:]:
            if "launch" in t or ".py" in t:
                return f"launch:{os.path.basename(t)}"
    return base

def get_containers_and_pids():
    """Maps running docker containers to their host PIDs and ROS 2 node names."""
    try:
        out = subprocess.check_output(["docker", "ps", "--format", "{{.ID}}|{{.Names}}"], text=True)
    except Exception as e:
        print(f"Error listing docker containers: {e}")
        return {}

    containers = {}
    for line in out.strip().splitlines():
        if not line:
            continue
        cid, name = line.split("|", 1)
        containers[name] = {"id": cid, "processes": []}
        try:
            top_out = subprocess.check_output(["docker", "top", cid], text=True)
            lines = top_out.strip().splitlines()
            if len(lines) > 1:
                for pline in lines[1:]:
                    parts = pline.split(None, 7)
                    if len(parts) >= 8:
                        pid = int(parts[1])
                        cmd = parts[7]
                        node_name = parse_ros_node_name(cmd)
                        containers[name]["processes"].append({
                            "pid": pid,
                            "cmd": cmd,
                            "node_name": node_name
                        })
        except Exception:
            pass
    return containers

def collect_statistics(duration_sec=30, interval_sec=1.0):
    clk_tck = get_clk_tck()
    num_cpus = get_num_cpus()
    page_size_kb = os.sysconf("SC_PAGE_SIZE") / 1024.0

    print(f"\n=========================================================================================================")
    print(f" ReloBot System Profiler: Collecting statistics over {duration_sec}s (sample interval: {interval_sec}s)")
    print(f" Platform: Linux ({num_cpus} CPU cores detected) | System clock ticks: {clk_tck} Hz")
    print(f"=========================================================================================================\n")

    containers = get_containers_and_pids()
    if not containers:
        print("Warning: No Docker containers found running.")

    tracked_procs = {}
    for cname, cdata in containers.items():
        for p in cdata["processes"]:
            pid = p["pid"]
            tracked_procs[pid] = {
                "container": cname,
                "node_name": p["node_name"],
                "cmd": p["cmd"],
                "is_composed": "component_container" in p["cmd"]
            }

    prev_sys_total, prev_sys_idle = get_system_cpu()
    prev_proc_stats = {}
    prev_thread_stats = {}

    for pid in tracked_procs:
        st = read_proc_stat(f"/proc/{pid}/stat")
        if st:
            prev_proc_stats[pid] = st
        if tracked_procs[pid]["is_composed"]:
            try:
                tids = os.listdir(f"/proc/{pid}/task")
                prev_thread_stats[pid] = {}
                for tid_str in tids:
                    t_st = read_proc_stat(f"/proc/{pid}/task/{tid_str}/stat")
                    if t_st:
                        prev_thread_stats[pid][int(tid_str)] = t_st
            except Exception:
                pass

    proc_history = defaultdict(list)
    thread_history = defaultdict(lambda: defaultdict(list))
    sys_history = []

    steps = int(duration_sec / interval_sec)
    for step in range(steps):
        time.sleep(interval_sec)
        cur_sys_total, cur_sys_idle = get_system_cpu()
        delta_sys_total = cur_sys_total - prev_sys_total
        delta_sys_idle = cur_sys_idle - prev_sys_idle
        
        sys_cpu_pct = 0.0
        if delta_sys_total > 0:
            sys_cpu_pct = (1.0 - (delta_sys_idle / delta_sys_total)) * 100.0 * num_cpus
        sys_history.append(sys_cpu_pct)

        for pid, meta in tracked_procs.items():
            cur_st = read_proc_stat(f"/proc/{pid}/stat")
            if cur_st and pid in prev_proc_stats:
                prev_st = prev_proc_stats[pid]
                delta_time = cur_st["total_time"] - prev_st["total_time"]
                cpu_pct = 0.0
                if delta_sys_total > 0:
                    cpu_pct = (delta_time / delta_sys_total) * 100.0 * num_cpus
                rss_mb = (cur_st["rss_pages"] * page_size_kb) / 1024.0
                proc_history[pid].append({
                    "cpu_pct": cpu_pct,
                    "rss_mb": rss_mb,
                    "utime": cur_st["utime"],
                    "stime": cur_st["stime"]
                })
                prev_proc_stats[pid] = cur_st
            elif cur_st:
                prev_proc_stats[pid] = cur_st

            if meta["is_composed"]:
                try:
                    cur_tids = os.listdir(f"/proc/{pid}/task")
                    if pid not in prev_thread_stats:
                        prev_thread_stats[pid] = {}
                    for tid_str in cur_tids:
                        tid = int(tid_str)
                        cur_t_st = read_proc_stat(f"/proc/{pid}/task/{tid_str}/stat")
                        if cur_t_st and tid in prev_thread_stats[pid]:
                            prev_t_st = prev_thread_stats[pid][tid]
                            delta_t_time = cur_t_st["total_time"] - prev_t_st["total_time"]
                            t_cpu_pct = 0.0
                            if delta_sys_total > 0:
                                t_cpu_pct = (delta_t_time / delta_sys_total) * 100.0 * num_cpus
                            thread_history[pid][tid].append({
                                "cpu_pct": t_cpu_pct,
                                "total_time": cur_t_st["total_time"],
                                "comm": cur_t_st["comm"]
                            })
                            prev_thread_stats[pid][tid] = cur_t_st
                        elif cur_t_st:
                            prev_thread_stats[pid][tid] = cur_t_st
                except Exception:
                    pass

        prev_sys_total = cur_sys_total
        prev_sys_idle = cur_sys_idle
        
        elapsed = (step + 1) * interval_sec
        sys.stdout.write(f"\r  Sampling Progress: {elapsed:4.1f}s / {duration_sec}s | Instantaneous System CPU: {sys_cpu_pct:5.1f}%")
        sys.stdout.flush()

    print("\n\n" + "=" * 105)
    print(" SAMPLING COMPLETE - SUMMARY & DESCRIPTIVE STATISTICS")
    print("=" * 105)

    proc_summary = []
    for pid, meta in tracked_procs.items():
        samples = proc_history.get(pid, [])
        if not samples:
            continue
        cpus = [s["cpu_pct"] for s in samples]
        rsss = [s["rss_mb"] for s in samples]
        mean_cpu = sum(cpus) / len(cpus)
        max_cpu = max(cpus)
        min_cpu = min(cpus)
        variance = sum((x - mean_cpu) ** 2 for x in cpus) / len(cpus)
        std_cpu = math.sqrt(variance)
        sorted_cpus = sorted(cpus)
        p50_cpu = sorted_cpus[len(sorted_cpus) // 2]
        p95_cpu = sorted_cpus[int(len(sorted_cpus) * 0.95)]
        latest_rss = rsss[-1] if rsss else 0.0
        
        total_delta_ticks = (samples[-1]["utime"] + samples[-1]["stime"]) - (samples[0]["utime"] + samples[0]["stime"])
        total_cpu_sec = total_delta_ticks / clk_tck

        proc_summary.append({
            "pid": pid,
            "container": meta["container"],
            "node_name": meta["node_name"],
            "cmd": meta["cmd"],
            "is_composed": meta["is_composed"],
            "mean_cpu": mean_cpu,
            "std_cpu": std_cpu,
            "min_cpu": min_cpu,
            "max_cpu": max_cpu,
            "p50_cpu": p50_cpu,
            "p95_cpu": p95_cpu,
            "rss_mb": latest_rss,
            "cpu_sec": total_cpu_sec
        })

    proc_summary.sort(key=lambda x: x["mean_cpu"], reverse=True)

    container_summary = defaultdict(lambda: {"mean_cpu": 0.0, "rss_mb": 0.0, "procs": []})
    for p in proc_summary:
        c = p["container"]
        container_summary[c]["mean_cpu"] += p["mean_cpu"]
        container_summary[c]["rss_mb"] += p["rss_mb"]
        container_summary[c]["procs"].append(p)

    return {
        "duration_sec": duration_sec,
        "interval_sec": interval_sec,
        "num_cpus": num_cpus,
        "sys_mean_cpu": sum(sys_history) / len(sys_history) if sys_history else 0.0,
        "container_summary": dict(container_summary),
        "proc_summary": proc_summary,
        "thread_history": thread_history,
        "tracked_procs": tracked_procs
    }

def print_report(report, out_json_path=None):
    print(f"\n1. AGGREGATE RESOURCE USAGE BY DOCKER CONTAINER")
    print("-" * 105)
    print(f"{'CONTAINER NAME':<28} {'MEAN CPU %':<14} {'TOTAL RSS (MB)':<18} {'PROCESS COUNT'}")
    print("-" * 105)
    
    sorted_containers = sorted(report["container_summary"].items(), key=lambda x: x[1]["mean_cpu"], reverse=True)
    for cname, cdata in sorted_containers:
        print(f"{cname:<28} {cdata['mean_cpu']:>8.2f}%      {cdata['rss_mb']:>10.1f} MB        {len(cdata['procs'])} processes")

    print("\n\n2. PER-NODE / PROCESS RESOURCE CONSUMPTION (Sorted by Mean CPU)")
    print("-" * 105)
    header = f"{'NODE / PROCESS NAME':<34} {'CONTAINER':<24} {'MEAN %':<8} {'STD':<6} {'MIN %':<7} {'MAX %':<7} {'P95 %':<7} {'RSS (MB)':<8}"
    print(header)
    print("-" * 105)

    for p in report["proc_summary"]:
        display_name = p["node_name"]
        if p["is_composed"]:
            display_name = f"[Composed] {p['node_name']}"
        print(f"{display_name:<34} {p['container']:<24} {p['mean_cpu']:>6.2f}% {p['std_cpu']:>5.2f} {p['min_cpu']:>6.2f}% {p['max_cpu']:>6.2f}% {p['p95_cpu']:>6.2f}% {p['rss_mb']:>7.1f}MB")

    print("\n\n3. THREAD-LEVEL BREAKDOWN FOR COMPOSED CONTAINERS")
    print("-" * 105)
    
    thread_history = report["thread_history"]
    for p in report["proc_summary"]:
        if p["is_composed"]:
            pid = p["pid"]
            print(f"\n>> Composed Container: {p['node_name']} (PID: {pid}, Container: {p['container']}, Mean CPU: {p['mean_cpu']:.2f}%)")
            t_data = thread_history.get(pid, {})
            
            threads_info = []
            for tid, samples in t_data.items():
                if not samples:
                    continue
                cpus = [s["cpu_pct"] for s in samples]
                mean_c = sum(cpus) / len(cpus)
                max_c = max(cpus)
                min_c = min(cpus)
                comm = samples[-1]["comm"]
                threads_info.append({
                    "tid": tid,
                    "mean_cpu": mean_c,
                    "max_cpu": max_c,
                    "min_cpu": min_c,
                    "comm": comm
                })

            threads_info.sort(key=lambda x: x["mean_cpu"], reverse=True)
            print(f"  {'TID':<10} {'THREAD COMM / NAME':<34} {'MEAN CPU %':<12} {'MAX CPU %':<12}")
            print("  " + "-" * 70)
            for t in threads_info:
                if t["mean_cpu"] > 0.05 or t["max_cpu"] > 1.0:
                    print(f"  {t['tid']:<10} {t['comm']:<34} {t['mean_cpu']:>8.2f}%   {t['max_cpu']:>8.2f}%")

    print("\n" + "=" * 105)
    if out_json_path:
        with open(out_json_path, "w") as f:
            clean_report = {
                "duration_sec": report["duration_sec"],
                "interval_sec": report["interval_sec"],
                "num_cpus": report["num_cpus"],
                "sys_mean_cpu": report["sys_mean_cpu"],
                "container_summary": {k: {"mean_cpu": v["mean_cpu"], "rss_mb": v["rss_mb"]} for k, v in report["container_summary"].items()},
                "proc_summary": report["proc_summary"]
            }
            json.dump(clean_report, f, indent=2)
        print(f"Metrics JSON exported to: {out_json_path}\n")

def main():
    parser = argparse.ArgumentParser(description="ReloBot ROS 2 CPU & Memory Profiler")
    parser.add_argument("duration", nargs="?", type=int, default=30, help="Sampling duration in seconds (default: 30)")
    parser.add_argument("-i", "--interval", type=float, default=1.0, help="Sampling interval in seconds (default: 1.0)")
    parser.add_argument("-o", "--output", type=str, default="/tmp/cpu_profile_report.json", help="Output path for JSON report")
    args = parser.parse_args()

    report = collect_statistics(duration_sec=args.duration, interval_sec=args.interval)
    print_report(report, out_json_path=args.output)

if __name__ == "__main__":
    main()
