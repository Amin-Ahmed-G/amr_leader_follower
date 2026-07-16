#!/usr/bin/env python3
"""Audit localization_source:=slam for clock/TF continuity (ground_truth-parity).

Run while the sim is up with localization_source:=slam. Writes
/tmp/slam_mode_audit.txt and prints a summary.
"""
import math
import time
from collections import defaultdict

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

OUT = "/tmp/slam_mode_audit.txt"
PAIRS = [
    ("map", "leader/base_footprint"),
    ("map", "follower/base_footprint"),
    ("map", "leader/odom"),
    ("map", "follower/odom"),
    ("leader/odom", "leader/base_footprint"),
    ("follower/odom", "follower/base_footprint"),
]


def main():
    lines = []

    def log(s=""):
        print(s)
        lines.append(s)

    rclpy.init()
    n = Node("slam_mode_audit")
    n.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    buf = Buffer(cache_time=Duration(seconds=60))
    TransformListener(buf, n)

    end = time.time() + 3.0
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)

    log("=== SLAM MODE AUDIT ===")
    log(f"sim_time_now={n.get_clock().now().nanoseconds * 1e-9:.3f}s")

    hist = {p: [] for p in PAIRS}
    errs = defaultdict(list)
    t0 = time.time()
    while time.time() - t0 < 10.0:
        rclpy.spin_once(n, timeout_sec=0.05)
        for parent, child in PAIRS:
            try:
                t = buf.lookup_transform(parent, child, Time())
                stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
                x = t.transform.translation.x
                y = t.transform.translation.y
                hist[(parent, child)].append((stamp, x, y))
            except Exception as e:
                errs[(parent, child)].append(str(e).split("\n")[0][:120])

    log("\n=== TF continuity (10s sample) ===")
    for parent, child in PAIRS:
        samples = hist[(parent, child)]
        e = errs[(parent, child)]
        if not samples:
            log(f"FAIL {parent}->{child}: 0 samples, errors={len(e)}")
            if e:
                log(f"  last_err: {e[-1]}")
            continue
        stamps = [s[0] for s in samples]
        xs = [s[1] for s in samples]
        ys = [s[2] for s in samples]
        backjumps = sum(1 for i in range(1, len(stamps)) if stamps[i] + 1e-6 < stamps[i - 1])
        gaps = sum(1 for i in range(1, len(stamps)) if stamps[i] - stamps[i - 1] > 0.5)
        pos_jumps = sum(
            1 for i in range(1, len(samples))
            if math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]) > 0.5
        )
        rate = len(samples) / max(1e-6, stamps[-1] - stamps[0]) if len(samples) > 1 else 0.0
        log(
            f"OK  {parent}->{child}: n={len(samples)} rate≈{rate:.1f}Hz "
            f"backjumps={backjumps} gaps>0.5s={gaps} pos_jumps>0.5m={pos_jumps} "
            f"end=({xs[-1]:.3f},{ys[-1]:.3f}) lookup_errs={len(e)}"
        )
        for ee in e[:2]:
            log(f"  err: {ee}")

    # Stamp lead: map->odom vs odom->base (AMCL transform_tolerance signature)
    log("\n=== map->odom stamp lead vs odom->base (5s) ===")
    leads = {"leader": [], "follower": []}
    t0 = time.time()
    while time.time() - t0 < 5.0:
        rclpy.spin_once(n, timeout_sec=0.05)
        for ns in ("leader", "follower"):
            try:
                t_mo = buf.lookup_transform("map", f"{ns}/odom", Time())
                t_ob = buf.lookup_transform(f"{ns}/odom", f"{ns}/base_footprint", Time())
                smo = t_mo.header.stamp.sec + t_mo.header.stamp.nanosec * 1e-9
                sob = t_ob.header.stamp.sec + t_ob.header.stamp.nanosec * 1e-9
                leads[ns].append(smo - sob)
            except Exception:
                pass
    for ns, xs in leads.items():
        if xs:
            log(
                f"{ns}: map->odom stamp - odom->base stamp "
                f"mean={sum(xs)/len(xs):.4f} max={max(xs):.4f} min={min(xs):.4f} n={len(xs)}"
            )
        else:
            log(f"{ns}: no stamp samples")

    log("\n=== Follower composed lookup (5s) ===")
    ok = fail = 0
    kinds = defaultdict(int)
    t0 = time.time()
    while time.time() - t0 < 5.0:
        rclpy.spin_once(n, timeout_sec=0.05)
        try:
            buf.lookup_transform("map", "follower/base_footprint", Time())
            ok += 1
        except Exception as e:
            fail += 1
            msg = str(e)
            if "past" in msg:
                kinds["past"] += 1
            elif "future" in msg:
                kinds["future"] += 1
            elif "does not exist" in msg:
                kinds["missing"] += 1
            else:
                kinds["other"] += 1
    log(f"map->follower/base_footprint: ok={ok} fail={fail} kinds={dict(kinds)}")

    log("\n=== Judgment ===")
    f_errs = len(errs[("map", "follower/base_footprint")])
    l_errs = len(errs[("map", "leader/base_footprint")])
    log(f"leader lookup_errs={l_errs} follower lookup_errs={f_errs}")
    if leads["follower"]:
        mean_lead = sum(leads["follower"]) / len(leads["follower"])
        if mean_lead > 1.5:
            log(
                f"NOTE: follower AMCL map->odom stamped ~{mean_lead:.2f}s ahead of "
                "odom->base (transform_tolerance); can cause intermittent Time() failures"
            )

    text = "\n".join(lines) + "\n"
    with open(OUT, "w") as f:
        f.write(text)
    log(f"\nWrote {OUT}")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
