#!/usr/bin/env python3
import csv, math
from collections import defaultdict

rows = list(csv.DictReader(open("/tmp/follower_track_experiment.csv")))

def g(r, k):
    return float(r[k])

print(f"total rows={len(rows)}")
ph = defaultdict(list)
for r in rows:
    ph[r["phase"]].append(r)

for name, rs in ph.items():
    path = 0.0
    for i in range(1, len(rs)):
        path += math.hypot(g(rs[i], "lx") - g(rs[i - 1], "lx"),
                           g(rs[i], "ly") - g(rs[i - 1], "ly"))
    print(f"{name}: leader_path={path:.2f}m  "
          f"L {g(rs[0],'lx'):.2f},{g(rs[0],'ly'):.2f} -> {g(rs[-1],'lx'):.2f},{g(rs[-1],'ly'):.2f}  "
          f"F {g(rs[0],'fx'):.2f},{g(rs[0],'fy'):.2f} -> {g(rs[-1],'fx'):.2f},{g(rs[-1],'fy'):.2f}")

straight = [r for r in rows if r["phase"] == "straight"]
print("\n=== STRAIGHT |f_ang| histogram ===")
angs = [abs(g(r, "f_ang")) for r in straight if math.isfinite(g(r, "f_ang"))]
bins = [0, 0.2, 0.5, 0.8, 1.2, 2, 3, 10]
for i in range(len(bins) - 1):
    c = sum(1 for a in angs if bins[i] <= a < bins[i + 1])
    print(f"  [{bins[i]},{bins[i+1]}): {c}")

print("\nTop |f_ang| during straight:")
scored = sorted(straight, key=lambda r: -abs(g(r, "f_ang")))[:8]
for r in scored:
    print(f"  t={g(r,'t'):.2f} f_ang={g(r,'f_ang'):+.3f} cte={g(r,'cte'):.3f} "
          f"obst={g(r,'f_obst_min'):.3f}@{math.degrees(g(r,'f_obst_ang')):.0f}deg "
          f"dist={g(r,'dist'):.2f} L=({g(r,'lx'):.2f},{g(r,'ly'):.2f}) F=({g(r,'fx'):.2f},{g(r,'fy'):.2f})")

moving = [r for r in rows if r["phase"] in ("straight", "after_turn", "loop_arc")]
print("\nHighest crosstrack:")
topcte = sorted([r for r in moving if math.isfinite(g(r, "cte"))],
                key=lambda r: -g(r, "cte"))[:10]
for r in topcte:
    print(f"  t={g(r,'t'):.2f} {r['phase']} cte={g(r,'cte'):.3f} f_ang={g(r,'f_ang'):+.3f} "
          f"obst={g(r,'f_obst_min'):.3f} L=({g(r,'lx'):.2f},{g(r,'ly'):.2f}) F=({g(r,'fx'):.2f},{g(r,'fy'):.2f})")

ds = [g(r, "dist") for r in moving]
print(f"\nDist to leader moving: mean={sum(ds)/len(ds):.3f} min={min(ds):.3f} max={max(ds):.3f}")

hi = [r for r in moving if math.isfinite(g(r, "f_ang")) and abs(g(r, "f_ang")) > 1.0]
near = sum(1 for r in hi if g(r, "f_obst_min") < 0.85)
far = [r for r in hi if g(r, "f_obst_min") >= 1.2]
print(f"|f_ang|>1.0: n={len(hi)} obst<0.85={near} obst>=1.2={len(far)}")
for r in far[:6]:
    print(f"  FAR+SWERVE t={g(r,'t'):.2f} {r['phase']} f_ang={g(r,'f_ang'):+.3f} "
          f"obst={g(r,'f_obst_min'):.3f} cte={g(r,'cte'):.3f} "
          f"L=({g(r,'lx'):.2f},{g(r,'ly'):.2f}) F=({g(r,'fx'):.2f},{g(r,'fy'):.2f})")

# Lateral drift rate during straight: |fy - ly| over time
if straight:
    ye = [g(r, "fy") - g(r, "ly") for r in straight]
    print(f"\nStraight lateral (fy-ly): mean={sum(ye)/len(ye):.3f} "
          f"max_abs={max(abs(v) for v in ye):.3f}")
    print(f"Straight cte: mean={sum(g(r,'cte') for r in straight)/len(straight):.3f} "
          f"max={max(g(r,'cte') for r in straight):.3f}")
