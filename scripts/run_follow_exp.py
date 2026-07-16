#!/usr/bin/env python3
"""Leader maneuver experiment + follower tracking recorder."""
import csv
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

LOG_PATH = "/tmp/follower_track_experiment.csv"
SUMMARY_PATH = "/tmp/follower_track_summary.txt"
EVENTS_PATH = "/tmp/follower_track_events.csv"


def yaw_from_q(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class Experiment(Node):
    def __init__(self):
        super().__init__("follower_track_experiment")
        self.buf = Buffer(cache_time=Duration(seconds=30))
        self.listener = TransformListener(self.buf, self)
        self.cmd_pub = self.create_publisher(
            TwistStamped, "/leader/diff_drive_controller/cmd_vel", 10)
        self.f_cmd = None
        self.l_scan_min = float("inf")
        self.f_scan_min = float("inf")
        self.f_scan_min_angle = 0.0
        self.create_subscription(
            TwistStamped, "/follower/diff_drive_controller/cmd_vel",
            self._on_fcmd, 10)
        self.create_subscription(
            LaserScan, "/follower/scan", self._on_fscan, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, "/leader/scan", self._on_lscan, qos_profile_sensor_data)

        self.phase = "warmup"
        self.t0 = time.time()
        self.rows = []
        self.events = []
        self.leader_path = deque(maxlen=5000)

        # From spawn (0,0): stay in open center aisle (|y|<1.2, shelves at ±2).
        # Sequence: straight +x, 90° pivot, short spur, then curved loop.
        self.schedule = [
            ("straight", 8.0, 0.45, 0.00),    # ~3.6 m along +x
            ("pivot_90", 3.0, 0.00, 0.55),     # ~94° CCW
            ("after_turn", 3.5, 0.35, 0.00),   # ~1.2 m along +y (still < shelf)
            ("pivot_back", 3.0, 0.00, -0.55),  # face roughly +x again
            ("loop_arc", 14.0, 0.32, 0.38),    # curved loop
            ("coast", 3.0, 0.00, 0.00),
        ]
        self.sched_i = 0
        self.phase_t0 = None
        self.started = False
        self.done = False

        self.create_timer(0.05, self.tick)

    def _on_fcmd(self, msg):
        self.f_cmd = msg

    def _on_fscan(self, msg):
        best = float("inf")
        bang = 0.0
        for i, r in enumerate(msg.ranges):
            if math.isfinite(r) and r > 0.05 and r < best:
                best = r
                bang = msg.angle_min + i * msg.angle_increment
        self.f_scan_min = best
        self.f_scan_min_angle = bang

    def _on_lscan(self, msg):
        vals = [r for r in msg.ranges if math.isfinite(r) and r > 0.05]
        self.l_scan_min = min(vals) if vals else float("inf")

    def lookup(self, frame):
        t = self.buf.lookup_transform("map", frame, Time())
        x = t.transform.translation.x
        y = t.transform.translation.y
        yaw = yaw_from_q(t.transform.rotation)
        return x, y, yaw

    def crosstrack(self, fx, fy):
        if len(self.leader_path) < 2:
            return float("nan")
        pts = list(self.leader_path)
        best = float("inf")
        for i in range(1, len(pts)):
            x0, y0 = pts[i - 1]
            x1, y1 = pts[i]
            dx, dy = x1 - x0, y1 - y0
            L2 = dx * dx + dy * dy
            if L2 < 1e-9:
                d = math.hypot(fx - x0, fy - y0)
            else:
                u = max(0.0, min(1.0, ((fx - x0) * dx + (fy - y0) * dy) / L2))
                d = math.hypot(fx - (x0 + u * dx), fy - (y0 + u * dy))
            if d < best:
                best = d
        return best

    def publish_cmd(self, lin, ang):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "leader/base_footprint"
        msg.twist.linear.x = float(lin)
        msg.twist.angular.z = float(ang)
        self.cmd_pub.publish(msg)

    def tick(self):
        if self.done:
            return
        now = time.time()
        try:
            lx, ly, lyaw = self.lookup("leader/base_footprint")
            fx, fy, fyaw = self.lookup("follower/base_footprint")
        except Exception:
            return

        if not self.started:
            if now - self.t0 < 2.0:
                self.phase = "warmup"
                self.publish_cmd(0.0, 0.0)
                return
            self.started = True
            self.phase_t0 = now
            self.sched_i = 0
            self.phase = self.schedule[0][0]
            self.get_logger().info(f"START phase={self.phase}")

        name, dur, lin, ang = self.schedule[self.sched_i]
        if now - self.phase_t0 >= dur:
            self.sched_i += 1
            if self.sched_i >= len(self.schedule):
                self.publish_cmd(0.0, 0.0)
                self.finish()
                return
            self.phase_t0 = now
            name, dur, lin, ang = self.schedule[self.sched_i]
            self.phase = name
            self.get_logger().info(f"PHASE -> {self.phase}")

        self.publish_cmd(lin, ang)

        if (not self.leader_path or
                math.hypot(lx - self.leader_path[-1][0], ly - self.leader_path[-1][1]) > 0.05):
            self.leader_path.append((lx, ly))

        dx = lx - fx
        dy = ly - fy
        dist = math.hypot(dx, dy)
        bearing = wrap(math.atan2(dy, dx) - fyaw)
        cte = self.crosstrack(fx, fy)
        f_lin = self.f_cmd.twist.linear.x if self.f_cmd else float("nan")
        f_ang = self.f_cmd.twist.angular.z if self.f_cmd else float("nan")

        row = {
            "t": now - self.t0,
            "phase": self.phase,
            "lx": lx, "ly": ly, "lyaw": lyaw,
            "fx": fx, "fy": fy, "fyaw": fyaw,
            "dist": dist, "bearing": bearing, "cte": cte,
            "f_lin": f_lin, "f_ang": f_ang,
            "f_obst_min": self.f_scan_min,
            "f_obst_ang": self.f_scan_min_angle,
            "l_obst_min": self.l_scan_min,
            "cmd_lin": lin, "cmd_ang": ang,
        }
        self.rows.append(row)

        if self.phase not in ("warmup", "coast", "pivot_90") and math.isfinite(f_ang):
            if math.isfinite(cte) and cte > 0.55 and abs(f_ang) > 0.6:
                kind = "pf_avoid" if self.f_scan_min < 0.85 else "unexpected_veer"
                if (not self.events or
                        now - self.t0 - self.events[-1]["t"] > 0.5):
                    ev = dict(row)
                    ev["kind"] = kind
                    self.events.append(ev)
                    self.get_logger().warn(
                        f"{kind.upper()} t={ev['t']:.2f} phase={self.phase} "
                        f"cte={cte:.3f} f_ang={f_ang:.3f} "
                        f"obst={self.f_scan_min:.3f}m "
                        f"@{math.degrees(self.f_scan_min_angle):.0f}deg "
                        f"L=({lx:.2f},{ly:.2f}) F=({fx:.2f},{fy:.2f})")

    def finish(self):
        if self.done:
            return
        self.done = True
        self.get_logger().info("DONE — writing logs")

        if self.rows:
            with open(LOG_PATH, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
                w.writeheader()
                w.writerows(self.rows)

        if self.events:
            with open(EVENTS_PATH, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=list(self.events[0].keys()))
                w.writeheader()
                w.writerows(self.events)

        phases = {}
        for r in self.rows:
            phases.setdefault(r["phase"], []).append(r)

        def stats(xs):
            if not xs:
                return "n/a"
            return f"mean={sum(xs)/len(xs):.3f} max={max(xs):.3f} min={min(xs):.3f}"

        lines = []
        n = len(self.rows)
        dur = self.rows[-1]["t"] if self.rows else 0.0
        lines.append("=== Follower tracking experiment summary ===")
        lines.append(f"samples={n} duration={dur:.1f}s")

        for ph, rs in phases.items():
            ctes = [r["cte"] for r in rs if math.isfinite(r["cte"])]
            angs = [abs(r["f_ang"]) for r in rs if math.isfinite(r["f_ang"])]
            dists = [r["dist"] for r in rs]
            obst = [r["f_obst_min"] for r in rs if math.isfinite(r["f_obst_min"])]
            lines.append(f"\n[{ph}] n={len(rs)}")
            lines.append(f"  crosstrack: {stats(ctes)}")
            lines.append(f"  |f_ang|:    {stats(angs)}")
            lines.append(f"  dist_lead:  {stats(dists)}")
            lines.append(f"  obst_min:   {stats(obst)}")
            r0, r1 = rs[0], rs[-1]
            lines.append(
                f"  start L=({r0['lx']:.2f},{r0['ly']:.2f}) F=({r0['fx']:.2f},{r0['fy']:.2f})")
            lines.append(
                f"  end   L=({r1['lx']:.2f},{r1['ly']:.2f}) F=({r1['fx']:.2f},{r1['fy']:.2f})")

        veers = [e for e in self.events if e.get("kind") in ("pf_avoid", "unexpected_veer")]
        lines.append(f"\n=== Veer events ({len(veers)}) ===")
        if not veers:
            lines.append("  (none flagged)")
        for e in veers:
            lines.append(
                f"  t={e['t']:.2f}s {e['kind']} phase={e['phase']} "
                f"cte={e['cte']:.3f} f_ang={e['f_ang']:.3f} "
                f"obst={e['f_obst_min']:.3f}m "
                f"ang={math.degrees(e['f_obst_ang']):.0f}deg "
                f"L=({e['lx']:.2f},{e['ly']:.2f}) F=({e['fx']:.2f},{e['fy']:.2f}) "
                f"bearing={math.degrees(e['bearing']):.0f}deg"
            )

        unexpected = [e for e in veers if e["kind"] == "unexpected_veer"]
        pf = [e for e in veers if e["kind"] == "pf_avoid"]
        moving = [r for r in self.rows
                  if r["phase"] in ("straight", "after_turn", "loop_arc", "recenter_drive")]
        max_cte = max((r["cte"] for r in moving if math.isfinite(r["cte"])), default=float("nan"))
        # Also: open-space veer = high |f_ang| while obst far and leader going straight
        open_space_swerves = []
        for r in self.rows:
            if r["phase"] != "straight":
                continue
            if (math.isfinite(r["f_ang"]) and abs(r["f_ang"]) > 0.8
                    and r["f_obst_min"] > 1.2 and math.isfinite(r["cte"]) and r["cte"] > 0.35):
                open_space_swerves.append(r)

        lines.append("\n=== Judgment inputs ===")
        lines.append(f"unexpected_veer_events={len(unexpected)} pf_avoid_events={len(pf)}")
        lines.append(
            f"max_crosstrack_while_moving={max_cte:.3f}"
            if math.isfinite(max_cte) else "max_crosstrack=n/a")
        lines.append(f"open_space_swerve_samples_during_straight={len(open_space_swerves)}")
        if open_space_swerves:
            s = open_space_swerves[0]
            lines.append(
                f"  first open-space swerve t={s['t']:.2f} "
                f"cte={s['cte']:.3f} f_ang={s['f_ang']:.3f} "
                f"obst={s['f_obst_min']:.3f} "
                f"L=({s['lx']:.2f},{s['ly']:.2f}) F=({s['fx']:.2f},{s['fy']:.2f})")

        text = "\n".join(lines) + "\n"
        with open(SUMMARY_PATH, "w") as f:
            f.write(text)
        print(text)
        self.get_logger().info(f"Wrote {LOG_PATH} and {SUMMARY_PATH}")
        # Request shutdown outside timer
        self.create_timer(0.1, lambda: rclpy.shutdown())


def main():
    rclpy.init()
    node = Experiment()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
