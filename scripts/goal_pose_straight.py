#!/usr/bin/env python3
"""Straight-line baseline via /leader/goal_pose (Nav2), no raw teleop.

Publishes a pose goal ~GOAL_DIST meters ahead of the leader in map frame,
then logs leader/follower TF + follower cmd_vel until the leader reaches the
goal (or timeout). Use this to compare against Twist teleop without
cmd_vel contention with Nav2.
"""
import csv
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
from lifecycle_msgs.srv import GetState

LOG_PATH = "/tmp/goal_pose_straight.csv"
SUMMARY_PATH = "/tmp/goal_pose_straight_summary.txt"
GOAL_DIST = 3.0  # meters ahead along current leader yaw
ARRIVE_THRESH = 0.35
TIMEOUT_S = 60.0


def yaw_from_q(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


MAP_WAIT_S = 45.0
NAV_WAIT_S = 90.0


class GoalStraight(Node):
    def __init__(self):
        super().__init__("goal_pose_straight")
        self.buf = Buffer(cache_time=Duration(seconds=30))
        TransformListener(self.buf, self)
        self.goal_pub = self.create_publisher(PoseStamped, "/leader/goal_pose", 10)
        self.f_cmd = None
        self.create_subscription(
            TwistStamped, "/follower/diff_drive_controller/cmd_vel",
            lambda m: setattr(self, "f_cmd", m), 10)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, 10)
        self.nav_client = self.create_client(GetState, "/leader/bt_navigator/get_state")
        self.rows = []
        self.t0 = time.time()
        self.goal = None
        self.sent = False
        self.done = False
        self.map_ready = False
        self.nav_ready = False
        self.map_meta = None
        self.create_timer(0.05, self.tick)
        self.create_timer(2.0, self.check_nav)

    def _on_map(self, msg):
        if msg.info.width > 0 and msg.info.height > 0:
            self.map_ready = True
            self.map_meta = (msg.info.width, msg.info.height, msg.info.resolution)

    def check_nav(self):
        if self.nav_ready or self.done:
            return
        if not self.nav_client.wait_for_service(timeout_sec=0.5):
            return
        req = GetState.Request()
        fut = self.nav_client.call_async(req)
        fut.add_done_callback(self._nav_state_cb)

    def _nav_state_cb(self, fut):
        try:
            resp = fut.result()
            if resp.current_state.id == 3:  # active
                self.nav_ready = True
        except Exception:
            pass

    def lookup(self, frame):
        t = self.buf.lookup_transform("map", frame, Time())
        return (
            t.transform.translation.x,
            t.transform.translation.y,
            yaw_from_q(t.transform.rotation),
        )

    def tick(self):
        if self.done:
            return
        now = time.time()
        try:
            lx, ly, lyaw = self.lookup("leader/base_footprint")
            fx, fy, fyaw = self.lookup("follower/base_footprint")
        except Exception:
            return

        # Warm up TF / settle at spawn
        if now - self.t0 < 3.0:
            return

        if not self.sent:
            # Nav2 global costmap needs /map (SLAM or map_server). In
            # ground_truth mode there is no map publisher — skip until one
            # appears so goal_pose tests fail fast with a clear log line.
            if not self.map_ready:
                return
            if not self.nav_ready:
                return
            gx = lx + GOAL_DIST * math.cos(lyaw)
            gy = ly + GOAL_DIST * math.sin(lyaw)
            self.goal = (gx, gy, lyaw)
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose.position.x = gx
            msg.pose.position.y = gy
            msg.pose.orientation.z = math.sin(lyaw / 2.0)
            msg.pose.orientation.w = math.cos(lyaw / 2.0)
            # Latch a few times so Nav2 reliably sees it
            for _ in range(5):
                self.goal_pub.publish(msg)
            self.sent = True
            self.get_logger().info(
                f"Sent goal_pose ({gx:.2f},{gy:.2f}) from leader ({lx:.2f},{ly:.2f})")

        f_ang = self.f_cmd.twist.angular.z if self.f_cmd else float("nan")
        f_lin = self.f_cmd.twist.linear.x if self.f_cmd else float("nan")
        dx = fx - lx
        dy = fy - ly
        lat = -dx * math.sin(lyaw) + dy * math.cos(lyaw)
        dist_goal = math.hypot(lx - self.goal[0], ly - self.goal[1]) if self.goal else float("nan")
        self.rows.append({
            "t": now - self.t0,
            "lx": lx, "ly": ly, "lyaw": lyaw,
            "fx": fx, "fy": fy, "fyaw": fyaw,
            "lat_err": lat,
            "dist_lead": math.hypot(dx, dy),
            "dist_goal": dist_goal,
            "f_lin": f_lin, "f_ang": f_ang,
        })

        if dist_goal < ARRIVE_THRESH:
            self.get_logger().info(f"Arrived (dist_goal={dist_goal:.3f})")
            self.finish(ok=True)
        elif now - self.t0 > TIMEOUT_S:
            self.get_logger().warn("Timeout waiting for goal")
            self.finish(ok=False)
        elif (not self.sent and now - self.t0 > MAP_WAIT_S and not self.map_ready):
            self.get_logger().error(
                "No /map after {:.0f}s — goal_pose Nav2 needs SLAM or map_server "
                "(ground_truth has no map publisher)".format(MAP_WAIT_S))
            self.finish(ok=False)
        elif (not self.sent and now - self.t0 > NAV_WAIT_S and not self.nav_ready):
            self.get_logger().error("Nav2 bt_navigator not active after {:.0f}s".format(NAV_WAIT_S))
            self.finish(ok=False)

    def finish(self, ok):
        if self.done:
            return
        self.done = True
        if self.rows:
            with open(LOG_PATH, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
                w.writeheader()
                w.writerows(self.rows)

        moving = [r for r in self.rows if r["t"] >= 3.0 and self.sent]
        lats = [abs(r["lat_err"]) for r in moving]
        dists = [r["dist_lead"] for r in moving]
        angs = [abs(r["f_ang"]) for r in moving if math.isfinite(r["f_ang"])]
        lines_travel = 0.0
        for i in range(1, len(moving)):
            lines_travel += math.hypot(
                moving[i]["lx"] - moving[i - 1]["lx"],
                moving[i]["ly"] - moving[i - 1]["ly"],
            )
        lines = [
            "=== goal_pose straight baseline ===",
            f"ok={ok} samples={len(self.rows)} duration={self.rows[-1]['t']:.1f}s" if self.rows else "no data",
            f"goal={self.goal}",
            f"map={self.map_meta}",
            f"leader_path={lines_travel:.3f}m",
            f"|lat_err| mean={sum(lats)/len(lats):.4f} max={max(lats):.4f} min={min(lats):.4f}" if lats else "lat n/a",
            f"dist_lead mean={sum(dists)/len(dists):.4f} max={max(dists):.4f} min={min(dists):.4f}" if dists else "dist n/a",
            f"|f_ang| mean={sum(angs)/len(angs):.4f} max={max(angs):.4f}" if angs else "f_ang n/a",
            f"start L=({self.rows[0]['lx']:.2f},{self.rows[0]['ly']:.2f}) "
            f"F=({self.rows[0]['fx']:.2f},{self.rows[0]['fy']:.2f})" if self.rows else "",
            f"end   L=({self.rows[-1]['lx']:.2f},{self.rows[-1]['ly']:.2f}) "
            f"F=({self.rows[-1]['fx']:.2f},{self.rows[-1]['fy']:.2f}) "
            f"dist_goal={self.rows[-1]['dist_goal']:.3f}" if self.rows else "",
        ]
        text = "\n".join(lines) + "\n"
        with open(SUMMARY_PATH, "w") as f:
            f.write(text)
        print(text)
        self.create_timer(0.1, lambda: rclpy.shutdown())


def main():
    rclpy.init()
    node = GoalStraight()
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
