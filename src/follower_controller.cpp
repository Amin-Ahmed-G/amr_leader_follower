#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <cmath>
#include <chrono>
#include <vector>
#include <limits>
#include <algorithm>

using namespace std::chrono_literals;

// ============================================================
// FollowerController
//   – Breadcrumb trail following (lookahead-based pure pursuit)
//   – Obstacle avoidance via repulsive + tangential potential
//     field computed from the follower's Lidar scan
//   – Hysteresis on tangential force side to prevent flip-flop
//   – Lidar beams transformed into base_footprint frame before
//     comparing against leader position (fixes leader-filter
//     mismatch when lidar is offset/rotated from base_footprint)
// ============================================================
class FollowerController : public rclcpp::Node
{
public:
  FollowerController()
  : Node("follower_controller"),
    tf_buffer_(this->get_clock(), tf2::durationFromSec(30.0)),
    tf_listener_(tf_buffer_, this)
  {
    // ---- Parameters (tunable at launch time) ----------------
    k_rep_  = this->declare_parameter<double>("k_rep",  0.8);
    k_tan_  = this->declare_parameter<double>("k_tan",  1.2);
    d_inf_  = this->declare_parameter<double>("d_inf",  0.5);
    d_safe_ = this->declare_parameter<double>("d_safe", 0.35);
    lookahead_ = this->declare_parameter<double>("lookahead", 0.6);
    // Combined physical radius of leader + follower footprints. Both
    // d_desired and the emergency-stop distance are measured center-to-
    // center, so without this padding the two robot BODIES can end up
    // much closer than the distances suggest (or literally touching).
    // Set this to (leader_footprint_radius + follower_footprint_radius)
    // from your 2_wheel.xacro — e.g. two 0.20m-radius robots -> 0.40.
    footprint_clearance_ = this->declare_parameter<double>("footprint_clearance", 0.40);

    // ---- Publisher ------------------------------------------
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/follower/diff_drive_controller/cmd_vel", 10);

    // ---- Lidar subscriber -----------------------------------
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/follower/scan",
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_scan_ = msg;
      });

    // ---- Control loop at 20 Hz ------------------------------
    timer_ = this->create_wall_timer(
      50ms, std::bind(&FollowerController::ControlLoop, this));

    RCLCPP_INFO(this->get_logger(),
      "AMR Breadcrumb+PotentialField Follower Controller initialized.");
  }

private:
  // ----------------------------------------------------------
  // Cache the static lidar -> base_footprint transform once,
  // the first time we see a scan. Needed because raw scan beams
  // are in the lidar's own frame, which may be offset/rotated
  // from base_footprint.
  // ----------------------------------------------------------
  void CacheLidarTransform(const std::string & lidar_frame)
  {
    if (lidar_tf_cached_) return;
    try {
      geometry_msgs::msg::TransformStamped t = tf_buffer_.lookupTransform(
        "follower/base_footprint", lidar_frame,
        tf2::TimePointZero, tf2::durationFromSec(0.5));
      tf2::fromMsg(t.transform, lidar_to_base_tf_);
      lidar_tf_cached_ = true;
      RCLCPP_INFO(this->get_logger(), "Cached lidar->base_footprint transform.");
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Waiting for lidar transform: %s", ex.what());
    }
  }

  // ----------------------------------------------------------
  // Compute obstacle avoidance force in the follower's LOCAL frame.
  // Returns {angular_force, speed_reduction_factor, min_distance}.
  //   angular_force          – rad/s additive term on angular velocity
  //   speed_reduction_factor – in [0,1], 0 = full stop, 1 = no slowdown
  //   min_distance           – closest obstacle distance seen this cycle
  // ----------------------------------------------------------
  struct ObstForce { double angular; double speed_factor; double min_distance; };

  ObstForce ComputeObstacleForce(double follower_yaw, double lx_rel, double ly_rel)
  {
    ObstForce result{0.0, 1.0, std::numeric_limits<double>::infinity()};
    if (!latest_scan_) return result;

    const auto & scan = *latest_scan_;
    CacheLidarTransform(scan.header.frame_id);
    if (!lidar_tf_cached_) return result;  // wait until transform is available

    int n = static_cast<int>(scan.ranges.size());
    if (n == 0) return result;

    // ---- Cluster beams into 15-degree bins ------------------
    const double bin_width_rad = 15.0 * M_PI / 180.0;
    int num_bins = static_cast<int>(std::ceil(
      (scan.angle_max - scan.angle_min) / bin_width_rad));
    std::vector<double> bin_min(num_bins, std::numeric_limits<double>::infinity());
    std::vector<double> bin_angle(num_bins, 0.0);

    for (int i = 0; i < n; ++i) {
      double r = scan.ranges[i];
      if (!std::isfinite(r) || r < 0.01) continue;
      double angle = scan.angle_min + i * scan.angle_increment;

      // Beam point in the LIDAR's own frame
      tf2::Vector3 beam_lidar(r * std::cos(angle), r * std::sin(angle), 0.0);
      // Transform into base_footprint frame so it's comparable to lx_rel/ly_rel
      tf2::Vector3 beam_base = lidar_to_base_tf_ * beam_lidar;

      // Filter out beams that hit the leader (within 0.45m of leader's center)
      double dist_to_leader_endpoint = std::sqrt(
        (beam_base.x() - lx_rel) * (beam_base.x() - lx_rel) +
        (beam_base.y() - ly_rel) * (beam_base.y() - ly_rel));
      if (dist_to_leader_endpoint < 0.45) {
        continue;
      }

      // Bin using the base-frame angle so it's consistent with the rest
      // of the force calculation (which operates in base_footprint frame)
      double base_angle = std::atan2(beam_base.y(), beam_base.x());
      int bin = static_cast<int>((base_angle - scan.angle_min) / bin_width_rad);
      if (bin < 0 || bin >= num_bins) continue;
      double d = beam_base.length();
      if (d < bin_min[bin]) {
        bin_min[bin] = d;
        bin_angle[bin] = base_angle;
      }
    }

    // ---- Determine dominant obstacle side for this cycle ----
    // (single decision based on closest bin, instead of letting
    // each bin flip independently — prevents left/right cancellation)
    double closest_angle = 0.0;
    double closest_d = std::numeric_limits<double>::infinity();
    for (int b = 0; b < num_bins; ++b) {
      double d = bin_min[b];
      if (!std::isfinite(d) || d >= d_inf_) continue;
      if (d < closest_d) {
        closest_d = d;
        closest_angle = bin_angle[b];
      }
    }

    double raw_sign = (closest_angle < 0.0) ? 1.0 : -1.0;

    // ---- Hysteresis: only accept a side-flip after kFlipThreshold
    // consecutive cycles favoring the new side. Prevents a single
    // noisy frame (e.g. leader crossing near an obstacle) from
    // causing the avoidance direction to flip abruptly.
    // Track a running vote toward whichever side disagrees with the
    // committed sign. A single noisy frame that happens to agree with
    // the stale side should NOT wipe out real progress toward a
    // genuine flip — only a sustained run of agreement should.
    if (raw_sign != last_tan_sign_) {
      flip_streak_ = std::min(flip_streak_ + 1, kFlipThreshold);
      if (flip_streak_ >= kFlipThreshold) {
        last_tan_sign_ = raw_sign;
        flip_streak_ = 0;
      }
    } else {
      flip_streak_ = std::max(flip_streak_ - 1, 0);
    }
    double tan_sign = last_tan_sign_;

    // Detect symmetric boxed-in case: obstacles on both sides within d_inf.
    // In this situation there is no clean side to dodge toward, so suppress
    // the tangential (steering) term entirely and rely on repulsive + slowdown only.
    bool obstacle_left = false, obstacle_right = false;
    for (int b = 0; b < num_bins; ++b) {
      double d = bin_min[b];
      if (!std::isfinite(d) || d >= d_inf_) continue;
      if (bin_angle[b] > 0.15) obstacle_left = true;
      else if (bin_angle[b] < -0.15) obstacle_right = true;
    }
    bool boxed_in = obstacle_left && obstacle_right;
    if (boxed_in) {
      tan_sign = 0.0;  // no tangential push when walled on both sides
    }

    // ---- Sum repulsive + tangential forces ------------------
    double fx_sum = 0.0;   // local frame x (forward)
    double fy_sum = 0.0;   // local frame y (left)
    double min_d = std::numeric_limits<double>::infinity();

    for (int b = 0; b < num_bins; ++b) {
      double d = bin_min[b];
      if (!std::isfinite(d) || d >= d_inf_) continue;
      if (d < min_d) min_d = d;
      double a = bin_angle[b];  // base_footprint-frame angle

      // Magnitude
      double mag = (1.0 / d - 1.0 / d_inf_) / (d * d);

      // Repulsive: directly away from obstacle
      double rep_x = -mag * std::cos(a);
      double rep_y = -mag * std::sin(a);

      // Tangential: perpendicular to repulsive, using the
      // stabilized (hysteresis-protected) sign for this whole cycle
      double tan_x = -mag * std::sin(a) * tan_sign;
      double tan_y =  mag * std::cos(a) * tan_sign;

      fx_sum += k_rep_ * rep_x + k_tan_ * tan_x;
      fy_sum += k_rep_ * rep_y + k_tan_ * tan_y;
    }

    // ---- Convert field to angular velocity additive term ----
    result.angular = std::max(std::min(fy_sum, 1.8), -1.8);

    result.min_distance = min_d;

    // ---- Speed reduction based on closest obstacle ----------
    if (min_d < d_safe_) {
      result.speed_factor = 0.0;           // full stop
    } else if (min_d < d_inf_) {
      result.speed_factor = (min_d - d_safe_) / (d_inf_ - d_safe_);
      result.speed_factor = std::max(0.0, std::min(1.0, result.speed_factor));
    }

    return result;
  }

  // ----------------------------------------------------------
  // Main 20 Hz control loop
  // ----------------------------------------------------------
  void ControlLoop()
  {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "follower/base_footprint";

    try
    {
      // 1. Look up leader and follower poses in map frame
      geometry_msgs::msg::TransformStamped t_leader = tf_buffer_.lookupTransform(
        "map", "leader/base_footprint",
        tf2::TimePointZero, tf2::durationFromSec(0.1));

      geometry_msgs::msg::TransformStamped t_follower = tf_buffer_.lookupTransform(
        "map", "follower/base_footprint",
        tf2::TimePointZero, tf2::durationFromSec(0.1));

      double lx = t_leader.transform.translation.x;
      double ly = t_leader.transform.translation.y;
      double fx = t_follower.transform.translation.x;
      double fy = t_follower.transform.translation.y;

      // 2. Record leader path as breadcrumbs (every 5 cm)
      if (breadcrumbs_.empty()) {
        breadcrumbs_.push_back({lx, ly});
      } else {
        double dx = lx - breadcrumbs_.back().x;
        double dy = ly - breadcrumbs_.back().y;
        if (std::sqrt(dx*dx + dy*dy) > 0.05) {
          breadcrumbs_.push_back({lx, ly});
        }
      }

      // 3. Pop breadcrumbs the follower has already passed (within 0.35 m)
      while (breadcrumbs_.size() > 1) {
        double dx = fx - breadcrumbs_.front().x;
        double dy = fy - breadcrumbs_.front().y;
        if (std::sqrt(dx*dx + dy*dy) < 0.35) {
          breadcrumbs_.erase(breadcrumbs_.begin());
        } else {
          break;
        }
      }

      // 4. Distance to leader
      double dist_to_leader = std::sqrt((lx - fx)*(lx - fx) + (ly - fy)*(ly - fy));
      // Desired following distance: must be < initial spawn separation (1.2m)
      // so the follower immediately starts moving on first launch.
      // d_desired = 1.0m → target_linear = kp*(1.2 - 1.0) = 0.5 m/s at startup.
      // NOTE: deliberately NOT padded by footprint_clearance_ — doing so
      // would push d_desired above the 1.2m spawn separation and the
      // follower would never start moving (target_linear clamps to 0
      // whenever dist_to_leader < d_desired).
      const double d_desired = 1.0;

      // Emergency stop only when physically about to collide. Padded by
      // footprint_clearance_ because dist_to_leader is measured center-to-
      // center — without this, a 0.5m center distance between two robots
      // with non-trivial body size can mean the bodies are already
      // touching or overlapping visually.
      if (dist_to_leader < 0.5 + footprint_clearance_) {
        cmd_vel_pub_->publish(cmd);   // zero twist
        return;
      }

      // 5. Target a breadcrumb at fixed lookahead distance
      //    (instead of always the oldest/nearest crumb).
      //    This smooths out leader wobble because we're aiming
      //    further down the path rather than chasing every jitter.
      double target_x, target_y;
      size_t idx = 0;
      for (; idx < breadcrumbs_.size(); ++idx) {
        double dxc = breadcrumbs_[idx].x - fx;
        double dyc = breadcrumbs_[idx].y - fy;
        if (std::sqrt(dxc*dxc + dyc*dyc) >= lookahead_) break;
      }
      if (idx >= breadcrumbs_.size()) idx = breadcrumbs_.size() - 1;
      target_x = breadcrumbs_[idx].x;
      target_y = breadcrumbs_[idx].y;

      // 6. Extract follower yaw in map frame
      double qx = t_follower.transform.rotation.x;
      double qy = t_follower.transform.rotation.y;
      double qz = t_follower.transform.rotation.z;
      double qw = t_follower.transform.rotation.w;
      double follower_yaw = std::atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz));

      // 7. Heading error to target (local-frame rotation method)
      double dx_map = target_x - fx;
      double dy_map = target_y - fy;
      double x_rel =  dx_map * std::cos(follower_yaw) + dy_map * std::sin(follower_yaw);
      double y_rel = -dx_map * std::sin(follower_yaw) + dy_map * std::cos(follower_yaw);
      double theta_to_target = std::atan2(y_rel, x_rel);

      // 8. Breadcrumb proportional control
      const double kp_linear  = 2.5;
      const double kp_angular = 4.5;

      double target_linear = kp_linear * (dist_to_leader - d_desired);
      if (target_linear < 0.0) target_linear = 0.0;

      // Scale down linear speed if facing away from target
      double heading_factor = std::cos(theta_to_target);
      if (heading_factor < 0.0) heading_factor = 0.0;
      target_linear *= heading_factor;

      double trail_angular = kp_angular * theta_to_target;

      // 9. Obstacle potential field
      double dx_leader = lx - fx;
      double dy_leader = ly - fy;
      double lx_rel = dx_leader * std::cos(follower_yaw) + dy_leader * std::sin(follower_yaw);
      double ly_rel = -dx_leader * std::sin(follower_yaw) + dy_leader * std::cos(follower_yaw);
      ObstForce of = ComputeObstacleForce(follower_yaw, lx_rel, ly_rel);

      // 10. Max speed (higher if far behind, reduced near obstacles)
      double max_speed = 0.5;
      if (dist_to_leader > 2.5) {
        max_speed = 1.2;
      } else if (dist_to_leader > 1.8) {
        max_speed = 0.5 + (dist_to_leader - 1.8) * (1.2 - 0.5) / (2.5 - 1.8);
      }
      max_speed *= of.speed_factor;

      // 11. Combine and clamp
      cmd.twist.linear.x  = std::min(target_linear, max_speed);
      cmd.twist.angular.z = std::max(std::min(trail_angular + of.angular, 3.0), -3.0);
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 300,
        "dist=%.2f trail_ang=%.2f obst_ang=%.2f tan_sign=%.0f min_d=%.2f",
        dist_to_leader, trail_angular, of.angular, last_tan_sign_, of.min_distance);

      cmd_vel_pub_->publish(cmd);
    }
    catch (const tf2::TransformException & ex)
    {
      cmd_vel_pub_->publish(cmd);   // publish zero twist on failure
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Waiting for TF lookup: %s", ex.what());
    }
  }

  // ---- ROS infrastructure ----------------------------------
  tf2_ros::Buffer    tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr   lidar_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ---- State -----------------------------------------------
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;

  struct Breadcrumb { double x, y; };
  std::vector<Breadcrumb> breadcrumbs_;

  // ---- Tunable parameters ----------------------------------
  double k_rep_, k_tan_, d_inf_, d_safe_, lookahead_;
  double footprint_clearance_;

  // ---- Hysteresis state for tangential force sign -----------
  double last_tan_sign_ = 1.0;
  int    flip_streak_   = 0;
  static constexpr int kFlipThreshold = 3;

  // ---- Cached lidar -> base_footprint transform --------------
  tf2::Transform lidar_to_base_tf_;
  bool lidar_tf_cached_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FollowerController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
