#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class GroundTruthBridge(Node):
    def __init__(self):
        super().__init__('ground_truth_bridge')
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        from rclpy.duration import Duration
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.last_published_time = {'leader': (0, 0), 'follower': (0, 0)}
        self.get_logger().info("Ground Truth TF Bridge initialized.")

    def timer_callback(self):
        for ns in ['leader', 'follower']:
            try:
                # 1. Look up empty_world or industrial_world -> ns (published by Gazebo PosePublisher)
                try:
                    world_frame = "empty_world"
                    t_world_base = self.tf_buffer.lookup_transform(world_frame, ns, Time())
                except Exception:
                    try:
                        world_frame = "industrial_world"
                        t_world_base = self.tf_buffer.lookup_transform(world_frame, ns, Time())
                    except Exception:
                        continue
                
                # 2. Look up odom -> base_footprint (published by robot diff_drive_controller)
                odom_frame = f"{ns}/odom"
                base_frame = f"{ns}/base_footprint"
                
                try:
                    t_odom_base = self.tf_buffer.lookup_transform(odom_frame, base_frame, Time())
                except Exception:
                    continue
                
                # 3. Compute T_map_to_odom = T_world_to_base * T_odom_to_base^-1
                xw = t_world_base.transform.translation.x
                yw = t_world_base.transform.translation.y
                qw = t_world_base.transform.rotation
                yaw_w = math.atan2(2.0 * (qw.w * qw.z + qw.x * qw.y), 1.0 - 2.0 * (qw.y * qw.y + qw.z * qw.z))
                
                xo = t_odom_base.transform.translation.x
                yo = t_odom_base.transform.translation.y
                qo = t_odom_base.transform.rotation
                yaw_o = math.atan2(2.0 * (qo.w * qo.z + qo.x * qo.y), 1.0 - 2.0 * (qo.y * qo.y + qo.z * qo.z))
                
                yaw_d = yaw_w - yaw_o
                
                x_odom = xw - (xo * math.cos(yaw_d) - yo * math.sin(yaw_d))
                y_odom = yw - (xo * math.sin(yaw_d) + yo * math.cos(yaw_d))
                
                # 4. Publish map -> odom transform
                # Stamp with the Gazebo pose timestamp (not wall clock) to stay
                # consistent with sim clock and avoid TF extrapolation errors.
                stamp = t_world_base.header.stamp
                stamp_tuple = (stamp.sec, stamp.nanosec)
                if stamp_tuple <= self.last_published_time[ns]:
                    continue
                self.last_published_time[ns] = stamp_tuple
                
                t = TransformStamped()
                t.header.stamp = stamp
                t.header.frame_id = "map"
                t.child_frame_id = odom_frame
                
                t.transform.translation.x = x_odom
                t.transform.translation.y = y_odom
                t.transform.translation.z = 0.0
                
                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = math.sin(yaw_d / 2.0)
                t.transform.rotation.w = math.cos(yaw_d / 2.0)
                
                self.tf_broadcaster.sendTransform(t)
                
            except Exception as e:
                self.get_logger().warn(f"Bridge processing error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
