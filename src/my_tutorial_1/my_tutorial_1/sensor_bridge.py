#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import serial
import math
import time

def yaw_to_quat(yaw):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return (0.0, 0.0, qz, qw)

class SensorBridge(Node):
    def __init__(self,
                 port='/dev/ttyACM0',
                 baud=115200,
                 ticks_per_rev=975.0,
                 wheel_radius=0.05,
                 wheel_base=0.25,
                 track_width=0.20,
                 publish_rate_hz=50.0):
        super().__init__('sensor_bridge')
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.odom_pub = self.create_publisher(Odometry, 'wheel/odom', 10)

        self.ser = serial.Serial(port, baud, timeout=1)
        self.get_logger().info(f"Opened serial {port} @ {baud}")

        # encoder params
        self.ticks_per_rev = float(ticks_per_rev)
        self.r = float(wheel_radius)
        # use half distances for L and W in formulas
        self.L = float(wheel_base) / 2.0
        self.W = float(track_width) / 2.0

        # storage for previous encoder readings
        self.prev_ticks = [0.0, 0.0, 0.0, 0.0]
        self.prev_time = time.time()

        # pose state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.timer = self.create_timer(1.0 / publish_rate_hz, self.timer_callback)

    def timer_callback(self):
        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            return

        # expect 10 comma-separated values
        parts = line.split(',')
        if len(parts) < 10:
            self.get_logger().warn("Bad serial line (expected 10 values): " + line)
            return

        try:
            gx, gy, gz, ax, ay, az, e1, e2, e3, e4 = map(float, parts[:10])
        except ValueError:
            self.get_logger().warn("Parse error: " + line)
            return

        now = time.time()
        dt = now - self.prev_time
        if dt <= 0:
            return

        # ---------------- IMU ----------------
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        # Arduino sends gyro in deg/s → convert to rad/s here
        imu_msg.angular_velocity.x = math.radians(gx)
        imu_msg.angular_velocity.y = math.radians(gy)
        imu_msg.angular_velocity.z = math.radians(gz)

        # accel already in m/s^2 (as you said)
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az

        imu_msg.orientation_covariance[0] = -1.0  # no orientation

        self.imu_pub.publish(imu_msg)

        # ---------------- Encoders -> wheel speeds ----------------
        ticks = [e1, e2, e3, e4]
        # compute delta ticks
        delta = [ticks[i] - self.prev_ticks[i] for i in range(4)]

        # wheel angular velocities (rad/s): (delta_ticks / ticks_per_rev) * 2π / dt
        omega_w = [(delta[i] / self.ticks_per_rev) * (2.0 * math.pi) / dt for i in range(4)]

        # wheel linear velocities (m/s)
        v_w = [omega_w[i] * self.r for i in range(4)]
        # wheel order: assume e1=FL, e2=FR, e3=RL, e4=RR (match your Arduino)
        v_FL, v_FR, v_RL, v_RR = v_w

        # ---------------- Mecanum inverse kinematics ----------------
        vx = (v_FL + v_FR + v_RL + v_RR) / 4.0
        vy = (-v_FL + v_FR + v_RL - v_RR) / 4.0
        omega = (-v_FL + v_FR - v_RL + v_RR) / (4.0 * (self.L + self.W))

        # integrate pose (body -> world)
        dx = (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
        dy = (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt
        d_yaw = omega * dt

        self.x += dx
        self.y += dy
        self.yaw += d_yaw

        # ---------------- Publish odometry ----------------
        odom = Odometry()
        odom.header.stamp = imu_msg.header.stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quat(self.yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # twist
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega

        # (optionally fill covariances)
        self.odom_pub.publish(odom)

        # save for next loop
        self.prev_ticks = ticks.copy()
        self.prev_time = now


def main(args=None):
    rclpy.init(args=args)
    node = SensorBridge(port='/dev/ttyACM0', baud=115200,
                        ticks_per_rev=975.0, wheel_radius=0.05,
                        wheel_base=0.25, track_width=0.20,
                        publish_rate_hz=50.0)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


'''slam_toolbox:
  ros__parameters:
    # General
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
    mode: mapping  # or 'localization' later
    debug_logging: false
    throttle_scans: 1
    map_update_interval: 5.0  # Seconds between map updates
    
    # Mapping params
    minimum_travel_distance: 0.5  # Meters before updating map
    minimum_travel_heading: 0.5   # Radians
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 20.0
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1
    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03
    loop_match_minimum_chain_size: 10
    loop_match_min_response_fine: 0.35
    loop_match_max_variance_coarse: 0.4
    loop_match_min_response_coarse: 0.6
    
    # Optimization
    do_loop_closing: true
    optimization_score: 0.5
    angular_odometry_reliability: 2.0
    linear_odometry_reliability: 2.0
    use_scan_barycenter: true
    use_scan_radius_filter: false
    
    # Map quality
    map_start_pose: [0.0, 0.0, 0.0]  # Initial pose [x, y, yaw]
    map_start_at_dock: false
    map_file_name: ""  # Leave empty for new map
    resolution: 0.05  # Meters per pixel (higher res = more detail, but larger file)
    
    # Use your fused odom
    use_odometry: true
    odom_topic: /wheel/odom # Remap if needed'''
