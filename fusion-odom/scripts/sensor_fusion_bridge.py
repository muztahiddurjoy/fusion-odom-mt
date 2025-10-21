#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sbg_driver.msg import SbgGpsPos
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
import math
from transforms3d.euler import euler2quat

class SensorFusionBridge(Node):
    """
    Bridge node to convert SBG GPS and yaw data to formats compatible with robot_localization
    """
    def __init__(self):
        super().__init__('sensor_fusion_bridge')
        
        # Publishers
        self.navsat_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        
        # Subscribers
        self.gps_sub = self.create_subscription(
            SbgGpsPos,
            '/best_gps_acc',
            self.gps_callback,
            10
        )
        
        self.yaw_sub = self.create_subscription(
            Float64,
            '/witmotion_eular/yaw',
            self.yaw_callback,
            10
        )
        
        # Store latest yaw value
        self.latest_yaw = 0.0
        
        self.get_logger().info('Sensor Fusion Bridge Node started')
        self.get_logger().info('Converting SBG GPS to NavSatFix')
        self.get_logger().info('Converting Yaw to IMU orientation')
        
    def yaw_callback(self, msg):
        """Store latest yaw value and publish as IMU message"""
        self.latest_yaw = msg.data
        
        # Create IMU message with orientation only
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        # Convert yaw (degrees) to quaternion
        yaw_rad = math.radians(self.latest_yaw)
        quaternion = euler2quat(0.0, 0.0, yaw_rad)  # roll=0, pitch=0, yaw=yaw_rad
        
        imu_msg.orientation.x = quaternion[0]
        imu_msg.orientation.y = quaternion[1]
        imu_msg.orientation.z = quaternion[2]
        imu_msg.orientation.w = quaternion[3]
        
        # Set covariance for orientation (we trust yaw data)
        imu_msg.orientation_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        
        # Set high covariance for angular velocity (not available)
        imu_msg.angular_velocity_covariance[0] = -1
        
        # Set high covariance for linear acceleration (not available)
        imu_msg.linear_acceleration_covariance[0] = -1
        
        self.imu_pub.publish(imu_msg)
        
    def gps_callback(self, msg):
        """Convert SbgGpsPos to NavSatFix"""
        navsat_msg = NavSatFix()
        navsat_msg.header.stamp = self.get_clock().now().to_msg()
        navsat_msg.header.frame_id = 'gps'
        
        navsat_msg.latitude = msg.latitude
        navsat_msg.longitude = msg.longitude
        navsat_msg.altitude = msg.altitude
        
        # Set status
        navsat_msg.status.status = 0  # STATUS_FIX
        navsat_msg.status.service = 1  # SERVICE_GPS
        
        # Set covariance (adjust based on your GPS accuracy)
        # Format: [lat, lon, alt] variance in m^2
        navsat_msg.position_covariance = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]
        navsat_msg.position_covariance_type = 2  # COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        self.navsat_pub.publish(navsat_msg)
        
        self.get_logger().info(
            f'Published NavSatFix: Lat={msg.latitude:.8f}, Lon={msg.longitude:.8f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()