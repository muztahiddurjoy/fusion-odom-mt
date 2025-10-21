#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sbg_driver.msg import SbgGpsPos
import math

class GpsPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher_node')
        
        # Publisher
        self.publisher_ = self.create_publisher(SbgGpsPos, '/best_gps_acc', 10)
        
        # Starting coordinates (latitude, longitude)
        self.start_lat = 23.773206
        self.start_lon = 90.42224
        
        # Parameters
        self.speed = 0.05  # 5 cm/s = 0.05 m/s
        self.total_distance = 100.0  # 100 meters
        self.current_distance = 0.0
        
        # Earth radius in meters
        self.earth_radius = 6371000.0
        
        # Timer - publish every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info('GPS Publisher Node started')
        self.get_logger().info(f'Starting position: {self.start_lat}, {self.start_lon}')
        self.get_logger().info(f'Moving forward {self.total_distance}m at {self.speed}m/s')
        
    def calculate_new_coordinates(self, lat, lon, distance, bearing=0.0):
        """
        Calculate new coordinates given a starting point, distance, and bearing.
        Bearing 0 = North (forward)
        
        Args:
            lat: Starting latitude in degrees
            lon: Starting longitude in degrees
            distance: Distance to move in meters
            bearing: Direction in degrees (0 = North)
        
        Returns:
            Tuple of (new_lat, new_lon)
        """
        # Convert to radians
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        bearing_rad = math.radians(bearing)
        
        # Calculate new latitude
        new_lat_rad = math.asin(
            math.sin(lat_rad) * math.cos(distance / self.earth_radius) +
            math.cos(lat_rad) * math.sin(distance / self.earth_radius) * math.cos(bearing_rad)
        )
        
        # Calculate new longitude
        new_lon_rad = lon_rad + math.atan2(
            math.sin(bearing_rad) * math.sin(distance / self.earth_radius) * math.cos(lat_rad),
            math.cos(distance / self.earth_radius) - math.sin(lat_rad) * math.sin(new_lat_rad)
        )
        
        # Convert back to degrees
        new_lat = math.degrees(new_lat_rad)
        new_lon = math.degrees(new_lon_rad)
        
        return new_lat, new_lon
    
    def timer_callback(self):
        if self.current_distance >= self.total_distance:
            self.get_logger().info('Reached 100m distance. Stopping.')
            self.timer.cancel()
            return
        
        # Calculate current position
        new_lat, new_lon = self.calculate_new_coordinates(
            self.start_lat, 
            self.start_lon, 
            self.current_distance,
            bearing=0.0  # Moving North (forward)
        )
        
        # Create and populate message
        msg = SbgGpsPos()
        msg.latitude = new_lat
        msg.longitude = new_lon
        msg.altitude = 0.0  # You can modify this if altitude data is needed
        
        # Publish
        self.publisher_.publish(msg)
        
        self.get_logger().info(
            f'Published GPS: Lat={new_lat:.8f}, Lon={new_lon:.8f}, '
            f'Distance={self.current_distance:.2f}m'
        )
        
        # Update distance for next iteration
        self.current_distance += self.speed

def main(args=None):
    rclpy.init(args=args)
    node = GpsPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()