#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import String

class ObjectSpawner(Node):
    def __init__(self):
        super().__init__('object_spawner')
        self.publisher_ = self.create_publisher(Marker, '/simulation/target_object', 10)
        self.state_sub = self.create_subscription(String, '/robot/system_state', self.state_callback, 10)
        self.current_state = "STANDBY"
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        # FAR AWAY COORDINATES
        self.floor_x = 1.5
        self.floor_y = 0.5 
        self.floor_z = 0.1 

    def state_callback(self, msg):
        self.current_state = msg.data

    def timer_callback(self):
        marker = Marker()
        marker.header.stamp.sec = 0; marker.header.stamp.nanosec = 0
        marker.ns = "payload"; marker.id = 0; marker.type = Marker.CUBE; marker.action = Marker.ADD
        marker.scale.x = 0.15; marker.scale.y = 0.15; marker.scale.z = 0.15
        marker.color.a = 1.0; marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0

        if self.current_state == "LIFTING":
            marker.header.frame_id = "left_hand"
            marker.pose.position.x = 0.0; marker.pose.position.y = 0.0; marker.pose.position.z = -0.15
        elif self.current_state == "FAILSAFE":
            marker.header.frame_id = "world"
            marker.pose.position.x = self.floor_x; marker.pose.position.y = self.floor_y; marker.pose.position.z = 0.0 
        else:
            marker.header.frame_id = "world"
            marker.pose.position.x = self.floor_x; marker.pose.position.y = self.floor_y; marker.pose.position.z = self.floor_z
        self.publisher_.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
