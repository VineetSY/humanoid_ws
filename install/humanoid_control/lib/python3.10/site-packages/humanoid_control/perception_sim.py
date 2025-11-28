#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PerceptionSim(Node):
    """
    Simulates a Vision/Perception System.
    Publishes 'detection' events based on user keyboard commands via terminal topics.
    """
    def __init__(self):
        super().__init__('perception_sim')
        self.publisher_ = self.create_publisher(String, '/perception/detected_object', 10)
        self.get_logger().info('Perception System Online. Waiting for sensory input...')

        # In a real robot, this would process camera frames.
        # Here, we just echo what we receive on a trigger topic for testing.
        self.trigger_sub = self.create_subscription(String, '/perception/trigger_event', self.trigger_callback, 10)

    def trigger_callback(self, msg):
        # Forward the trigger as a detection event
        detection_msg = String()
        detection_msg.data = msg.data
        self.publisher_.publish(detection_msg)
        self.get_logger().info(f'SENSORY INPUT RECEIVED: Detected {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
