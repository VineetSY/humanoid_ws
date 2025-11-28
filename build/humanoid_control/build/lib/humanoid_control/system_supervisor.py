#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float32
from std_srvs.srv import SetBool

class SystemSupervisor(Node):
    def __init__(self):
        super().__init__('system_supervisor')
        self.get_logger().info('Initializing Robotics Mobility & Manipulation Controller...')
        
        # State Management
        self.system_state = "STANDBY"
        self.state_publisher = self.create_publisher(String, '/robot/system_state', 10)
        
        # Inputs
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.temp_sub = self.create_subscription(Float32, '/sensors/core_temp', self.temp_callback, 10)
        
        # Service
        self.srv_teleop = self.create_service(SetBool, '/control/request_teleop', self.handle_teleop_request)

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)

    def joint_callback(self, msg):
        if msg.velocity:
            max_vel = max([abs(v) for v in msg.velocity])
            if max_vel > 3.0: 
                self.trigger_failsafe("OVERSPEED_DETECTED")

    def temp_callback(self, msg):
        if msg.data > 75.0:
            self.trigger_failsafe("OVERHEAT_DETECTED")

    def handle_teleop_request(self, request, response):
        if self.system_state == "FAILSAFE":
            response.success = False
            response.message = "Cannot handover: System in FAILSAFE mode."
        else:
            if request.data:
                self.system_state = "TELEOP"
                response.message = "Handover complete. Remote Operations Active."
            else:
                self.system_state = "AUTO"
                response.message = "Returned to Autonomous Mode."
            response.success = True
            self.get_logger().info(f'State transition: {response.message}')
        return response

    def trigger_failsafe(self, reason):
        if self.system_state != "FAILSAFE":
            self.get_logger().error(f'CRITICAL FAILURE: {reason}. Engaging LIMP HOME mode.')
            self.system_state = "FAILSAFE"

    def control_loop(self):
        msg = String()
        msg.data = self.system_state
        self.state_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SystemSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
