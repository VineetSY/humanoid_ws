#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float32
from std_srvs.srv import SetBool

class SystemSupervisor(Node):
    def __init__(self):
        super().__init__('system_supervisor')
        self.get_logger().info('Initializing Main Robotics Processing Unit...')
        
        # State
        self.system_state = "STANDBY"
        self.state_publisher = self.create_publisher(String, '/robot/system_state', 10)
        
        # Inputs
        self.perception_sub = self.create_subscription(String, '/perception/detected_object', self.perception_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.temp_sub = self.create_subscription(Float32, '/sensors/core_temp', self.temp_callback, 10)
        self.srv_teleop = self.create_service(SetBool, '/control/request_teleop', self.handle_teleop_request)

        self.timer = self.create_timer(0.1, self.control_loop)

    def perception_callback(self, msg):
        """Reacts to sensory info from Perception Unit"""
        if self.system_state == "FAILSAFE":
            return

        obj_type = msg.data
        self.get_logger().info(f'PROCESSING UNIT: Analysis complete. Target identified: {obj_type}')
        
        if obj_type == "TARGET_LEFT":
            self.system_state = "REACH_LEFT"
        elif obj_type == "TARGET_RIGHT":
            self.system_state = "REACH_RIGHT"
        elif obj_type == "CLEAR":
            self.system_state = "AUTO"

    def joint_callback(self, msg):
        # Monitor motor load (Simulated)
        if msg.velocity:
            max_vel = max([abs(v) for v in msg.velocity])
            # If velocity is high, load is high.
            if max_vel > 4.0: 
                self.trigger_failsafe("MOTOR_OVERLOAD_PROTECTION")

    def temp_callback(self, msg):
        if msg.data > 75.0:
            self.trigger_failsafe("THERMAL_PROTECTION_SYSTEM")

    def handle_teleop_request(self, request, response):
        if self.system_state == "FAILSAFE":
            response.success = False
            response.message = "REJECTED: System in FAILSAFE."
        else:
            if request.data:
                self.system_state = "TELEOP"
                response.message = "HANDOVER: Remote Operations Active."
            else:
                self.system_state = "AUTO"
                response.message = "Autonomous Mode Engaged."
            response.success = True
        return response

    def trigger_failsafe(self, reason):
        if self.system_state != "FAILSAFE":
            self.get_logger().error(f'CRITICAL: {reason}. CUTTING MOTOR POWER -> LIMP HOME.')
            self.system_state = "FAILSAFE"

    def control_loop(self):
        msg = String()
        msg.data = self.system_state
        self.state_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SystemSupervisor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
