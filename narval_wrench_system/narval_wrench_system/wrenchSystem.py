import rclpy
from rclpy.node import Node

from ds4_driver_msgs.msg import Status
from geometry_msgs.msg import Wrench, WrenchStamped


class WrenchSystem:

    def __init__(self, node: Node):

        self.node = node
        self._logger = self.node.get_logger()

        self.node.declare_parameter("use_dualshock4", True)

        self.node.declare_parameter("wrench_topic_name", "joy_wrench")
        self.node.declare_parameter("wrench_stamped_topic_name", "joy_wrench_stmp")

        self.node.declare_parameter("send_stamped", True)
        self.node.declare_parameter("frame_id", "base_link")
        
        self.node.declare_parameter("max_force", 1)
        self.node.declare_parameter("max_torque", 1)

        self.node.declare_parameter("ds4_force_x", "axis_left_y")
        self.node.declare_parameter("ds4_force_y", "axis_left_x")
        self.node.declare_parameter("ds4_force_z", "axis_right_y")
        self.node.declare_parameter("ds4_torque_x", "axis_right_x")
        self.node.declare_parameter("ds4_torque_y", "")
        self.node.declare_parameter("ds4_torque_z", "")
        

        self.use_dualshock4 = self.node.get_parameter("use_dualshock4").value

        self.send_stamped = self.node.get_parameter("send_stamped").value
        self.frame_id = self.node.get_parameter("frame_id").value

        self.max_force = self.node.get_parameter("max_force").value
        self.max_torque = self.node.get_parameter("max_torque").value
        
        if self.send_stamped:
            self.pub_joy_wrench = self.node.create_publisher(WrenchStamped, "joy_wrench_stmp", 0)
        else:
            self.pub_joy_wrench = self.node.create_publisher(Wrench, "joy_wrench", 0)

        if self.use_dualshock4:
            self.sub_ds4_driver = self.node.create_subscription(Status, "status", self.cb_ds4_driver, 0)

            self.joy_force_x = self.node.get_parameter("ds4_force_x").value
            self.joy_force_y = self.node.get_parameter("ds4_force_y").value
            self.joy_force_z = self.node.get_parameter("ds4_force_z").value

            self.joy_torque_x = self.node.get_parameter("ds4_torque_x").value
            self.joy_torque_y = self.node.get_parameter("ds4_torque_y").value
            self.joy_torque_z = self.node.get_parameter("ds4_torque_z").value



    
    def cb_ds4_driver(self, msg: Status):
        
        now = self.node.get_clock().now()

        if self.send_stamped:
            wrench_msg = WrenchStamped()
            wrench_msg.header.frame_id = self.frame_id
            wrench_msg.header.stamp = now.to_msg()
            
            w = wrench_msg.wrench

        else:
            wrench_msg = Wrench()

            w = wrench_msg

        try:
            w.force.x = self.max_force * float(getattr(msg, self.joy_force_x))
            w.force.y = self.max_force * float(getattr(msg, self.joy_force_y))
            w.force.z = self.max_force * float(getattr(msg, self.joy_force_z))

            w.torque.x = self.max_torque * float(msg.button_dpad_right - msg.button_dpad_left)
            w.torque.y = self.max_torque * float(msg.axis_r2 - msg.axis_l2)
            w.torque.z = self.max_torque * float(getattr(msg, self.joy_torque_x))

        except Exception as e:
            self._logger.error(f"Wrong wrench: {e}")

        if self.send_stamped:
            wrench_msg.wrench.force.x = w.force.x
            wrench_msg.wrench.force.y = w.force.y
            wrench_msg.wrench.force.z = w.force.z

            wrench_msg.wrench.torque.x = w.torque.x
            wrench_msg.wrench.torque.y = w.torque.y
            wrench_msg.wrench.torque.z = w.torque.z

        else:
            wrench_msg.force.x = w.force.x
            wrench_msg.force.y = w.force.y
            wrench_msg.force.z = w.force.z

            wrench_msg.torque.x = w.torque.x
            wrench_msg.torque.y = w.torque.y
            wrench_msg.torque.z = w.torque.z

        self.pub_joy_wrench.publish(wrench_msg)