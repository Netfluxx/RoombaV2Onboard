#gets wheel speeds from master arduino through topic and sends them to /odom

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import JointState
from tf_transformations import quaternion_from_euler
import math
import time
from random import randrange
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion



class OdometryNode(Node):

    def __init__(self):
        super().__init__('odometry_node')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)  # Add this line
        self.timer = self.create_timer(0.1, self.publish_odometry)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        self.rover_width = 0.2  # 20cm between the centers of the left and right wheels.
        self.rover_mass = 4  # kg

        self.qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                                depth=5)

        self.serial_port = self.detect_serial_port()
        if not self.serial_port:
            self.get_logger().error("NO VALID SERIAL PORT FOUND THE MASTER ARDUINO IS COOOOOKED")
            raise RuntimeError("NO VALID SERIAL PORT FOUND")
        else:
            self.get_logger().info("VAILD SERIAL PORT FOUND LETS GOOOO")

    
    def detect_serial_port(self):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if 'USB' in port.description: #and '1A86:7523' in port.hwid: #master arduino hwid number
                try:
                    #print(f"HWID: " {port.hwid})
                    serial_port = serial.Serial(port.device, 9600, timeout=1)
                    self.get_logger().info(f"CONNECTED to serial port: {port.device} with hwid {port.hwid}")
                    return serial_port
                except serial.SerialException as e:
                    self.get_logger().error(f"FAILED to open serial port {port.device}: {e}")
        return None

    def compute_velocities(self):
        #TODO: Get encoder ticks from arduino, v=omega*wheel radius, omega = nbr of ticks/(nbr of ticks per rev * delta_t) probably

        left_wheel_velocity = 0#randrange(1, 4) / 5.0
        right_wheel_velocity = 0#randrange(1, 4) / 5.0

        # basic differential kinematic TODO: Add friction and correct slip? difficult without IMU
        v = (right_wheel_velocity + left_wheel_velocity) / 2.0
        omega = (right_wheel_velocity - left_wheel_velocity) / self.rover_width

        return v, omega

    def publish_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        v, omega = self.compute_velocities()

        # Update the pose by integration of speed
        delta_x = v * math.cos(self.theta) * dt
        delta_y = v * math.sin(self.theta) * dt
        delta_theta = omega * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Create the odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        #odom.pose.pose.orientation = quaternion_from_euler(0, 0, self.theta)
        
        q = quaternion_from_euler(0, 0, self.theta)
        quaternion_msg = Quaternion()
        quaternion_msg.x = q[0]
        quaternion_msg.y = q[1]
        quaternion_msg.z = q[2]
        quaternion_msg.w = q[3]
    
        odom.pose.pose.orientation = quaternion_msg

        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = omega
        self.odom_pub.publish(odom)

        # Broadcast the transform
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform)    #AMCL needs the transform from odom to base_link
        self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    odometry_node = OdometryNode()
    rclpy.spin(odometry_node)
    odometry_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()