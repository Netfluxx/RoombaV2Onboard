#subscribes to /joystick_cmd_vel data sent from joystick and applies the rover's kinematics to send the 
#speed values to each wheel of the rover in m/s.
#then sends the speeds to the master arduino
#also periodically reads from the arduino serial port to check the wheel speeds and send them to /odom for slam


#JOYSTICK CONTROLLER FOR SLAM AND MANUAL CONTROL


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
import serial
from serial.tools import list_ports
from serial.serialutil import SerialException
import time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import JointState
from tf_transformations import quaternion_from_euler
import math
from random import randrange
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion


class JoystickMotorControl(Node):
    def __init__(self):
        super().__init__('joystick_motor_control')
        self.serial_port = self.detect_serial_port()

        if not self.serial_port:
            self.get_logger().error("NO VALID SERIAL PORT FOUND, are we cooked ?")
            self.reconnect_serial()

        # Subscribe to the input topic
        #reliability best effort qos profile for the subscriber (UDP-like)
        self.qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                                depth=5)
        
        self.subscription = self.create_subscription(   #TODO: USE TWIST MUX LIKE ARTICULATED ROBOTICS TO BE ABLE TO TAKE CONTROL OF NAV2 IF NEEDED
            Twist,
            '/joystick_cmd_vel',
            self.joystick_cmd_callback,
            10
        )

        self.wheel_speeds_publisher = self.create_publisher(String, '/wheel_speeds', 10)
        self.battery_publisher = self.create_publisher(String, '/battery', 10)
        self.sent_wheel_speeds_publisher = self.create_publisher(String, '/sent_wheel_speeds', 10)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        #self.timer = self.create_timer(0.1, self.publish_odometry)  #DEBUG MEEEEE AND TEST ME

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        self.rover_width = 0.25  # 20cm between the centers of the left and right wheels.
        self.rover_mass = 4  # kg

        #add timer to read the serial port for messages from the arduino
        timer_period = 0.05  # seconds  MAYBE THIS IS TOO FAST?? we'll have to wait and see the performance with SLAM
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.fr_wheel_speed = 0.0
        self.fl_wheel_speed = 0.0
        self.br_wheel_speed = 0.0
        self.bl_wheel_speed = 0.0

        self.slip_factor = 0.8 #TODO: need to test it empirically 
        #under-rotating => significant slippage => slip factor lower than 1 (ex: 0.85)
        #over-rotating => turns too much, too little slippage => slip factor higher than 1

    def detect_serial_port(self):
        try:
            ports = serial.tools.list_ports.comports()
            for port in ports:
                if 'USB' in port.description and '1A86:7523' in port.hwid: #master arduino hwid number
                    try:
                        serial_port = serial.Serial(port.device, 115200, timeout=1)
                        self.get_logger().info(f"CONNECTED to serial port: {port.device} at hwid: {port.hwid}")
                        return serial_port
                    except serial.SerialException as e:
                        self.get_logger().error(f"FAILED to open serial port {port.device}: {e}")
            return None
        except Exception as e:
            self.get_logger().error(f"Error while detecting serial port: {e}")
            return None
    
    def reconnect_serial(self):
        while self.serial_port is None:
            self.get_logger().info("Trying to reconnect to the Arduino...")
            self.serial_port = self.detect_serial_port()
            if self.serial_port:
                self.get_logger().info("Reconnected to the Arduino.")
            else:
                self.get_logger().warn("Arduino not found. Retrying in 5 seconds...")
                time.sleep(5)  # Wait before retrying
    
    def joystick_cmd_callback(self, msg):
        if self.serial_port is None:
            self.get_logger().error("No serial connection. Skipping command.")
            return
        lin_vel = msg.linear.x
        ang_vel = msg.angular.z
        wheel_vels = self.compute_kinematics(lin_vel, ang_vel)

        #format speed values to 2 decimal points and send as string : 
        #front_right_speed,front_left_speed,back_right_speed,back_left_speed
        
        msg = f"{wheel_vels[0]:.2f},{wheel_vels[1]:.2f},{wheel_vels[2]:.2f},{wheel_vels[3]:.2f}"
        sent_wheel_speeds_msg = String()
        sent_wheel_speeds_msg.data = msg
        self.sent_wheel_speeds_publisher.publish(sent_wheel_speeds_msg)

        #self.get_logger().info(f"sent: {msg}")

        #self.serial_port.write((msg + '\n').encode('utf-8'))
        try:
            self.serial_port.write((msg + '\n').encode('utf-8'))
        except SerialException as e:
            self.get_logger().error(f"SerialException occurred: {e}")


    def timer_callback(self):
        try:
            received_from_arduino = None
            if self.serial_port.in_waiting > 0:
                received_from_arduino = self.serial_port.readline().decode('utf-8', errors = 'ignore').strip()
            
            if received_from_arduino:
                curr_time=time.strftime("%d-%m-%Y %H:%M:%S")
                #self.get_logger().info(f"Rover Master Nano @{curr_time}: {received_from_arduino}")

                required_terms = ["FR", "FL", "BR", "BL"]  #parsing the incoming arduino logs 
                if all(term in received_from_arduino for term in required_terms):
                    parsed_speeds = received_from_arduino.split(',')
                    parsed_speeds = [_.split(':') for _ in parsed_speeds]

                    #self.get_logger().info(f"parsed_speeds split: {parsed_speeds}")

                    if parsed_speeds[0][1] != "NAN":
                        self.fr_wheel_speed = float(parsed_speeds[0][1])
                    else:
                        self.fr_wheel_speed = 0.00

                    if parsed_speeds[1][1] != "NAN":
                        self.fl_wheel_speed = float(parsed_speeds[1][1])
                    else:
                        self.fl_wheel_speed = 0.00

                    if parsed_speeds[2][1] != "NAN":
                        self.br_wheel_speed = float(parsed_speeds[2][1])
                    else:
                        self.br_wheel_speed = 0.00

                    if parsed_speeds[3][1] != "NAN":
                        self.bl_wheel_speed = float(parsed_speeds[3][1])
                    else:
                        self.bl_wheel_speed = 0.00

                    wheel_speed_msg = String()
                    wheel_speed_msg.data = f"{self.fr_wheel_speed},{self.fl_wheel_speed},{self.br_wheel_speed},{self.bl_wheel_speed}"
                    self.wheel_speeds_publisher.publish(wheel_speed_msg)
                
                if "battery voltage:" in received_from_arduino:
                    try:
                        batt_voltage_str = received_from_arduino.split(':')[1].strip()
                        batt_voltage = float(batt_voltage_str)
                        
                        batt_pub_msg = String()
                        batt_pub_msg.data = f"{batt_voltage}"
                        self.battery_publisher.publish(batt_pub_msg)

                    except ValueError:
                        self.get_logger().warn(f"Invalid battery voltage received: {batt_voltage_str}")

                #self.get_logger().info(f"----------------")

        except SerialException as e:
            self.get_logger().error(f"SerialException occurred: {e}")
            #close current connection and try again
            #self.serial_port.close()
            #self.serial_port = None
            #self.reconnect_serial()

    def compute_kinematics(self, lin_vel, ang_vel):
        #very simple kinematics

        vel_left  = (lin_vel - self.slip_factor * (self.rover_width * ang_vel/2.0))
        vel_right = (lin_vel + self.slip_factor * (self.rover_width * ang_vel/2.0))

        if ang_vel != 0 and abs(lin_vel) < 1.0:
            vel_left  = vel_left  * 1.6
            vel_right = vel_right * 1.6

        return [vel_right, vel_left, vel_right, vel_left]


    def get_odom_velocities(self):

        right_wheel_velocity = (self.fr_wheel_speed + self.br_wheel_speed)/2.0
        left_wheel_velocity = (self.fl_wheel_speed + self.bl_wheel_speed)/2.0

        v = (right_wheel_velocity + left_wheel_velocity) / 2.0
        omega = ( (right_wheel_velocity - left_wheel_velocity) / self.rover_width )

        return v, omega

    def publish_odometry(self):
        
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        v, omega = self.get_odom_velocities()

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
    node = JoystickMotorControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial_port:
            node.serial_port.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
