#subscribes to /joystick_cmd_vel data sent from joystick and applies the rover's kinematics to send the 
#speed values to each wheel of the rover in m/s.
#then sends the speeds to the master arduino
#also periodically reads from the arduino serial port to check the wheel speeds and send them to /odom for slam


#JOYSTICK CONTROLLER FOR SLAM AND MANUAL CONTROL


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
from serial.tools import list_ports
from serial.serialutil import SerialException
import time
import math


from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class JoyPwmMotorControl(Node):
    def __init__(self):
        super().__init__('joy_pwm_motor_control')
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
        self.sent_wheel_speeds_publisher = self.create_publisher(String, '/sent_pwm', 10)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_odometry)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        self.rover_width = 0.25  # 20cm between the centers of the left and right wheels.
        self.rover_mass = 4  # kg

        #add timer to read the serial port for messages from the arduino
        timer_period = 0.05# seconds  MAYBE THIS IS TOO FAST?? we'll have to wait and see the performance with SLAM
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.ack_received = True

        self.prev_lin_vel = 0.0
        self.prev_ang_vel = 0.0
        self.vel_threshold = 0.1

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
                self.get_logger().warn("Arduino not found. Retrying in 4 seconds...")
                time.sleep(4)
    
    def joystick_cmd_callback(self, msg):

        #if not self.ack_received:
        #    self.get_logger().warn("Waiting for ACK...")
        #    return

        if self.serial_port is None:
            self.get_logger().error("No serial connection. Skipping command.")
            return
    
        lin_vel = msg.linear.x
        ang_vel = msg.angular.z

        #check if the new velocities are significantly different from the previous ones
        #to avoid sending the same cmd and potentially overflowing serial buffer of the master aduino

        if abs(lin_vel - self.prev_lin_vel) < self.vel_threshold and abs(ang_vel - self.prev_ang_vel) < self.vel_threshold:
            return
        self.prev_lin_vel = lin_vel
        self.prev_ang_vel = ang_vel
        
        pwm_vals = self.compute_kinematics_pwm(lin_vel, ang_vel)

        #format speed values to 2 decimal points and send as string : 
        #front_right_speed,front_left_speed,back_right_speed,back_left_speed
        
        msg = f"{pwm_vals[0]:.2f},{pwm_vals[1]:.2f},{pwm_vals[2]:.2f},{pwm_vals[3]:.2f}"
        sent_wheel_speeds_msg = String()
        sent_wheel_speeds_msg.data = msg
        self.sent_wheel_speeds_publisher.publish(sent_wheel_speeds_msg)

        #self.get_logger().info(f"sent: {msg}")
        try:
            self.serial_port.write((msg + '\n').encode('utf-8'))
            #self.ack_received = False
            #self.wait_for_ack()
        except Exception as e:
            self.get_logger().error(f"SerialException occurred: {e}")

    def wait_for_ack(self):
        ack = None
        start_time = time.time()
        timeout = 0.5  # seconds

        while time.time() - start_time < timeout:
            if self.serial_port.in_waiting > 0:
                ack = self.serial_port.readline().decode('utf-8').strip()
                if ack == "ACK":
                    self.get_logger().info(f"ack: {ack}")
                    self.ack_received = True  # ACK received, allow next command
                    return
                else:
                    self.get_logger().warn(f"Unexpected ack: {ack}")
                    return
        else:
            # Timeout: ACK not received, allow system to recover
            self.get_logger().warn("ACK not received. Retrying...")
            self.ack_received = True



    def timer_callback(self):
        try:
            received_from_arduino = None
            if self.serial_port.in_waiting > 0:
                received_from_arduino = self.serial_port.readline().decode('utf-8').strip()
            if received_from_arduino:
                curr_time=time.strftime("%d-%m-%Y %H:%M:%S")
                self.get_logger().info(f"Master @{curr_time}: {received_from_arduino}")

                required_terms = ["FR", "FL", "BR", "BL"]  #parsing the incoming arduino logs 
                if all(term in received_from_arduino for term in required_terms):

                    parsed_speeds = received_from_arduino.split(',')
                    parsed_speeds = [_.split(':') for _ in parsed_speeds]


                    if parsed_speeds[0][1] != "NAN" and parsed_speeds[0][1] != "nan":
                        self.fr_wheel_speed = float(parsed_speeds[0][1])
                    else:
                        self.fr_wheel_speed = 0.00

                    if parsed_speeds[1][1] != "NAN" and parsed_speeds[1][1] != "nan":
                        self.fl_wheel_speed = float(parsed_speeds[1][1])
                    else:
                        self.fl_wheel_speed = 0.00

                    if parsed_speeds[2][1] != "NAN" and parsed_speeds[2][1] != "nan":
                        self.br_wheel_speed = float(parsed_speeds[2][1])
                    else:
                        self.br_wheel_speed = 0.00

                    if parsed_speeds[3][1] != "NAN" and parsed_speeds[3][1] != "nan":
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

        except Exception as e:
            #close current connection and try again
            self.serial_port.close()
            self.serial_port = None
            self.reconnect_serial()


    def get_odom_velocities(self):
        #output: linear velocity and angular velocity given the wheel speeds

        right_wheel_velocity = (self.fr_wheel_speed + self.br_wheel_speed)/2.0
        left_wheel_velocity = (self.fl_wheel_speed + self.bl_wheel_speed)/2.0

        v = (right_wheel_velocity + left_wheel_velocity) / 2.0
        omega = ( (right_wheel_velocity - left_wheel_velocity) / self.rover_width )

        return v, omega

    def compute_kinematics_pwm(self, lin_vel, ang_vel):
        #input: linear velocity and angular velocity
        #output: pwm values for each wheel
        #simple open loop kinematics that send the pwm values

        pwm_left  = (lin_vel - self.slip_factor * (self.rover_width * ang_vel/2.0))
        pwm_right = (lin_vel + self.slip_factor * (self.rover_width * ang_vel/2.0))

        #not enough power to turn the rover, so boost the pwm when turning
        if ang_vel != 0 and abs(lin_vel) < 1.0:
            pwm_left = pwm_left * 2.0
            pwm_right = pwm_right * 2.0

        #map 0--> 0 pwm, 2--> 255 pwm
        pwm_left = 255 * (pwm_left/2)
        pwm_right = 255 * (pwm_right/2)
        
        return [pwm_right, pwm_left, pwm_right, pwm_left]

    def publish_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Time delta in seconds

        # Get current velocities based on wheel speeds
        v, omega = self.get_odom_velocities()

        # Update the robot's pose using kinematic equations
        delta_x = v * math.cos(self.theta) * dt
        delta_y = v * math.sin(self.theta) * dt
        delta_theta = omega * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Create the odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"  # Reference frame
        odom.child_frame_id = "base_link"  # Robot's reference frame

        # Update the pose in the odometry message
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        # Convert theta (yaw angle) to quaternion
        quaternion = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion()
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]

        # Set the velocity in the odometry message
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega

        # Publish the odometry message
        self.odom_pub.publish(odom)

        # Broadcast the transform (from odom to base_link)
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform)

        # Update the last time for the next iteration
        self.last_time = current_time



def main(args=None):
    rclpy.init(args=args)
    node = JoyPwmMotorControl()
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
