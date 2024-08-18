#RPi Nav2 Motor Controller Node

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import serial
from serial.tools import list_ports
import time
import string

#nav2 publishes velocity commands on /cmd_vel

class Nav2MotorControl(Node):
    def __init__(self):
        super().__init__('nav2_motor_control')
        self.serial_port = self.detect_serial_port()

        if not self.serial_port:
            self.get_logger().error("No valid serial port found")
            raise RuntimeError("No valid serial port found")

        # Subscribe to the input topic
        #reliability best effort qos profile for the subscriber (UDP-like)
        self.qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                               history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                               depth=1)
        self.subscription = self.create_subscription(
            Twist,
            '/joystick_cmd_vel',
            self.message_callback,
            10
        )

    def clamp_pwm(self, pwm_val):
        return max(min(pwm_val, 255), -255)
    
    def detect_serial_port(self):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if 'USB' in port.description and '1A86:7523' in port.hwid: #master arduino hwid number
                try:
                    print(port.hwid)
                    serial_port = serial.Serial(port.device, 9600, timeout=1)
                    self.get_logger().info(f"Connected to serial port: {port.device}")
                    return serial_port
                except serial.SerialException as e:
                    self.get_logger().error(f"Failed to open serial port {port.device}: {e}")
        return None

    def compute_kinematics(self, lin_vel, ang_vel):
        WHEEL_RADIUS = 0.15 #m
        WHEEL_DIST =   0.3  #m
        MU = 0.15   
        MASS = 4 #kg
        MAX_LIN_SPEED = 4#m/s probably???
        MAX_ANG_SPEED = 3.14 #rad/sec = 180 in 1 sec, seems like a reasonnable max speed

        corr_factor =  1 + (MU*WHEEL_DIST*ang_vel/(2*MASS*9.81))

        speed_left =  (lin_vel - (WHEEL_DIST*ang_vel/2))/corr_factor
        speed_right = (lin_vel + (WHEEL_DIST*ang_vel/2))/corr_factor

        #estim_lin_vel = (speed_left + speed_right) / 2
        #estim_ang_vwl = (speed_right - speed_left) / WHEEL_DIST

        pwm_left = clamp_pwm((speed_left/MAX_LIN_SPEED)*255)
        pwm_right = clamp_pwm((speed_right/MAX_LIN_SPEED)*255)
        return [pwm_left, pwm_right]
    
    def message_callback(self, msg):
        lin_vel = msg.linear.x
        ang_vel = msg.angular.z
        pwms = compute_kinematics(lin_vel, ang_vel)
        
        msg = f"{pwms[0]}, {pwms[1]}"
        self.serial_port.write((msg + '\n').encode('utf-8'))    #sends pwm left, pwm right. Example : 145, -145

        received_from_arduino = self.serial_port.readline().decode('utf-8').strip()
        #in the form: Master ACK, FR_ACK, FL_ACK, BR_ACK, BL_ACK, FR_SPEED, FL_SPEED, BR_SPEED, BL_SPEED
        
        if received_from_arduino:
            #master_log = received_from_arduino.split(",")
            #ACK_log = [True if master_log[i] == "ACK" else False for i in [0, 1, 2, 3, 4]]
            #speed_log = [val for val in master_log[5::]]
            #print(f"speeds : Front Right Wheel: {speed_log[0]}")
            #...
            curr_time=time.strftime("%d-%m-%Y %H:%M:%S")
            self.get_logger().info(f"Rover Log @{curr_time}: {received_from_arduino}")
            print("----------------")
        

    
    




def main(args=None):
    rclpy.init(args=args)
    node = Nav2MotorControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_port.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
