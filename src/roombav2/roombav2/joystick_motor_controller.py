#subscribes to /joystick_cmd_vel and applies the rover's kinematics to send the 
#speed values to each wheel of the rover in m/s.


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import serial
from serial.tools import list_ports
import time


class JoystickMotorControl(Node):
    def __init__(self):
        super().__init__('joystick_motor_control')
        self.serial_port = self.detect_serial_port()

        if not self.serial_port:
            self.get_logger().error("No valid serial port found")
            raise RuntimeError("No valid serial port found")

        # Subscribe to the input topic
        #reliability best effort qos profile for the subscriber (UDP-like)
        self.qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                                depth=5)
        self.subscription = self.create_subscription(
            Twist,
            '/joystick_cmd_vel',
            self.message_callback,
            10
        )
    def detect_serial_port(self):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if 'USB' in port.description and '1A86:7523' in port.hwid: #master arduino hwid number
                try:
                    #print(port.hwid)
                    serial_port = serial.Serial(port.device, 9600, timeout=1)
                    self.get_logger().info(f"CONNECTED to serial port: {port.device}")
                    return serial_port
                except serial.SerialException as e:
                    self.get_logger().error(f"FAILED to open serial port {port.device}: {e}")
        return None
    
    def message_callback(self, msg):
        lin_vel = msg.linear.x
        ang_vel = msg.angular.z
        wheel_vels = self.compute_kinematics(lin_vel, ang_vel)

        #format speed values to 2 decimal points and send as string : front_right_speed,front_left_speed,back_right_speed,back_left_speed
        
        msg = f"{wheel_vels[0]:.2f},{wheel_vels[1]:.2f},{wheel_vels[2]:.2f},{wheel_vels[3]:.2f}"

        self.get_logger().info(f"sent: {msg}")

        self.serial_port.write((msg + '\n').encode('utf-8'))
        received_from_arduino = self.serial_port.readline().decode('utf-8').strip()
    
        if received_from_arduino:

            curr_time=time.strftime("%d-%m-%Y %H:%M:%S")
            self.get_logger().info(f"Rover Master Nano @{curr_time}: {received_from_arduino}")
            print("----------------")
    
    def compute_kinematics(self, lin_vel, ang_vel):
    
        WHEEL_DIST =   0.3  #m, dist from center of right wheel to center of left wheel

        vel_left =  (lin_vel - (WHEEL_DIST*ang_vel/2))
        vel_right = (lin_vel + (WHEEL_DIST*ang_vel/2))

        #for odom : omega = (vel_right - vel_left)/WHEEL_DIST

        return [vel_right, vel_left, vel_right, vel_left]



def main(args=None):
    rclpy.init(args=args)
    node = JoystickMotorControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        #node.serial_port.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()