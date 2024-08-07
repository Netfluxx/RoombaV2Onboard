#RPi ReceiverNode

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
from serial.tools import list_ports
import time
import string

class ReceiveNode(Node):
    def __init__(self):
        super().__init__('receive_node')
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
            String,
            'rover_input',
            self.message_callback,
            10
        )

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

    def message_callback(self, msg):
        if "RV2" in msg.data:
            stripped_msg = msg.data.replace("RV2", "")
            #send command to master arduino
            self.serial_port.write((stripped_msg + '\n').encode('utf-8'))
        
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
    node = ReceiveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_port.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
