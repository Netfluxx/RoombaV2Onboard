#RPi ReceiverNode

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
from serial.tools import list_ports
class ReceiveNode(Node):
    def __init__(self):
        super().__init__('receive_node')
        self.serial_port = self.detect_serial_port()

        if not self.serial_port:
            self.get_logger().error("No valid serial port found")
            raise RuntimeError("No valid serial port found")

        # Subscribe to the input topic
        self.subscription = self.create_subscription(
            String,
            'rover_input',
            self.message_callback,
            10
        )

    def detect_serial_port(self):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if 'USB' in port.description:
                if '1A86:7523' in port.hwid: #master arduino hwid number
                    try:
                        print(port.hwid)
                        serial_port = serial.Serial(port.device, 9600, timeout=1)
                        self.get_logger().info(f"Connected to serial port: {port.device}")
                        return serial_port
                    except serial.SerialException as e:
                        self.get_logger().error(f"Failed to open serial port {port.device}: {e}")
        return None

        #TODO: id master arduino nano : port.hwid  or alternatively port.vid, port.pid, port.serial_number, port.product

    def message_callback(self, msg):
        self.serial_port.write((msg.data + '\n').encode('utf-8'))
        received_from_arduino = self.serial_port.readline().decode('utf-8').strip()

        if received_from_arduino:
            self.get_logger().info(f"Received from Arduino: {received_from_arduino}")
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


#new message structure between RPi and Arduino:

# RPI ID, Master ACK, FR_ACK, FL_ACK, BR_ACK, BL_ACK, FR_SPEED, FL_SPEED, BR_SPEED, BL_SPEED
