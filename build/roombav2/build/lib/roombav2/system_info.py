import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import psutil

class SystemInfoPublisher(Node):
    def __init__(self):
        super().__init__('system_info_publisher')

        self.publisher_ = self.create_publisher(String, 'system_info', 10)
        self.timer = self.create_timer(3.0, self.publish_system_info)

    def publish_system_info(self):
        try:
            temp_result = subprocess.Popen(
                ['bash', '-c', 'cat /sys/class/thermal/thermal_zone0/temp | awk \'{print $1/1000}\''],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            stdout, stderr = temp_result.communicate()

            if temp_result.returncode == 0:
                temperature_str = stdout.decode().strip()
            else:
                temperature_str = "Error getting temperature"

            cpu_utilization = psutil.cpu_percent(interval=None)
            memory_info = psutil.virtual_memory()
            ram_utilization = memory_info.percent

            system_info_msg = String()
            system_info_msg.data = (
                f"{temperature_str},{cpu_utilization:.2f},{ram_utilization:.2f}")  # °C, %, %

            self.publisher_.publish(system_info_msg)

        except Exception as e:
            self.get_logger().error(f"Failed to gather system information: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = SystemInfoPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
