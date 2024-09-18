import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os

class RoverManagerNode(Node):
    def __init__(self):
        super().__init__('mode_manager')
        self.mode_manager = self.create_subscription(String, '/rover_mode', self.mode_callback, 10)
        self.manual_process = None
        self.autonomous_process = None

    def mode_callback(self, msg):
        mode = msg.data.lower()
        if mode == "manual":
            self.start_manual_mode()
        elif mode == "autonomous":
            self.start_autonomous_mode()
        else:
            self.get_logger().warn("UNKNOWN mode")
    
    def start_manual_mode(self):
        self.get_logger().info("Switching to Manual mode")
        
        if self.autonomous_process is not None:
            self.get_logger().info("KILLING Autonomous process")
            self.autonomous_process.terminate()
            self.autonomous_process.wait()
            self.autonomous_process = None
        
        if self.manual_process is None:
            self.get_logger().info("STARTING Manual process")
            # Sourcing the workspace and launching the file
            self.manual_process = subprocess.Popen(
                ['bash', '-c', 'source ~/RoombaV2Onboard/install/setup.bash && ros2 launch roombav2 microver.launch.py']
            )
    
    def start_autonomous_mode(self):
        self.get_logger().info("Switching to Autonomous mode")
        
        if self.manual_process is not None:
            self.get_logger().info("KILLING Manual process")
            self.manual_process.terminate()
            self.manual_process.wait()
            self.manual_process = None
        
        if self.autonomous_process is None:
            self.get_logger().info("STARTING Autonomous process")
            # Sourcing the workspace and launching the file
            self.autonomous_process = subprocess.Popen(
                ['bash', '-c', 'source ~/RoombaV2Onboard/install/setup.bash && ros2 launch roombav2 nav2_bringup.launch.py']
            )
    
    def shutdown(self):
        self.get_logger().info("SHUTTING DOWN ALL MODES")

        if self.manual_process is not None:
            self.manual_process.terminate()
            self.manual_process.wait()
        
        if self.autonomous_process is not None:
            self.autonomous_process.terminate()
            self.autonomous_process.wait()

def main(args=None):
    rclpy.init(args=args)
    rover_mode_manager = RoverManagerNode()

    try:
        rclpy.spin(rover_mode_manager)
    except KeyboardInterrupt:
        pass
    finally:
        rover_mode_manager.shutdown()
        rover_mode_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
