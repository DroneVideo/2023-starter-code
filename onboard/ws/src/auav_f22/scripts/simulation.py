import gymnasium as gym

import rclpy
from rclpy.node import Node

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode

from std_msgs.msg import Float32MultiArray

class Environment(Node, gym.Env):

    def __init__(self):
        super().__init__('sim_env')

        self.pose_publisher_ = self.create_publisher(Float32MultiArray, "/set_pose", 10)

        timer_period = 0.1  # 100 milliseconds
        self.timer_ = self.create_timer(timer_period, self.reset)

    def reset(self):
        rover_pos = [0.0, 2.0, 0.2, 0.0, 0.0, 1.0, 1.0]
        drone_pos = [0.0, 0.0, 0.1, 0.0, 0.0, 1.0, 1.0]

        poses = rover_pos + drone_pos

        msg = Float32MultiArray()
        msg.data = poses

        self.pose_publisher_.publish(msg)
        self.get_logger().info('Sent reset')
    
    def step(self, action):
        pass

def main(args=None):
    rclpy.init(args=args)
    sim_env = Environment()
    rclpy.spin(sim_env)
    sim_env.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
     
