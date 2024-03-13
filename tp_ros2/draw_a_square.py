import rclpy
from geometry_msgs.msg import Twist
from time import sleep
import math


def main():
  rclpy.init()
  node = rclpy.create_node('square_mover')

  # Define movement parameters
  linear_vel = 0.2  # m/s
  angular_vel = 0.5  # rad/s
  side_length = 1  # meter
  time_per_side = side_length / linear_vel  # seconds

  # Create a Twist message publisher
  vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)

  # Move in a square pattern
  for _ in range(4):
    # Move forward
    msg = Twist()
    msg.linear.x = linear_vel
    vel_pub.publish(msg)
    sleep(time_per_side)

    # Turn 90 degrees
    msg.linear.x = 0.0
    msg.angular.z = angular_vel
    vel_pub.publish(msg)
    sleep(math.pi / 2 / angular_vel)  # Adjust for turning time

  # Stop the robot
  msg.linear.x = 0.0
  msg.angular.z = 0.0
  vel_pub.publish(msg)

  # Shutdown the node
  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()