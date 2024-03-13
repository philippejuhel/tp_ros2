import rclpy
from geometry_msgs.msg import Twist, PoseStamped
from visualization_msgs.msg import Marker
from time import sleep


def main():
  rclpy.init()
  # Create the ROS2 node
  node = rclpy.create_node('square_mover_visualizer')

  # Define movement parameters
  linear_vel = 0.2  # m/s
  angular_vel = 0.5  # rad/s
  side_length = 1  # meter
  time_per_side = side_length / linear_vel  # seconds

  # Create publishers
  vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
  marker_pub = node.create_publisher(Marker, 'trajectory_markers', 10)

  # Initialize robot poses list
  robot_poses = []
  while(True):
    # Move in a square pattern and publish markers
    for _ in range(4):
        # Move forward
        msg = Twist()
        msg.linear.x = linear_vel
        vel_pub.publish(msg)
        sleep(time_per_side)

        # Get current robot pose (replace with actual data source)
        current_pose = get_robot_pose(node)  # Replace with your pose retrieval function
        robot_poses.append(current_pose)

        # Turn 90 degrees and publish marker
        msg.linear.x = 0.0
        msg.angular.z = angular_vel
        vel_pub.publish(msg)
        sleep(time_per_side / angular_vel)  # Adjust for turning time

        current_pose = get_robot_pose(node)  # Get pose after turn
        robot_poses.append(current_pose)

    # Stop the robot
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    vel_pub.publish(msg)

    # Publish final marker for complete trajectory
    publish_trajectory_marker(marker_pub, node, robot_poses)

  # Shutdown the node
  node.destroy_node()
  rclpy.shutdown()


# Function to retrieve robot pose (replace with your actual implementation)
def get_robot_pose(node):
  # Replace with your code to get the robot's current pose (e.g., using odometry)
  # This is a placeholder, you need to implement how to get the actual pose.
  pose = PoseStamped()
  pose.header.stamp = node.get_clock().now().to_msg()
  # Fill pose data with actual robot position and orientation
  return pose


# Function to publish a marker for the entire trajectory
def publish_trajectory_marker(marker_pub, node, poses):
  marker = Marker()
  marker.header.stamp = node.get_clock().now().to_msg()
  marker.header.frame_id = 'odom'  # Replace with your robot's base frame ID
  marker.ns = "trajectory"
  marker.id = 0  # Single marker for the whole trajectory
  marker.type = Marker.LINE_STRIP  # Display as a line
  marker.scale.x = 0.1  # Line thickness
  marker.color.r = 1.0  # Red color
  marker.color.g = 0.0  # Green color
  marker.color.b = 0.0  # Blue color
  marker.color.a = 1.0  # Full opacity

  # Add each point (x, y) from all poses
  marker.points = [p.pose.position for p in poses]
  marker_pub.publish(marker)


if __name__ == '__main__':
  main()
