import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
# from nav2_msgs.action import ClearEntireCostmap  # ClearEntireCostmap with issues in ROS2 Foxy
import yaml

class MoveToSpot(Node):
    def __init__(self):
        super().__init__('move_to_spot')
        self.get_logger().info('Move To Spot Node is running.')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # self.clear_costmaps_client = ActionClient(self, ClearEntireCostmap, 'clear_entire_costmap')  # ClearEntireCostmap with issues in ROS2 Foxy
        while not self.action_client.wait_for_server(timeout_sec=1.0):  # Wait for navigate_to_pose, not clear_entire_costmap
            self.get_logger().info('Action server /navigate_to_pose not available, waiting...')

    def move_to_spot(self, spot_name):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Load spot positions from the YAML file
        try:
            with open('/home/user/ros2_ws/src/project_path_planning/project_path_planning/spot_positions.yaml', 'r') as file:
                spot_positions = yaml.load(file, Loader=yaml.FullLoader)
        except FileNotFoundError:
            self.get_logger().error('spot_positions.yaml not found!')
            return

        if spot_name not in spot_positions:
            self.get_logger().error(f'Spot name "{spot_name}" not found in spot_positions.yaml!')
            return

        spot_info = spot_positions[spot_name]['ros__parameters']
        position = spot_info['position']
        orientation = spot_info['orientation']
        goal_msg.pose.pose.position.x = position['x']
        goal_msg.pose.pose.position.y = position['y']
        goal_msg.pose.pose.orientation.x = orientation['x']
        goal_msg.pose.pose.orientation.y = orientation['y']
        goal_msg.pose.pose.orientation.z = orientation['z']
        goal_msg.pose.pose.orientation.w = orientation['w']

        self.get_logger().info(f'Moving to spot: {spot_name}')
        future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            result = future.result()
            if result.result == NavigateToPose.Result.SUCCESS:
                self.get_logger().info('Navigation to spot succeeded.')
            else:
                self.get_logger().error('Navigation to spot failed.')
        else:
            self.get_logger().error('Failed to get response from the action server.')

        # ClearEntireCostmap with issues in ROS2 Foxy
        # clear_costmaps_goal = ClearEntireCostmap.Goal()
        # clear_future = self.clear_costmaps_client.send_goal_async(clear_costmaps_goal)
        # rclpy.spin_until_future_complete(self, clear_future)
        #
        # if clear_future.result() is not None:
        #     self.get_logger().info('Costmap cleared.')
        # else:
        #     self.get_logger().error('Failed to clear the costmap.')

def main(args=None):
    rclpy.init(args=args)
    node = MoveToSpot()
    try:
        while True:
            spot_name = input('Enter the spot name to move to (or "exit" to quit): ')
            if spot_name == "exit":
                break
            node.move_to_spot(spot_name)
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
