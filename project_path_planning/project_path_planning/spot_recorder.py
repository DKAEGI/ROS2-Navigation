import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import yaml

class SpotRecorder(Node):
    def __init__(self):
        super().__init__('spot_recorder')
        self.get_logger().info('Spot Recorder Node is running.')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.pose_callback,
            10  # Adjust the queue size as needed
        )
        self.positions = {}

    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        spot_label = input("Enter a label for this spot: ")

        # Create the data structure for this spot
        spot_data = {
            'ros__parameters': {
                'spot_name': spot_label,
                'position': {
                    'x': x,
                    'y': y
                },
                'orientation': {
                    'x': orientation.x,
                    'y': orientation.y,
                    'z': orientation.z,
                    'w': orientation.w
                }
            }
        }

        # Add the spot data to the positions dictionary
        self.positions[f'spot_{len(self.positions) + 1}'] = spot_data

        self.get_logger().info(f'Spot recorded: Label - {spot_label}, Position - ({x}, {y})')

    def save_positions_to_yaml(self):
        with open('spot_positions.yaml', 'w') as file:
            yaml.dump(self.positions, file)

def main(args=None):
    rclpy.init(args=args)
    node = SpotRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_positions_to_yaml()
        node.get_logger().info('Spot positions saved to spot_positions.yaml.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
