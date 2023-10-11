import rclpy
from rclpy.node import Node
from project_custom_interfaces.srv import MyServiceMessage  # Import custom service message
from geometry_msgs.msg import PoseWithCovarianceStamped

class SpotRecorderNode(Node):
    def __init__(self):
        super().__init__('spot_recorder')
        self.service = self.create_service(
            MyServiceMessage,
            'save_spot',
            self.save_spot_callback
        )
        self.robot_coordinates = None  # Initialize coordinates as None
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',  # Replace with the actual topic name
            self.pose_callback,
            10  # Adjust the queue size as needed
        )
        
# Call the service by: ros2 service call /save_spot project_custom_interfaces/srv/MyServiceMessage "label: 'spot1'"
    def save_spot_callback(self, request, response):
        # Check if robot_coordinates is available
        if self.robot_coordinates:
            # Store the coordinates and label in a file (e.g., spots.txt)
            label = request.label
            with open('spots.txt', 'a') as f:
                f.write(f'Label: {label}, Coordinates: {self.robot_coordinates}\n')
            response.navigation_successfull = True
            response.message = 'Spot saved successfully'
        else:
            response.navigation_successfull = False
            response.message = 'Robot coordinates not available'
        return response

    def pose_callback(self, msg):
        # Extract the position and orientation information from the received message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        orientation = msg.pose.pose.orientation
        # Store the robot's current pose (position and orientation)
        self.robot_coordinates = f'x: {x}, y: {y}, z: {z}, ' \
                            f'orientation: x: {orientation.x}, y: {orientation.y}, ' \
                            f'z: {orientation.z}, w: {orientation.w}'


def main(args=None):
    rclpy.init(args=args)
    node = SpotRecorderNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
