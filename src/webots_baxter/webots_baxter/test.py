import rclpy
from rclpy.node import Node
from webots_ros2_core.webots_node import WebotsNode

class SupervisorController(Node):
    def __init__(self):
        super().__init__('supervisor_controller')
        self.webots_node = WebotsNode(self)

        # Get the Supervisor instance.
        self.supervisor = self.webots_node.robot

        # Get the robot's root node.
        self.root = self.supervisor.getRoot()

        # Get the 'children' field of the root node.
        self.children_field = self.root.getField('children')

        # Import the PROTO node into the 'children' field.
        self.children_field.importMFNodeFromString(-1, 'Kinect')

def main(args=None):
    rclpy.init(args=args)

    supervisor_controller = SupervisorController()

    rclpy.spin(supervisor_controller)

    supervisor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
