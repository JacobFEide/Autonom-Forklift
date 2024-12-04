import rclpy
from rclpy.node import Node
from pynput import keyboard
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
import os
from ament_index_python.packages import get_package_share_directory


class RobotReloader(Node):
    def __init__(self):
        super().__init__('robot_reloader')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        
        self.robot_name = 'trucken'
        self.urdf_path = os.path.join(
            get_package_share_directory('trucken'), 'urdf', 'trucken.urdf')
        
        # Lytt etter tastetrykk
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()
        self.get_logger().info('Press "k" to reload the robot in Gazebo')

    def on_press(self, key):
        try:
            if key.char == 'k':
                self.get_logger().info('Reloading robot...')
                self.reload_robot()
        except AttributeError:
            pass

    def reload_robot(self):
        # Slett roboten
        delete_request = DeleteEntity.Request()
        delete_request.name = self.robot_name
        self.delete_client.call_async(delete_request)

        # Spawn robot på nytt
        with open(self.urdf_path, 'r') as urdf_file:
            urdf = urdf_file.read()

        spawn_request = SpawnEntity.Request()
        spawn_request.name = self.robot_name
        spawn_request.xml = urdf
        spawn_request.robot_namespace = ''
        spawn_request.initial_pose.position.x = 0.0
        spawn_request.initial_pose.position.y = 0.0
        spawn_request.initial_pose.position.z = 0.1

        self.spawn_client.call_async(spawn_request)

def main(args=None):
    rclpy.init(args=args)
    reloader = RobotReloader()
    rclpy.spin(reloader)
    reloader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
