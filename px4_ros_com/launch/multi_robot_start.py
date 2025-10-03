from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os 

def generate_launch_description():

    ld = LaunchDescription()

    pkg_share = get_package_share_directory('px4_ros_com')
    config = os.path.join(pkg_share, 'config', 'multi_robot_params.yaml')
    # Or you can load parameters for loop
    ld = load_drones(number_of_drones=3, ld=ld, config=config)


    return ld



def load_drones(number_of_drones, ld, config):
    for idx in range(1, number_of_drones + 1):
        drone_node = Node(
            package='px4_ros_com',
            executable='main_class',
            name=f'drone{idx}',
            parameters=[config, {'sys_id': idx}]
        )
        ld.add_action(drone_node)

    return ld