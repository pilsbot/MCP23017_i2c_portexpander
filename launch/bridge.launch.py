import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('portexpander'),
        'config',
        'params.yaml'
        )
        
    node=Node(
        package = 'portexpander',
        name = 'portexpander_i2c_bridge',
        executable = 'bridge',
        parameters = [config]
        #parameters = [{"i2c_device": "/dev/null"}]
    )
    ld.add_action(node)
    return ld