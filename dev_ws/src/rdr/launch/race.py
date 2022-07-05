import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_filepath = launch.substitutions.LaunchConfiguration('config_filepath')
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('config_filepath', default_value=[
            launch.substitutions.TextSubstitution(text=os.path.join(
                get_package_share_directory('rdr'), 'config', '')),
            'hsv_cam', launch.substitutions.TextSubstitution(text='.config.yaml')]),

        Node(
            package='rdr',
            executable='hsv_cam',
            name='hsv_cam',
            parameters=[config_filepath]
        )
    ])