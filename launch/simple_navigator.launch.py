from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    tb3_pkg = get_package_share_directory('turtlebot3_gazebo')
    world = os.path.join(tb3_pkg, 'worlds', 'turtlebot3_world.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_pkg, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={'world_file': world}.items()
    )

    planner = Node(
        package='turtlebot3_simple_navigator',
        executable='simple_navigator',
        name='turtlebot3_simple_navigator',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('turtlebot3_simple_navigator'),
            'config', 'params.yaml'
        )]
    )

    return LaunchDescription([gazebo, planner])