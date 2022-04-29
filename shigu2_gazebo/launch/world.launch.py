import os
import pathlib
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

 
# this is the function launch  system will look for
 
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_name = 'shigu'
    world_file_name = 'playpen.world'
    world = os.path.join(get_package_share_directory('shigu2_gazebo'), 'worlds', world_file_name)
    urdf = os.path.join(get_package_share_directory('shigu2_description'), 'urdf', 'shigu.urdf')
    robot_description=pathlib.Path(os.path.join(get_package_share_directory('shigu2_description'), 'urdf', 'shigu.urdf')).read_text()
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
     
 
    # create and return launch description object
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=['-entity',
                    'shigu',
                    '-x', '3.5', '-y', '1.0', '-z', '1.0',
                    '-file', urdf
                    ]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf]),

        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher',
        # ),
    ])