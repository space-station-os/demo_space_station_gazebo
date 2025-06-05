from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('ssos_new_description')
    xacro_file = os.path.join(share_dir, 'urdf', 'ssos_new.xacro')

   
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    
    gazebo_launch = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r', '--headless-rendering', 'empty.sdf'],
        output='screen'
    )

    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}]
    )

   
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=None  
    )

   
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'ssos_new', '-topic', 'robot_description'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity_node,
    ])
