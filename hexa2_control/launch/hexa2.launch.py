from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # Get the package directory
    hexa2_dir = get_package_share_directory('hexa2')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file = LaunchConfiguration('urdf_file', default=os.path.join(
                                        hexa2_dir, 'urdf', 'hexa.urdf.xacro'))
                                       
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true')
        
    declare_urdf_file = DeclareLaunchArgument(
        'urdf_file',
        default_value=os.path.join(hexa2_dir, 'urdf', 'hexa.urdf.xacro'),
        description='URDF/XACRO file for the hexapod')
        
    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', urdf_file])
        }],
        output='screen')
        
    # Servo controller node
    servo_controller = Node(
        package='hexa2_control',
        executable='servo_controller.py',
        name='hexa2_servo_controller',
        output='screen')
        
    # Gait controller node
    gait_controller = Node(
        package='hexa2_gait',
        executable='gait_controller.py',
        name='hexa2_gait_controller',
        output='screen')
        
    # Teleop keyboard node
    teleop_keyboard = Node(
        package='hexa2_teleop',
        executable='hexa2_teleop_key.py',
        name='hexa2_teleop_key',
        output='screen')
        
    # Launch description
    return LaunchDescription([
        # Launch Arguments
        declare_use_sim_time,
        declare_urdf_file,
        
        # Nodes
        robot_state_publisher,
        servo_controller,
        gait_controller,
        teleop_keyboard
    ])