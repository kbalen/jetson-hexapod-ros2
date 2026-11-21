#!/usr/bin/env python3
# hexapod_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'),
            
        # Load the robot description
        ExecuteProcess(
            cmd=['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
                '--ros-args', '-p', 'robot_description_file:=`rospack find hexa`/urdf/hexa.urdf.xacro',
                '-p', ['use_sim_time:=', use_sim_time]],
            shell=True,
            output='screen'
        ),
        
        # Launch the kinematics node
        Node(
            package='hexa2_control',
            executable='hexapod_kinematics.py',
            name='hexapod_kinematics',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Launch the gait controller
        Node(
            package='hexa2_control',
            executable='hexapod_gait_controller.py',
            name='hexapod_gait_controller',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Launch the servo controller
        Node(
            package='hexa2_control',
            executable='hexapod_servo_controller.py',
            name='hexapod_servo_controller',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Launch RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '$(find hexa2_control)/rviz/hexapod.rviz'],
            output='screen'
        ),
        
        # Teleop control with keyboard (optional)
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e'
        )
    ])