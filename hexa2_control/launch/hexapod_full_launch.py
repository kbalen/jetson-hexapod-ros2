#!/usr/bin/env python3
# hexapod_full_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_teleop = LaunchConfiguration('use_teleop', default='true')
    use_joy = LaunchConfiguration('use_joy', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_imu = LaunchConfiguration('use_imu', default='true')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        DeclareLaunchArgument(
            'use_teleop',
            default_value='true',
            description='Launch teleop keyboard controller'
        ),
        DeclareLaunchArgument(
            'use_joy',
            default_value='true',
            description='Launch joystick controller'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz for visualization'
        ),
        DeclareLaunchArgument(
            'use_imu',
            default_value='true',
            description='Use IMU for orientation feedback'
        ),
            
        # Load the robot description
        ExecuteProcess(
            cmd=['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
                '--ros-args', '-p', 'robot_description_file:=`rospack find hexa`/urdf/hexa.urdf.xacro',
                '-p', 'use_sim_time:='+LaunchConfiguration('use_sim_time')],
            shell=True,
            output='screen'
        ),
        
        # Launch the IMU node
        Node(
            package='hexa2_control',
            executable='hexapod_imu.py',
            name='hexapod_imu',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_imu)
        ),
        
        # Launch the kinematics node
        Node(
            package='hexa2_control',
            executable='hexapod_kinematics.py',
            name='hexapod_kinematics',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Launch the enhanced gait controller
        Node(
            package='hexa2_control',
            executable='hexapod_gait_controller_enhanced.py',
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
        
        # Launch the high-level controller
        Node(
            package='hexa2_control',
            executable='hexapod_controller.py',
            name='hexapod_controller',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Launch joy node for joystick input
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.1,
                'autorepeat_rate': 20.0,
            }],
            condition=IfCondition(use_joy)
        ),
        
        # Launch RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '$(find hexa2_control)/rviz/hexapod.rviz'],
            output='screen',
            condition=IfCondition(use_rviz)
        ),
        
        # Teleop control with keyboard (optional)
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e',
            condition=IfCondition(use_teleop)
        ),
        
        # Print launch complete message
        LogInfo(
            msg=["Hexapod control system launched! \n",
                 "Use keyboard or joystick to control the robot."]
        ),
    ])