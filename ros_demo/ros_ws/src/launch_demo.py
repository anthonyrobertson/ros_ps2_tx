import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([

        ### 
        #   Basic example of how to subscribe to /joy messages in a ROS2 project
        ### 

        ### rosbridge WebSocket server to communicate with the real robot's existing websocket API
        Node(
            package='rosbridge_server', # installed in Dockerfile "rosbridge-suite"
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[
                {'port': 9090}  # Port for WebSocket server
            ]
        ),

        # Launch the joy_listener script
        ExecuteProcess(
            cmd=['python3', os.path.join(os.path.dirname(__file__), 'joy_listener.py')],
            output='screen',
            name='joy_listener'
        )
    ])