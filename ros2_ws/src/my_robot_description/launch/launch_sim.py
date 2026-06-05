'''
The purpose of this file is launch ros with all the configuration, robot, and world models
'''

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # including name for package located in the package.xml
    package_name = 'my_robot_description'

    # launching rsp file
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    user_input_arg = DeclareLaunchArgument(
        'user_input',
        default_value="hello",
        description="user passing in string to"
    )
    
    # launching gazebo bridge provided by gazebo ros package
    ros_bridge = Node( 
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Camera bridges (Gazebo -> ROS 2)
            '/robot/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            

            
            # Transform bridge (Gazebo -> ROS 2)
            '/model/robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'

                ],
        # Remap the namespaced topics back to standard clean ROS 2 topics
        # so your teleop keyboard and YOLO node can read/write seamlessly.
        remappings=[
            ('/model/robot/tf', '/tf'),
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    yolo_node = Node(
        package='my_yolo_detector',
        executable='yolo',
        name='yolo',
        output='screen',
        parameters=[{'use_sim_time': True, 'target_param': LaunchConfiguration('user_input')}]
    )

        
    # launching all of them together
    return LaunchDescription([
        rsp,
        ros_bridge,
        user_input_arg,
        yolo_node,
    ])