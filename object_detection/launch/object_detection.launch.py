# object_detection/launch/object_detection.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # ----- launch args -----
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/wrist_rgbd_depth_sensor/image_raw',
        description='Image topic to subscribe to'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 with preset config'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            get_package_share_directory('object_detection'),
            'rviz',
            'object_detection.rviz'
        ),
        description='Path to RViz2 config file'
    )

    namespace_arg = DeclareLaunchArgument(
        'ns',
        default_value='',
        description='Namespace'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)'
    )

    # ----- nodes -----
    detector_node = Node(
        package='object_detection',
        executable='object_detection',
        name='object_detection_node',
        namespace=LaunchConfiguration('ns'),
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic')
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=LaunchConfiguration('ns'),
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )

    return LaunchDescription([
        image_topic_arg,
        use_rviz_arg,
        rviz_config_arg,
        namespace_arg,
        log_level_arg,
        detector_node,
        rviz_node,
    ])
