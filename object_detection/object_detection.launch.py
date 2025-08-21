from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    x = DeclareLaunchArgument('x', default_value='0.0')
    y = DeclareLaunchArgument('y', default_value='0.0')
    z = DeclareLaunchArgument('z', default_value='0.0')
    roll  = DeclareLaunchArgument('roll',  default_value='0.0')
    pitch = DeclareLaunchArgument('pitch', default_value='0.0')
    yaw   = DeclareLaunchArgument('yaw',   default_value='0.0')
    world = DeclareLaunchArgument('world_frame', default_value='base_link')
    cam   = DeclareLaunchArgument('camera_optical_frame', default_value='wrist_rgbd_camera_depth_optical_frame')
    topic = DeclareLaunchArgument('pointcloud_topic', default_value='/wrist_rgbd_depth_sensor/points')

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_cam',
        arguments=[
            LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z'),
            LaunchConfiguration('roll'), LaunchConfiguration('pitch'), LaunchConfiguration('yaw'),
            LaunchConfiguration('world_frame'), LaunchConfiguration('camera_optical_frame')
        ]
    )

    perception = Node(
        package='object_detection',
        executable='object_detection.py',
        name='object_detection',
        output='screen',
        parameters=[{
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            'camera_frame': LaunchConfiguration('camera_optical_frame'),
            'world_frame': LaunchConfiguration('world_frame')
        }]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        # You can switch to a saved config later
    )

    return LaunchDescription([x,y,z,roll,pitch,yaw,world,cam,topic, static_tf, perception, rviz])
