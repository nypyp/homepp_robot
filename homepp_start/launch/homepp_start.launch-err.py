import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    
    # base startup
    turn_on_wheeltec_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turn_on_wheeltec_robot'), 'launch'), 
            '/turn_on_wheeltec_robot.launch.py'])
    )
    
    # Camera startup
    realsense2_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch'), 
            '/rs_launch.py'])
    )
    
    #yolov8_ros starup
    yolov8_ros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('yolov8_bringup'), 'launch'), 
            '/yolov8.launch.py']
        ),
        launch_arguments={'input_image_topic'    : '/camera/color/image_raw',
                         'image_reliability'    : '1', 
                         'device'               : 'cuda:0',
                         }.items(),
    )

    # #homepp_follower starup
    # homepp_follower = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('homepp_follower'), 'launch'), 
    #         '/homepp_follower.launch.py']
    #     ),
    # )
    
    respeaker_ros2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('respeaker_ros2'), 'launch'), 
            '/respeaker.launch.py']
        ),
        launch_arguments={
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
    )
    
    
    
    return LaunchDescription([
        turn_on_wheeltec_robot,
        realsense2_camera,
        yolov8_ros,
        respeaker_ros2,
        Node(
            package='homepp_sherpa_onnx',
            namespace='keywordSpotter_from_topic',
            executable='keywordSpotter_from_topic',
            output='log',
            emulate_tty=True
        ),
        Node(
            package='homepp_doa_follower',
            namespace='doa_follower',
            executable='doa_follower',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='homepp_start',
            namespace='homepp_manager',
            executable='homepp_manager',
            output='log',
            emulate_tty=True
        ),
        Node(
            package='homepp_follower',
            namespace='yolotracker',
            executable='yoloTracker',
            output='log',
            emulate_tty=True,
        ),
        Node(
            package='homepp_sherpa_onnx',
            namespace='speechRecognition_from_topic',
            executable='speechRecognition_from_topic',
            output='screen',
            emulate_tty=True
        )

    ])
    