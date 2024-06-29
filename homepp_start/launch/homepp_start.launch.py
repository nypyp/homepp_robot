import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

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

    # 声明node2的输出启动参数
    tracker_log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='warn',
        description='Set the log leverl for tracker (log, screen)'
    )

    #homepp_follower starup
    homepp_follower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('homepp_follower'), 'launch'), 
            '/homepp_follower.launch.py']
        ),
        launch_arguments={
            'tracker_log_level': LaunchConfiguration('log_level'),
        }.items(),
    )
    
    return LaunchDescription([
        turn_on_wheeltec_robot,
        realsense2_camera,
        yolov8_ros,
        tracker_log_level_arg,
        homepp_follower,

    ])
    