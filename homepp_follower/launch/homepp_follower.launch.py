from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
  
    # 声明日志等级启动参数
    tracker_log_level_arg = DeclareLaunchArgument(
        'tracker_log_level',
        default_value='warn',
        description='Set the log level (debug, info, warn, error, fatal)'
    )
    
    tracker_log_level = LaunchConfiguration('tracker_log_level')
  
    return LaunchDescription([
      tracker_log_level_arg,
      Node(
          package='homepp_follower',
          namespace='yolotracker',
          executable='yoloTracker',
          output='log',
          emulate_tty=True,
          parameters=[{'log_level': tracker_log_level}]
          
      ),
      Node(
          package='homepp_follower',
          namespace='yolofollower',
          executable='yoloFollower',
          output='screen'
      ),
    ])