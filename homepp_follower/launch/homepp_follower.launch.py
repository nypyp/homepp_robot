from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  
    return LaunchDescription([
      Node(
          package='homepp_follower',
          namespace='yolotracker',
          executable='yoloTracker',
          output='log',
          emulate_tty=True,
      ),
      Node(
          package='homepp_follower',
          namespace='yolofollower',
          executable='yoloFollower',
          output='screen',
          emulate_tty=True,
      ),
    ])