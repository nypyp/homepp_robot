from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='homepp_sherpa_onnx',
            namespace='keywordSpotter_from_topic',
            executable='keywordSpotter_from_topic',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='homepp_sherpa_onnx',
            namespace='speechRecognition_from_topic',
            executable='speechRecognition_from_topic',
            output='screen',
            emulate_tty=True
        )
    ])