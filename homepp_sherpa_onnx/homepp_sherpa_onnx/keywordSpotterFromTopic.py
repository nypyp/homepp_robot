#!/usr/bin/env python3

import argparse
import sys
from pathlib import Path

from typing import List

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from std_msgs.msg import String, Int8
from audio_common_msgs.msg import AudioData
from ament_index_python.packages import get_package_share_directory
import os

try:
    import sounddevice as sd
except ImportError:
    print("Please install sounddevice first. You can use")
    print()
    print("  pip install sounddevice")
    print()
    print("to install it")
    sys.exit(-1)

import sherpa_onnx

class KeywordsSpotterNode(Node):

    def __init__(self):
        super().__init__('keywords_spotter_node')

        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.get_logger().debug('Initializing keywords_spotter_node')
        package_share_path = get_package_share_directory('homepp_sherpa_onnx')
        workspace_path = os.path.abspath(os.path.join(package_share_path, '..', '..','..','..'))
        package_src_path = os.path.join(workspace_path, 'src', 'homepp_sherpa_onnx')
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('tokens', package_src_path+'/kws-zipformer-wenetspeech/tokens.txt'),
                ('encoder', package_src_path+'/kws-zipformer-wenetspeech/encoder-epoch-12-avg-2-chunk-16-left-64.onnx'),
                ('decoder', package_src_path+'/kws-zipformer-wenetspeech/decoder-epoch-12-avg-2-chunk-16-left-64.onnx'),
                ('joiner', package_src_path+'/kws-zipformer-wenetspeech/joiner-epoch-12-avg-2-chunk-16-left-64.onnx'),
                ('num_threads', 1),
                ('provider', 'cpu'),
                ('max_active_paths', 4),
                ('num_trailing_blanks', 1),
                ('keywords_file', package_src_path+'/keywords.txt'),
                ('keywords_score', 1.0),
                ('keywords_threshold', 0.25),
            ]
        )

        # Get parameters
        self.tokens = self.get_parameter('tokens').value
        self.encoder = self.get_parameter('encoder').value
        self.decoder = self.get_parameter('decoder').value
        self.joiner = self.get_parameter('joiner').value
        self.num_threads = self.get_parameter('num_threads').value
        self.provider = self.get_parameter('provider').value
        self.max_active_paths = self.get_parameter('max_active_paths').value
        self.num_trailing_blanks = self.get_parameter('num_trailing_blanks').value
        self.keywords_file = self.get_parameter('keywords_file').value
        self.keywords_score = self.get_parameter('keywords_score').value
        self.keywords_threshold = self.get_parameter('keywords_threshold').value

        # Initialize keyword spotter
        self.get_logger().debug('Initializing KeywordSpotter')
        self.keyword_spotter = sherpa_onnx.KeywordSpotter(
            tokens=self.tokens,
            encoder=self.encoder,
            decoder=self.decoder,
            joiner=self.joiner,
            num_threads=self.num_threads,
            max_active_paths=self.max_active_paths,
            keywords_file=self.keywords_file,
            keywords_score=self.keywords_score,
            keywords_threshold=self.keywords_threshold,
            num_trailing_blanks=self.num_trailing_blanks,
            provider=self.provider,
        )

        # Setup sound device stream
        self.sample_rate = 16000
        self.samples_per_read = int(0.1 * self.sample_rate)  # 0.1 second = 100 ms
        self.stream = self.keyword_spotter.create_stream()

        # Publisher for keyword detection result
        qos = QoSProfile(depth=10)
        self.keyword_pub = self.create_publisher(String, 'keyword_detection', qos)

        # Subscriber for audio data
        self.audio_sub = self.create_subscription(AudioData, 'audio', self.audio_callback, qos)

        self.get_logger().info('Keyword spotter node is running.')
        rclpy.spin(self)

    def audio_callback(self, msg):
        # self.get_logger().info('received audio callback')
        samples = np.frombuffer(msg.data, dtype=np.int16).astype('float32') / 32768.0
        self.stream.accept_waveform(self.sample_rate, samples)
        while self.keyword_spotter.is_ready(self.stream):
            self.keyword_spotter.decode_stream(self.stream)
        result = self.keyword_spotter.get_result(self.stream)
        if result:
            self.get_logger().debug(f'Detected keyword: {result}')
            msg = String()
            msg.data = result
            self.keyword_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeywordsSpotterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
