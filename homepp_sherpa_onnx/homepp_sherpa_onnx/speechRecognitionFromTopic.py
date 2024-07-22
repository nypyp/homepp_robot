#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sherpa_onnx import OnlineRecognizer
from pathlib import Path
import threading
import websocket
import time
import os
from ament_index_python.packages import get_package_share_directory
from audio_common_msgs.msg import AudioData
import numpy as np

get_result_event = threading.Event()
recording_results = ""

def message_sender():
    global recording_results
    websocket.enableTrace(False)
    server = "ws://8.134.150.174:8000/voicechat"
    ws = websocket.WebSocketApp(server)

    def on_open(ws):
        print("[INFO] messageSender: WebSocket on open")
        while not get_result_event.is_set():
            time.sleep(1)
        print("[INFO] messageSender: Send %s to %s", recording_results, server)
        ws.send(recording_results)
        get_result_event.clear()

    def on_message(ws, message):
        print("[INFO] messageSender: Received from server:", message)
        on_open(ws)

    def on_close(ws, close_code, close_reason):
        print("[INFO] messageSender: WebSocket connection closed")
        on_open(ws)

    ws.on_open = on_open
    ws.on_message = on_message
    ws.on_close = on_close
    print("[INFO] websocket inited...")

    ws.run_forever()

def assert_file_exists(filename: str):
    assert Path(filename).is_file(), (
        f"{filename} does not exist!\n"
        "Please refer to "
        "https://k2-fsa.github.io/sherpa/onnx/pretrained_models/index.html to download it"
    )

class SpeechRecognitionNode(Node):
    def __init__(self):
        super().__init__('speech_recognition_node')

        package_share_path = get_package_share_directory('homepp_sherpa_onnx')
        workspace_path = os.path.abspath(os.path.join(package_share_path, '..', '..','..','..'))
        package_src_path = os.path.join(workspace_path, 'src', 'homepp_sherpa_onnx')
        self.declare_parameter('tokens', package_src_path+'/streaming-zipformer-bil/tokens.txt')
        self.declare_parameter('encoder', package_src_path+'/streaming-zipformer-bil/encoder-epoch-99-avg-1.onnx')
        self.declare_parameter('decoder', package_src_path+'/streaming-zipformer-bil/decoder-epoch-99-avg-1.onnx')
        self.declare_parameter('joiner', package_src_path+'/streaming-zipformer-bil/joiner-epoch-99-avg-1.onnx')
        self.declare_parameter('decoding_method', 'greedy_search')
        self.declare_parameter('provider', 'cpu')
        self.declare_parameter('hotwords_file', '')
        self.declare_parameter('hotwords_score', 1.5)
        self.declare_parameter('blank_penalty', 0.0)
        self.declare_parameter('device_name', 'plughw:0,0')

        self.args = {
            'tokens': self.get_parameter('tokens').value,
            'encoder': self.get_parameter('encoder').value,
            'decoder': self.get_parameter('decoder').value,
            'joiner': self.get_parameter('joiner').value,
            'decoding_method': self.get_parameter('decoding_method').value,
            'provider': self.get_parameter('provider').value,
            'hotwords_file': self.get_parameter('hotwords_file').value,
            'hotwords_score': self.get_parameter('hotwords_score').value,
            'blank_penalty': self.get_parameter('blank_penalty').value,
            'device_name': self.get_parameter('device_name').value,
        }

        self.recognizer = None
        self.init_recognizer()
        self.stream = self.recognizer.create_stream()
        self.segment_id = 0
        self.last_result = ''

        # Subscriber for audio data
        self.audio_sub = self.create_subscription(AudioData, 'audio', self.audio_callback, 10)

    def init_recognizer(self):
        assert_file_exists(self.args['encoder'])
        assert_file_exists(self.args['decoder'])
        assert_file_exists(self.args['joiner'])
        assert_file_exists(self.args['tokens'])

        self.recognizer = OnlineRecognizer.from_transducer(
            tokens=self.args['tokens'],
            encoder=self.args['encoder'],
            decoder=self.args['decoder'],
            joiner=self.args['joiner'],
            num_threads=1,
            sample_rate=16000,
            feature_dim=80,
            enable_endpoint_detection=True,
            rule1_min_trailing_silence=2.4,
            rule2_min_trailing_silence=1.2,
            rule3_min_utterance_length=300,  # it essentially disables this rule
            decoding_method=self.args['decoding_method'],
            provider=self.args['provider'],
            hotwords_file=self.args['hotwords_file'],
            hotwords_score=self.args['hotwords_score'],
            blank_penalty=self.args['blank_penalty'],
        )

    def audio_callback(self, msg):
        samples = np.frombuffer(msg.data, dtype=np.int16).astype('float32') / 32768.0
        self.process_audio(samples)

    def process_audio(self, samples):
        self.stream.accept_waveform(16000, samples)
        while self.recognizer.is_ready(self.stream):
            self.recognizer.decode_stream(self.stream)

        is_endpoint = self.recognizer.is_endpoint(self.stream)
        result = self.recognizer.get_result(self.stream)

        if result and (self.last_result != result):
            result = result.replace('小月', '小悦')
            print("\r{}:{}".format(self.segment_id, result), end="", flush=True)

        if is_endpoint:
            if result:
                result = result.replace('小月', '小悦')
                print("\r{}:{}".format(self.segment_id, result), flush=True)
                if '小悦' in result:
                    result = result.replace('小悦', '')
                    global recording_results
                    recording_results = result
                    get_result_event.set()
                self.segment_id += 1
            self.recognizer.reset(self.stream)

    def main_loop(self):
        while rclpy.ok():
            rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)
    node = SpeechRecognitionNode()
    print("[INFO] try to start sender...")
    sender = threading.Thread(target=message_sender, name='messageSender')
    sender.start()
    node.main_loop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
