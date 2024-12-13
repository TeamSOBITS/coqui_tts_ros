#!/usr/bin/env python3
# coding: utf-8

import rclpy
from rclpy.node import Node
import subprocess
import requests
import codecs
import wave
import os
import time
from rclpy.action import ActionServer
from rclpy.action.server import GoalResponse, CancelResponse

from sobits_interfaces.action import TextToSpeech

class CoquiTTSNode(Node):
    def __init__(self):
        super().__init__('coqui_tts_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('url', 'http://localhost:5002'),
                ('addStopChar', True),
                ('filename', 'output.wav'),
                ('speaker_id', 'p225'),
                ('language_id', ''),
                ('style_wav', ''),
                ('sound_audio', True),
            ])

        self.url = self.get_parameter('url').get_parameter_value().string_value
        self.addStopChar = self.get_parameter('addStopChar').get_parameter_value().bool_value
        self.filename = self.get_parameter('filename').get_parameter_value().string_value
        self.speaker_id = self.get_parameter('speaker_id').get_parameter_value().string_value
        self.language_id = self.get_parameter('language_id').get_parameter_value().string_value
        self.style_wav = self.get_parameter('style_wav').get_parameter_value().string_value
        self.sound_audio = self.get_parameter('sound_audio').get_parameter_value().bool_value
        self.path = os.path.join(os.path.dirname(__file__), '..', 'sounds')
        self.filename = os.path.join(self.path, self.filename)
        self.style_wav = os.path.join(self.path, self.style_wav) if self.style_wav else ''
        self.VALID_END_OF_PHRASE = ['.', ';', '!', '?']

        self.action_server = ActionServer(
            self, TextToSpeech, 'tts',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

        self.get_logger().info('Action Server is ready...')

    def goal_callback(self, goal_request):
        self.get_logger().info("Received goal request")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        text = goal_handle.request.text
        text = codecs.decode(str(text).encode('utf-8'))
        self.get_logger().info(f"Processing TTS request: {text}")

        feedback_msg = TextToSpeech.Feedback()
        result = TextToSpeech.Result()

        try:
            if not self.textToSoundFile(text):
                self.get_logger().error("TTS processing failed.")
                goal_handle.abort()
                result.success = False
                return result

            with wave.open(self.filename, 'r') as audio_file:
                frame_rate = audio_file.getframerate()
                n_frames = audio_file.getnframes()
                duration = n_frames / float(frame_rate)

            cmd = f"ffplay -nodisp -autoexit {self.filename}"
            process = subprocess.Popen(cmd, shell=True)

            elapsed_time = 0.0
            interval = 0.1
            while elapsed_time < duration:
                time.sleep(interval)
                elapsed_time += interval
                feedback_msg.progress = elapsed_time
                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info(f"Progress: {elapsed_time:.1f}s ...")

            process.wait()

            feedback_msg.progress = duration
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info("TTS processing and playback completed.")

            goal_handle.succeed()
            result.success = True

        except Exception as e:
            self.get_logger().error(f"Execution error: {str(e)}")
            goal_handle.abort()
            result.success = False

        return result

    def endText(self, text):
        if self.addStopChar and text[-1] not in self.VALID_END_OF_PHRASE:
            text += "."
        return text

    def textToSoundFile(self, text):
        if len(text) == 0:
            self.get_logger().error("No text specified.")
            return False

        try:
            req = requests.get(
                f"{self.url}/api/tts",
                params={
                    'text': self.endText(text),
                    'speaker_id': self.speaker_id,
                    'language_id': self.language_id,
                    'style_wav': self.style_wav
                })
        except Exception as e:
            self.get_logger().error(f"API call error: {e}")
            return False

        if req.status_code == 200 and req.headers['Content-Type'] == 'audio/wav':
            with open(self.filename, 'wb') as f:
                f.write(req.content)
            return True

        self.get_logger().warn("No valid audio received.")
        return False

def main(args=None):
    rclpy.init(args=args)
    node = CoquiTTSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
