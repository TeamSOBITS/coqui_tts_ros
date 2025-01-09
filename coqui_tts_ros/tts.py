#!/usr/bin/env python3
# coding: utf-8

import rclpy
from rclpy.node import Node
import pygame
import requests
import codecs
import wave
import os
import time
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from sobits_interfaces.action import TextToSpeech

class CoquiTTSNode(Node):
    def __init__(self):
        super().__init__('coqui_tts_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('url', 'http://localhost:5002'),
                ('addStopChar', True),
                ('speaker_id', 'p225'),
                ('language_id', ''),
                ('style_wav', ''),
                ('sound_audio', True),
            ])

        self.url = self.get_parameter('url').get_parameter_value().string_value
        self.addStopChar = self.get_parameter('addStopChar').get_parameter_value().bool_value
        self.speaker_id = self.get_parameter('speaker_id').get_parameter_value().string_value
        self.language_id = self.get_parameter('language_id').get_parameter_value().string_value
        self.style_wav = self.get_parameter('style_wav').get_parameter_value().string_value
        self.sound_audio = self.get_parameter('sound_audio').get_parameter_value().bool_value
        self.path = os.path.join(os.path.dirname(__file__), '..', 'sounds')
        self.filename = os.path.join(get_package_share_directory('coqui_tts_ros'), 'sounds', 'output.wav')
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
        thread_node = Node("execute_callback")
        feedback = TextToSpeech.Feedback()
        response = TextToSpeech.Result()

        text = goal_handle.request.text
        text = codecs.decode(str(text).encode('utf-8'))
        self.get_logger().info(f"Processing TTS request: {text}")

        response.success = False
        response.total_time = 0.0

        try:
            if not self.textToSoundFile(text):
                self.get_logger().error("TTS processing failed.")
                goal_handle.abort()
                thread_node.destroy_node()
                del thread_node
                return response

            with wave.open(self.filename, 'r') as audio_file:
                frame_rate = audio_file.getframerate()
                n_frames = audio_file.getnframes()
                duration = n_frames / float(frame_rate)

            pygame.mixer.init()
            pygame.mixer.music.load(self.filename)
            pygame.mixer.music.play()

            interval = 0.1
            feedback.remaining_time = duration
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Goal canceled')
                    if pygame.mixer.music.get_busy():
                        pygame.mixer.music.stop()
                    goal_handle.canceled()
                    thread_node.destroy_node()
                    del thread_node
                    return response

                rclpy.spin_once(thread_node, timeout_sec=0.1)
                response.total_time += interval
                feedback.remaining_time -= interval

                if (feedback.remaining_time <= 0.0):
                    break
                else:
                    goal_handle.publish_feedback(feedback)

            feedback.remaining_time = 0.0
            goal_handle.publish_feedback(feedback)
            self.get_logger().info("TTS processing and playback completed.")

            response.success = True
            goal_handle.succeed()

        except Exception as e:
            self.get_logger().error(f"Execution error: {str(e)}")
            goal_handle.abort()

        thread_node.destroy_node()
        del thread_node
        return response

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
