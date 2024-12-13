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

from sobits_interfaces.srv import TextToSpeech


class CoquiTTSNode(Node):
    def __init__(self):
        super().__init__('coqui_tts_node')

        # Get parameters from launcher
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

        # Get path of the package
        self.path = os.path.join(os.path.dirname(__file__), '..', 'sounds')
        self.filename = os.path.join(self.path, self.filename)
        self.style_wav = os.path.join(self.path, self.style_wav) if self.style_wav else ''

        # Valid end of phrase characters
        self.VALID_END_OF_PHRASE = ['.', ';', '!', '?']

        self.get_logger().info("Coqui TTS node has been initialized.")
        self.get_logger().info(f"Server URL: {self.url}")
        self.get_logger().info(f"Add stop character: {self.addStopChar}")
        self.get_logger().info(f"Output filename: {self.filename}")
        self.get_logger().info(f"Speaker id: {self.speaker_id}")
        self.get_logger().info(f"Language id: {self.language_id}")
        self.get_logger().info(f"Style wav: {self.style_wav}")
        self.get_logger().info(f"Sound audio: {self.sound_audio}")

        # Create service
        self.srv = self.create_service(TextToSpeech, 'tts', self.tts_request_callback)

    # Service callback
    def tts_request_callback(self, request, response):
        text = request.text
        text = codecs.decode(str(text).encode('utf-8'))
        self.get_logger().info(f"Text to be converted to speech: {text}")

        if self.textToSoundFile(text):
            response.result = True
        else:
            response.result = False

        return response

    # Add stop character if requested
    def endText(self, text):
        if self.addStopChar and text[-1] not in self.VALID_END_OF_PHRASE:
            text += "."
        return text
    
    def soundFileToAudio(self):
        # Get sound file duration
        duration = 0
        with wave.open(self.filename, 'r') as audio_file:
            frame_rate = audio_file.getframerate()
            n_frames = audio_file.getnframes()
            duration = n_frames / float(frame_rate)

        # Play sound file using ffplay
        cmd = f"ffplay -nodisp -autoexit {self.filename}"
        subprocess.Popen(cmd, shell=True)

        # Use time.sleep to wait for the audio duration
        time.sleep(duration)

    # Convert text to sound file
    def textToSoundFile(self, text):
        if len(text) == 0:
            self.get_logger().error("No text has been specified.")
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
            self.get_logger().error("Error calling Coqui TTS server api")
            self.get_logger().error(str(e))
            return False
        
        if req.status_code == 200 and req.headers['Content-Type'] == 'audio/wav':
            self.get_logger().info("Valid audio has been returned from Coqui TTS api.")

            with open(self.filename, 'wb') as f:
                f.write(req.content)

            if self.sound_audio:
                self.soundFileToAudio()

            return True

        else:
            self.get_logger().warn("No audio has been returned from Coqui TTS server api")

        return False


def main(args=None):
    rclpy.init(args=args)
    node = CoquiTTSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
