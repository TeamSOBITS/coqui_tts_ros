#!/usr/bin/env python3
# coding: utf-8

import rospy
import rospkg

import subprocess
import requests
import codecs
import wave

from sobits_msgs.srv import TextToSpeech, TextToSpeechResponse


class CoquiTTSNode:
    def __init__(self):
        # Get parameters from launcher
        self.url         = rospy.get_param('~url', 'http://localhost:5002')
        self.addStopChar = rospy.get_param('~addStopChar', True)
        self.filename    = rospy.get_param('~filename', 'output.wav')
        self.speaker_id  = rospy.get_param('~speaker_id', 'p225')
        self.language_id = rospy.get_param('~language_id', '')
        self.style_wav   = rospy.get_param('~style_wav', '')
        self.sound_audio = rospy.get_param('~sound_audio', True)

        # Get path of the package
        rp = rospkg.RosPack()
        self.path = rp.get_path('coqui_tts_ros')
        self.filename = self.path + "/sounds/" + self.filename
        self.style_wav = self.path + "/sounds/" + self.style_wav if self.style_wav != '' else ''

        # Valid end of phrase characters
        self.VALID_END_OF_PHRASE = ['.',';','!','?']

        rospy.loginfo("[coqui_tts_ros] Coqui TTS node has been initialized.")
        rospy.loginfo("[coqui_tts_ros] Server URL: %s", self.url)
        rospy.loginfo("[coqui_tts_ros] Add stop character: %s", self.addStopChar)
        rospy.loginfo("[coqui_tts_ros] Output filename: %s", self.filename)
        rospy.loginfo("[coqui_tts_ros] Speaker id: %s", self.speaker_id)
        rospy.loginfo("[coqui_tts_ros] Language id: %s", self.language_id)
        rospy.loginfo("[coqui_tts_ros] Style wav: %s", self.style_wav)
        rospy.loginfo("[coqui_tts_ros] Sound audio: %s", self.sound_audio)

        # Create service
        rospy.Service('~tts', TextToSpeech, self.tts_request_callback)


    # Service callback
    def tts_request_callback(self, req):
        text = req.text
        text = codecs.decode(str(text).encode('utf-8'))
        rospy.loginfo("[coqui_tts_ros] Text to be converted to speech: %s", text)

        if self.textToSoundFile(text):
            return TextToSpeechResponse(True)
        else:
            return TextToSpeechResponse(False)

    # Add stop character if requested
    def endText(self, text):
        if self.addStopChar and text[-1] not in self.VALID_END_OF_PHRASE:
            text = text + "."
        return text
    
    def soundFileToAudio(self):
        # Get sound file duration
        duration = 0
        with wave.open(self.filename, 'r') as audio_file:
            frame_rate = audio_file.getframerate()
            n_frames = audio_file.getnframes()
            duration = n_frames / float(frame_rate)

        # Play sound file
        cmd = "aplay -q %s" % self.filename
        subprocess.Popen(cmd, shell=True)
        rospy.sleep(duration)

    # Convert text to sound file
    def textToSoundFile(self, text):
        if len(text) == 0:
            rospy.logerr("[coqui_tts_ros] No text has been specified.")
            return False

        try:
            req = requests.get(self.url + "/api/tts", params={'text': self.endText(text), 'speaker_id': self.speaker_id, 'language_id': self.language_id, 'style_wav': self.style_wav})

        except Exception as e:
            rospy.logerr("[coqui_tts_ros] Error calling Coqui TTS server api")
            rospy.logerr(e)
            return False
        
        if req.status_code == 200 and req.headers['Content-Type'] == 'audio/wav':
            rospy.loginfo("[coqui_tts_ros]  Valid audio has been returned from Coqui TTS api.")

            with open(self.filename, 'wb') as f:
                f.write(req.content)

            if self.sound_audio: self.soundFileToAudio()

            return True

        else:
            rospy.logwarn("[coqui_tts_ros]  No audio has been returned from Coqui TTS server api")

        return False


if __name__ == "__main__":
    rospy.init_node('coqui_tts_test', anonymous=True)
    coqui_tts_node = CoquiTTSNode()
    rospy.spin()
