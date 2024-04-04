#!/usr/bin/env python3
# coding: utf-8

import rospy
import rospkg
import requests
import codecs

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

        # Get path of the package
        rp = rospkg.RosPack()
        self.path = rp.get_path('coqui_tts_ros')
        self.filename = self.path + "/sounds/" + self.filename
        self.style_wav = self.path + "/sounds/" + self.style_wav if self.style_wav != '' else ''

        # Valid end of phrase characters
        self.VALID_END_OF_PHRASE = ['.',';','!','?']

        rospy.loginfo("Coqui TTS node has been initialized.")
        rospy.loginfo("Server URL: %s", self.url)
        rospy.loginfo("Add stop character: %s", self.addStopChar)
        rospy.loginfo("Output filename: %s", self.filename)
        rospy.loginfo("Speaker id: %s", self.speaker_id)
        rospy.loginfo("Language id: %s", self.language_id)
        rospy.loginfo("Style wav: %s", self.style_wav)

        # Create service
        rospy.Service('~tts', TextToSpeech, self.tts_request_callback)


    # Service callback
    def tts_request_callback(self, req):
        text = req.text
        text = codecs.decode(str(text).encode('utf-8'))
        rospy.loginfo("Text to be converted to speech: %s", text)

        if self.textToSoundFile(text):
            return TextToSpeechResponse(True)
        else:
            return TextToSpeechResponse(False)

    # Add stop character if requested
    def endText(self, text):
        if self.addStopChar and text[-1] not in self.VALID_END_OF_PHRASE:
            text = text + "."
        return text

    # Convert text to sound file
    def textToSoundFile(self, text):
        if len(text) == 0:
            rospy.logerr("No text has been specified.")
            return False

        try:
            req = requests.get(self.url + "/api/tts", params={'text': self.endText(text), 'speaker_id': self.speaker_id, 'language_id': self.language_id, 'style_wav': self.style_wav})

        except Exception as e:
            rospy.logerr("Error calling Coqui TTS server api")
            rospy.logerr(e)
            return False
        
        if req.status_code == 200 and req.headers['Content-Type'] == 'audio/wav':
            rospy.loginfo("Valid audio has been returned from Coqui TTS api.")

            with open(self.filename, 'wb') as f:
                f.write(req.content)
            return True

        else:
            rospy.logwarn("No audio has been returned from Coqui TTS server api")

        return False


if __name__ == "__main__":
    rospy.init_node('coqui_tts_test', anonymous=True)
    coqui_tts_node = CoquiTTSNode()
    rospy.spin()
