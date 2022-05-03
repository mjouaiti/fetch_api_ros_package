import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient


class SpeechControl(object):
    """ Robot Speech interface """

    def __init__(self): #rosrun sound_play soundplay_node.py needs to run on the robot!
        self.soundhandle = SoundClient()
        rospy.sleep(1)

    def say(self, sentence, voice = 'voice_rab_diphone', volume=1.0):
        #voice = 'voice_ked_diphone', 'voice_rab_diphone', 'voice_kal_diphone', 'voice_don_diphone'
        self.soundhandle.say(sentence, voice, volume)

if __name__ == '__main__':
    rospy.init_node("test_speech", anonymous=True)
    speech_module = SpeechControl()
    speech_module.say("Hello, I am Fetch, your butler for the day. How may I help you?")
