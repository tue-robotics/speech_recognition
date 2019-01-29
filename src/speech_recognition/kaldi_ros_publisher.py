#!/usr/bin/env python

# ROS imports
import rospy
from std_msgs.msg import String


class KaldiROSPub:
    """Kaldi ROS publisher"""
    def __init__(self):
        """Initialize Publisher"""
        self.pub_str = ""
        self.pub = rospy.Publisher('kaldi_spr', String, queue_size=10)

    def _on_word_publish(self, asr, word):
        # Publish only when a pause has been registered (might be less robust than single words when pauses are not
        # recognized due to, e.g., too much noise or talking in the background):
        if word == "<#s>":                              # Silence
            # rospy.loginfo(self.pub_str)               # For testing purposes
            self.pub.publish(self.pub_str)
            self.pub_str = ""
        elif self.pub_str == "":                        # No spaces at start of new sentence
            self.pub_str = self.pub_str + word
        else:
            self.pub_str = self.pub_str + " " + word

        # Publish single words:
        # if not word == "<#s>":
        #     # rospy.loginfo(word)
        #     self.pub.publish(word)
