import rospy
from hmi import AbstractHMIServer, HMIResult

# New imports
from speech_recognition.kaldi_gstreamer_app import KaldiGstApp
import sys
import os
# Gstreamer imports
import gi
gi.require_version('Gst', '1.0')
from gi.repository import GObject, Gst


class HMIServerKaldiClient(AbstractHMIServer):

    def __init__(self):
        """
        DragonflyHMIServer that exposes the HMI ROS Server and holds a socket client that talks to the dragonfly
        speech recognition server.
        """
        super(HMIServerKaldiClient, self).__init__(rospy.get_name())
        self.model_path = rospy.get_param("/kaldi_model_path")     # Assuming this param has been set somewhere

    def _determine_answer(self, description, grammar, target, is_preempt_requested):
        # Todo: Take in grammar and target and compose the HCLG.fst (speech recognition graph) through a different
        #   class or whatever, before initialising the KaldiGstApp

        kaldi_app = KaldiGstApp(self.model_path)

        while not kaldi_app.sentence and not rospy.is_shutdown():
            print("No sentence received so far...")
            rospy.sleep(1)

        if kaldi_app.sentence:
            # Todo: Find something that completely kills Gstreamer, below line only pauses
            kaldi_app.pipeline.set_state(Gst.State.PAUSED)

            print(kaldi_app.sentence)
            return HMIResult(kaldi_app.sentence, "")
        return None
