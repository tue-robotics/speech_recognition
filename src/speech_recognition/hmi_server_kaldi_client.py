#! /usr/bin/env python
#
# ROS Node and HMI Client of Kaldi Gstreamer App

import sys
import os

# Gstreamer imports
import gi
gi.require_version('Gst', '1.0')
from gi.repository import GObject, Gst
from speech_recognition.kaldi_gstreamer_app import KaldiGstApp

# ROS imports
import rospy
from hmi import AbstractHMIServer, HMIResult

class HMIServerKaldiClient(AbstractHMIServer):
    """Subclass of AbstractHMIServer that creates a HMI client for
    KaldiGstApp"""
    def __init__(self):
        """Class constructor that initializes the parent AbstractHMIServer class
        and gets parameters from ROS parameter server"""
        AbstractHMIServer.__init__(rospy.get_name())

        # Get the kaldi model path from ROS Parameter server
        self.model_path = rospy.get_param("/kaldi_model_path")

    def _determine_answer(self, description, grammar, target, is_preempt_requested):
        """Method override to start speech recognition upon receiving a query
        from the HMI server"""
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

