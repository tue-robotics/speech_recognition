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

if __name__ == "__main__":
    # TODO Move to KaldiGstApp class definition
    try:
        kaldi_root = os.environ['KALDI_ROOT']
    except:
        sys.exit("Environment variable KALDI_ROOT unset")

    try:
        gst_plugin_path = os.environ['GST_PLUGIN_PATH']
    except:
        sys.exit("GST_PLUGIN_PATH is unset. Kaldi gst-plugin not in path")
    else:
        kaldi_gst_plugin_path = os.path.join(kaldi_root, "src/gst-plugin")

        if not kaldi_gst_plugin_path in gst_plugin_path:
            sys.exit("Kaldi gst-plugin is not in GST_PLUGIN_PATH")

    # Initialize gstreamer library using threads
    GObject.threads_init()
    Gst.init(sys.argv)


    rospy.init_node('hmi_server_kaldi_client')
    c = HMIServerKaldiClient()

    # # Generate some required parameters.
    # description = ""
    # grammar = ""
    # target = ""
    # is_preempt_requested = False
    #
    # # Run the _determine_answer function once to recognize a command (single sentence/request)
    # c._determine_answer(description, grammar, target, is_preempt_requested)
    #
    # print("Finished.")

    rospy.spin()

