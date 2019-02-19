#! /usr/bin/env python
#
# ROS Node and HMI Client of Kaldi Gstreamer App

# System imports
import os
import sys

# Gstreamer imports
import gi
gi.require_version('Gst', '1.0')
from gi.repository import GLib, GObject, Gst
from speech_recognition.kaldi_gstreamer_app import KaldiGstApp

# ROS imports
import rospy
from hmi import AbstractHMIServer, HMIResult


class HMIServerKaldiClient(AbstractHMIServer):
    """
    Subclass of AbstractHMIServer that creates a HMI client for
    KaldiGstApp
    """
    def __init__(self):
        """
        Class constructor that initializes the parent AbstractHMIServer class
        and gets parameters from ROS parameter server
        """
        super(HMIServerKaldiClient, self).__init__(rospy.get_name())

        # Get the kaldi model path from ROS Parameter server
        self.model_path = rospy.get_param("/kaldi_model_path")

        self._kaldi_app = None

    def _determine_answer(self, description, grammar, target, is_preempt_requested):
        """Method override to start speech recognition upon receiving a query
        from the HMI server"""
        # Todo: Take in grammar and target and compose the HCLG.fst (speech recognition graph) through a different
        #   class or whatever, before initialising the KaldiGstApp

        self._kaldi_app = KaldiGstApp(self.model_path)
        bus = self._kaldi_app.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message::eos", self._on_message)

        while not self._kaldi_app.sentence and not rospy.is_shutdown():
            rospy.logdebug("No sentence received so far...")
            rospy.sleep(0.2)

        if self._kaldi_app.sentence:
            rospy.loginfo("Kaldi returned: {}".format(self._kaldi_app.sentence))
            result_str = self._kaldi_app.sentence
            rospy.logdebug("Sending stop event")
            self._kaldi_app.pipeline.send_event(Gst.Event.new_eos())
            return HMIResult(result_str, "")
        return None

    def _on_message(self, bus, message):
        rospy.logdebug("Stopping gstreamer pipeline")
        self._kaldi_app.pipeline.set_state(Gst.State.NULL)
        rospy.logdebug("Setting kaldi app to None")
        self._kaldi_app = None
        rospy.logdebug("Gstreamer pipeline stopped successfully")


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

    # Start the rosnode and create the HMI Server/Kaldi Client
    rospy.init_node('hmi_server_kaldi_client')
    rospy.loginfo("Creating HMIServerKaldiClient")
    c = HMIServerKaldiClient()

    # Run the GLib main loop. This makes sure the messages on the bus are handled, which is required for neatly stopping
    # gstreamer
    rospy.loginfo("Starting GLib main loop")
    GLib.MainLoop().run()
