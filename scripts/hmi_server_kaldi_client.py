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
    """Subclass of AbstractHMIServer that creates a HMI client for
    KaldiGstApp"""
    def __init__(self):
        """Class constructor that initializes the parent AbstractHMIServer class
        and gets parameters from ROS parameter server"""
        super(HMIServerKaldiClient, self).__init__(rospy.get_name())

        # Get the kaldi model path from ROS Parameter server
        self.model_path = rospy.get_param("/kaldi_model_path")

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
            print("No sentence received so far...")
            rospy.sleep(1)

        if self._kaldi_app.sentence:
            # Todo: Find something that completely kills Gstreamer, below line only pauses
            result_str = self._kaldi_app.sentence
            print "Stopping pipeline"
            self._kaldi_app.pipeline.send_event(Gst.Event.new_eos())
            print "Pipeline stopped..."

            print(result_str)
            return HMIResult(result_str, "")
        return None

    def _on_message(self, bus, message):
        t = message.type
        if t == Gst.MessageType.EOS:
            print("Received EOS")
        with open("/tmp/on_message", "w") as f:
            f.write("Yeehah")
        rospy.loginfo("Received message")
        rospy.loginfo("Stopping gstreamer pipeline")
        self._kaldi_app.pipeline.set_state(Gst.State.NULL)
        self._kaldi_app = None
        rospy.loginfo("Gstreamer pipeline stopped successfully")


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

    GLib.MainLoop().run()

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

    # rospy.spin()
