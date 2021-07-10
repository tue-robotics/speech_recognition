# System imports
import os

# Gstreamer imports
import gi
gi.require_version('Gst', '1.0')
from gi.repository import GObject, Gst

# ROS imports
import rospy

# TU/e Robotics
from hmi import AbstractHMIServer, HMIResult

# Speech recognition
from .kaldi_gstreamer_app import KaldiGstApp


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
        self.model_path = rospy.get_param("~kaldi_model_path")

        self._kaldi_app = None

        # Check Kaldi environment variables
        kaldi_root = os.environ.get("KALDI_ROOT", None)
        if kaldi_root is None:
            raise Exception("Environment variable KALDI_ROOT is not set")

        gst_plugin_path = os.environ.get("GST_PLUGIN_PATH", None)
        if gst_plugin_path is None:
            raise Exception("Environment variable GST_PLUGIN_PATH is not set")

        kaldi_gst_plugin_path = os.path.join(kaldi_root, "src/gst-plugin")
        if kaldi_gst_plugin_path not in gst_plugin_path:
            raise Exception("Kaldi gst plugin path {} not in gst plugin path {}".format(kaldi_gst_plugin_path,
                                                                                        gst_plugin_path))

    def _determine_answer(self, description, grammar, target, is_preempt_requested):
        """
        Method override to start speech recognition upon receiving a query
        from the HMI server

        :param description: (str) description of the HMI request
        :param grammar: (str) grammar that should be used
        :param target: (str) target that should be obtained from the grammar
        :param is_preempt_requested: (callable) checks whether a preempt is requested by the hmi client
        """
        self._kaldi_app = KaldiGstApp(self.model_path, grammar, target, is_preempt_requested)

        bus = self._kaldi_app.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message::eos", self._on_message)

        while not self._kaldi_app.sentence and not rospy.is_shutdown() and not is_preempt_requested():
            rospy.logdebug("No sentence received so far...")
            rospy.sleep(0.2)

        if self._kaldi_app.sentence:
            rospy.loginfo("Kaldi returned: {}".format(self._kaldi_app.sentence))
            rospy.logdebug("Sending stop event")
            self._kaldi_app.pipeline.send_event(Gst.Event.new_eos())
            return HMIResult(self._kaldi_app.sentence, self._kaldi_app.semantics)
        return None

    def _on_message(self, bus, message):
        """ Callback for gstreamer pipeline bus message. Sets the gstreamer pipeline state to NULL and sets the
        kaldi app to None

        :param bus:
        :param message:
        """
        rospy.logdebug("Stopping gstreamer pipeline")
        self._kaldi_app.pipeline.set_state(Gst.State.NULL)
        rospy.logdebug("Setting kaldi app to None")
        self._kaldi_app = None
        rospy.loginfo("Gstreamer pipeline stopped successfully")
