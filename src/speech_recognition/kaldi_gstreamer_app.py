#!/usr/bin/env python

# Make python 2/3 compatible
from __future__ import (absolute_import, division,
                        print_function, unicode_literals)
from builtins import *

import sys
import argparse
import os

# Gstreamer imports
import gi
gi.require_version('Gst', '1.0')
from gi.repository import GObject, Gst

# ROS imports
import rospy
from std_msgs.msg import String

# Import classes
from speech_recognition.gstreamer_app import GstApp
from speech_recognition.kaldi_ros_publisher import KaldiROSPub


class KaldiGstApp(GstApp, KaldiROSPub):
    """Kaldi Gstreamer Application"""
    def __init__(self, model_path):
        """Initialize a KaldiGstApp object"""
        GstApp.__init__(self)
        KaldiROSPub.__init__(self)

        self.type = 'Kaldi-Gst-App'
        self.asr = Gst.ElementFactory.make("onlinegmmdecodefaster", "asr")

        if self.asr:
            if not os.path.isdir(model_path):
                self.error("Model (%s) not downloaded. Run run-simulated.sh first" % model_path)

            self.asr.set_property("fst", model_path + "HCLG.fst")
            self.asr.set_property("lda-mat", model_path + "matrix")
            self.asr.set_property("model", model_path + "model")
            self.asr.set_property("word-syms", model_path + "words.txt")
            self.asr.set_property("silence-phones", "1:2:3:4:5")
            self.asr.set_property("max-active", 4000)
            self.asr.set_property("beam", 12.0)
            self.asr.set_property("acoustic-scale", 0.0769)
        else:
            print_msg = "Couldn't create the onlinegmmfasterdecoder element.\n"
            if "GST_PLUGIN_PATH" in os.environ:
              print_msg += "Kaldi Gstreamer Plugin probably not compiled."
            else:
              print_msg += "GST_PLUGIN_PATH unset.\nTry running: export GST_PLUGIN_PATH=$KALDI_ROOT/src/gst-plugin"
            self.error(print_msg)

        # Complete Gstreamer pipeline and start playing
        self.pipeline.add(self.asr)
        self.audioresample.link(self.asr)
        self.asr.link(self.fakesink)
        self.asr.connect('hyp-word', self._on_word_publish)
        self.pipeline.set_state(Gst.State.PLAYING)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="ROS Publisher for Gstreamer-Kaldi plugin.")
    parser.add_argument("model", type=str, help='Model path')
    args = parser.parse_args()

    # Initialize gstreamer library using threads
    GObject.threads_init()
    Gst.init(sys.argv)

    rospy.init_node('gstreamer_kaldi_stream', anonymous=True)
    app = KaldiGstApp(args.model)

    rospy.spin()
