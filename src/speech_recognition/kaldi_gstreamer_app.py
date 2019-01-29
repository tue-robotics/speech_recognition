#!/usr/bin/env python
#

# Make python 2/3 compatible
from __future__ import (absolute_import, division,
                        print_function, unicode_literals)
from builtins import *

import sys
import argparse
import os
import gi
gi.require_version('Gst', '1.0')
from gi.repository import GObject, Gst

# ROS imports
# TODO: Move to different script
import rospy
from std_msgs.msg import String


class KaldiGstApp:
    """Kaldi Gstreamer Application"""
    # TODO: Make model directory as an input argument and move talker to
    # separate class
    def __init__(self, model_path):
        """Initialize a KaldiGstApp object"""
        self.init_gst(model_path)
        self.init_talker()

    def error(self, *args, **kwargs):
        """Print errors to stderr and exit program"""
        print("[Kaldi]", *args, file=sys.stderr, **kwargs)
        sys.exit(1)

    def init_gst(self, model_path):
        """Initialize the speech components"""
        self.pulsesrc = Gst.ElementFactory.make("pulsesrc", "pulsesrc")
        if self.pulsesrc == None:
            self.error("Error loading pulsesrc GST plugin. You probably need the gstreamer1.0-pulseaudio package")

        self.audioconvert = Gst.ElementFactory.make("audioconvert", "audioconvert")
        self.audioresample = Gst.ElementFactory.make("audioresample", "audioresample")
        self.asr = Gst.ElementFactory.make("onlinegmmdecodefaster", "asr")
        self.fakesink = Gst.ElementFactory.make("fakesink", "fakesink")

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

        # Generate Gstreamer pipeline (from source to sink)
        self.pipeline = Gst.Pipeline()
        for element in [self.pulsesrc, self.audioconvert, self.audioresample, self.asr, self.fakesink]:
            self.pipeline.add(element)
        self.pulsesrc.link(self.audioconvert)
        self.audioconvert.link(self.audioresample)
        self.audioresample.link(self.asr)
        self.asr.link(self.fakesink)
        self.asr.connect('hyp-word', self._on_word_publish)
        self.pipeline.set_state(Gst.State.PLAYING)

    def init_talker(self):
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


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="ROS Publisher for Gstreamer-Kaldi plugin.")
    parser.add_argument("model", type=str, help='Model path')
    arguments = parser.parse_args()
    model = arguments.model

    # Initialize gstreamer library using threads
    GObject.threads_init()
    Gst.init(sys.argv)

    rospy.init_node('gstreamer_kaldi_stream', anonymous=True)
    app = KaldiGstApp(model)

    rospy.spin()
