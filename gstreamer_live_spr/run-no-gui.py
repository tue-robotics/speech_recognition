#!/usr/bin/env python
#
# Copyright (c) 2013 Tanel Alumae
#
# Slightly inspired by the CMU Sphinx's Pocketsphinx Gstreamer plugin demo (which has BSD license)
#
# Apache 2.0

import sys
import os
import gi
import rospy
from std_msgs.msg import String
gi.require_version('Gst', '1.0')
from gi.repository import GObject, Gst


class DemoApp(object):
    """GStreamer/Kaldi Demo Application"""
    def __init__(self):
        """Initialize a DemoApp object"""
        self.init_gst()
        self.init_talker()

    def init_gst(self):
        """Initialize the speech components"""
        self.pulsesrc = Gst.ElementFactory.make("pulsesrc", "pulsesrc")
        if self.pulsesrc == None:
            print >> sys.stderr, "Error loading pulsesrc GST plugin. You probably need the gstreamer1.0-pulseaudio package"
            sys.exit(1)
        self.audioconvert = Gst.ElementFactory.make("audioconvert", "audioconvert")
        self.audioresample = Gst.ElementFactory.make("audioresample", "audioresample")    
        self.asr = Gst.ElementFactory.make("onlinegmmdecodefaster", "asr")
        self.fakesink = Gst.ElementFactory.make("fakesink", "fakesink")
        
        if self.asr:
            model_dir = "online-data/models/tri2b_mmi/"  # Current test directory
            if not os.path.isdir(model_dir):
                print >> sys.stderr, "Model (%s) not downloaded. Run run-simulated.sh first" % model_dir
                sys.exit(1)
            self.asr.set_property("fst", model_dir + "HCLG.fst")
            self.asr.set_property("lda-mat", model_dir + "matrix")
            self.asr.set_property("model", model_dir + "model")
            self.asr.set_property("word-syms", model_dir + "words.txt")
            self.asr.set_property("silence-phones", "1:2:3:4:5")
            self.asr.set_property("max-active", 4000)
            self.asr.set_property("beam", 12.0)
            self.asr.set_property("acoustic-scale", 0.0769)
        else:
            print >> sys.stderr, "Couldn't create the onlinegmmfasterdecoder element. "
            if "GST_PLUGIN_PATH" in os.environ:
              print >> sys.stderr, "Have you compiled the Kaldi GStreamer plugin?"
            else:
              print >> sys.stderr, "You probably need to set the GST_PLUGIN_PATH environment variable"
              print >> sys.stderr, "Try running: export GST_PLUGIN_PATH=$KALDI_ROOT/src/gst-plugin"
            sys.exit(1)

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
    # Initialize gstreamer library using threads
    GObject.threads_init()
    Gst.init(sys.argv)

    app = DemoApp()
    rospy.init_node('gstreamer_kaldi_stream', anonymous=True)

    print '''
    The (bigram) language model used to build the decoding graph was
    estimated on an audio book's text. The text in question is
    King Solomon's Mines" (http://www.gutenberg.org/ebooks/2166).
    You may want to read some sentences from this book first ...'''

    rospy.spin()
