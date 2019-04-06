#! /usr/bin/env python
#
# Kaldi Gstreamer App

# Make python 2/3 compatible
from __future__ import (absolute_import, division,
                        print_function, unicode_literals)

# System imports
import os

# Gstreamer imports
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

# Speech recognition
from .gstreamer_app import GstApp


class KaldiGstApp(GstApp):
    """Kaldi Gstreamer Application"""
    def __init__(self, model_path, grammar):
        """Initialize a KaldiGstApp object"""
        GstApp.__init__(self)

        self.type = 'Kaldi-Gst-App'
        self.pub_str = ""
        self.sentence = None
        self.asr = Gst.ElementFactory.make("onlinegmmdecodefaster", "asr")
        self.grammar = grammar

        if self.asr:
            if not os.path.isdir(model_path):
                self._error("Model (%s) not downloaded. Place the model at (%s) first" % model_path)

            self.asr.set_property("fst", model_path + "HCLG.fst")
            # Add LDA matrix if it exists
            if os.path.exists(model_path + "final.mat"):
                self.asr.set_property("lda-mat", model_path + "final.mat")

            self.asr.set_property("model", model_path + "final.mdl")
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
            self._error(print_msg)

        # Complete Gstreamer pipeline and start playing
        self.pipeline.add(self.asr)
        self.audioresample.link(self.asr)
        self.asr.link(self.fakesink)
        self.asr.connect('hyp-word', self._wait_for_sentence)
        self.pipeline.set_state(Gst.State.PLAYING)

    def _wait_for_sentence(self, asr, word):
        # TODO: If the 'word' input changes from single words to a sentence (i.e. if the c++ file of the Gstreamer-kaldi
        #   -plugin is edited to push out sentences instead of words) then this function should change
        # Publish only when a pause has been registered (might be less robust than single words when pauses are not
        # recognized due to, e.g., too much noise or talking in the background):
        if word == "<#s>":                              # Silence
            self.sentence = self.pub_str
            self.pub_str = ""
        elif self.pub_str == "":                        # No spaces at start of new sentence
            self.pub_str += word
        else:
            self.pub_str += " " + word
