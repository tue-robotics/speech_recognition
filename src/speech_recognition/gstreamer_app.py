# Make python 2/3 compatible
from __future__ import (absolute_import, division,
                        print_function, unicode_literals)
from builtins import *

import sys
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst


class GstApp:
    """Base class for Gstreamer Kaldi application"""
    def __init__(self):
        """Initialize the speech components"""
        self.type = 'Gstreamer'
        self.pulsesrc = Gst.ElementFactory.make("pulsesrc", "pulsesrc")
        if self.pulsesrc is None:
            self._error("Error loading pulsesrc GST plugin. You probably need the gstreamer1.0-pulseaudio package")

        self.audioconvert = Gst.ElementFactory.make("audioconvert", "audioconvert")
        self.audioresample = Gst.ElementFactory.make("audioresample", "audioresample")
        self.fakesink = Gst.ElementFactory.make("fakesink", "fakesink")

        # Generate Gstreamer pipeline (from source to sink)
        self.pipeline = Gst.Pipeline()
        for element in [self.pulsesrc, self.audioconvert, self.audioresample, self.fakesink]:
            self.pipeline.add(element)
        self.pulsesrc.link(self.audioconvert)
        self.audioconvert.link(self.audioresample)

    def _error(self, *args, **kwargs):
        """Print errors to stderr and exit program"""
        print("[{}]".format(self.type), *args, file=sys.stderr, **kwargs)
        sys.exit(1)

