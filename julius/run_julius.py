#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2017 Toyota Motor Corporation

"""This module starts Julius process."""
import rospy

_MODULEPORT = 10500


class RunJulius(object):
    """This class starts Julius process."""

    def __init__(self):
        """Constructor."""
        pass

    def create_run_julius_cmd(self):
        """Create julius command."""
        voice_ns_prefix = rospy.get_param('voice_ns_prefix')
        p_julius_config_file = rospy.get_param(
            voice_ns_prefix + '/julius_config_file')
        p_lm_grammar_mode = rospy.get_param(
            voice_ns_prefix + '/lm_grammar_mode')
        p_lm_locale = rospy.get_param(
            voice_ns_prefix + '/lm_locale')
        p_acoustic_model = rospy.get_param(
            voice_ns_prefix + '/acoustic_model', "")
        if p_acoustic_model == "":
            if p_lm_locale == "ja":
                p_acoustic_model = rospy.get_param(
                    voice_ns_prefix + '/acoustic_model_ja')
            else:
                p_acoustic_model = rospy.get_param(
                    voice_ns_prefix + '/acoustic_model_en')
        opt1 = " -C {0} {1}".format(p_julius_config_file, p_acoustic_model)

        p_dic_list = rospy.get_param(voice_ns_prefix + '/dic_list', "")

        if p_lm_grammar_mode:
            opt_dic_list = "-gramlist"
            if p_dic_list == "":
                if p_lm_locale == "ja":
                    p_dic_list = rospy.get_param(
                        voice_ns_prefix + '/dic_list_ja_grammar')
                else:
                    p_dic_list = rospy.get_param(
                        voice_ns_prefix + '/dic_list_en_grammar')
        else:
            opt_dic_list = "-wlist"
            if p_dic_list == "":
                if p_lm_locale == "ja":
                    p_dic_list = rospy.get_param(
                        voice_ns_prefix + '/dic_list_ja_word')
                else:
                    p_dic_list = rospy.get_param(
                        voice_ns_prefix + '/dic_list_en_word')
        opt2 = " {0} {1} ".format(opt_dic_list, p_dic_list)

        p_other_options = rospy.get_param(
            voice_ns_prefix + '/other_options', "")
        if p_other_options == "":
            if p_lm_locale == "ja":
                p_other_options = rospy.get_param(
                    voice_ns_prefix + '/other_options_ja')
            else:
                p_other_options = rospy.get_param(
                    voice_ns_prefix + '/other_options_en')
        opt3 = p_other_options

        p_moduleport = int(rospy.get_param(
            voice_ns_prefix + '/moduleport', _MODULEPORT))
        opt4 = " -input pulseaudio -module {0}".format(p_moduleport)

        p_run_julius_cmd = rospy.get_param(
            voice_ns_prefix + '/run_julius_cmd')
        final_cmd = p_run_julius_cmd + opt1 + opt2 + opt3 + opt4
        return final_cmd
