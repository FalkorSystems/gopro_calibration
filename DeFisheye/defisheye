#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, I Heart Engineering
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of I Heart Engineering nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import sys
import os
import yaml
import platform
import argparse
import subprocess
from PySide import QtGui

class gui(QtGui.QWidget):
    def __init__(self):
        super(gui, self).__init__()
        self.setGeometry(300, 300, 600, 150)
        self.setWindowTitle('DeFishEye')   

        self.layout = QtGui.QGridLayout()

        self.camera_label = QtGui.QLabel("Camera: ")

        self.camera_combo = QtGui.QComboBox()
        self.camera_combo.activated.connect(self.combo_activated)

        self.layout.addWidget(self.camera_label, 0, 0)
        self.layout.addWidget(self.camera_combo, 0, 1, 1, 2)

        self.active_calibration = None
        self.calibration = []

        self.input_label = QtGui.QLabel("Input Video: ")
        self.input_file = QtGui.QLineEdit()
        self.input_file.setMinimumWidth(240)
        self.input_file.setMaximumWidth(1024)
        self.input_button = QtGui.QPushButton("...")
        self.input_button.setMinimumWidth(24)
        self.input_button.setMaximumWidth(24)
        self.input_button.clicked.connect(self._input_dialog)

        self.layout.addWidget(self.input_label, 2, 0);
        self.layout.addWidget(self.input_file, 2, 1);
        self.layout.addWidget(self.input_button, 2, 2);

        self.output_label = QtGui.QLabel("Output Video: ")
        self.output_file = QtGui.QLineEdit()
        self.output_file.setMinimumWidth(240)
        self.output_file.setMaximumWidth(1024)
        self.output_button = QtGui.QPushButton("...")
        self.output_button.setMinimumWidth(24)
        self.output_button.setMaximumWidth(24)
        self.output_button.clicked.connect(self._output_dialog)

        self.layout.addWidget(self.output_label, 3, 0);
        self.layout.addWidget(self.output_file, 3, 1,);
        self.layout.addWidget(self.output_button, 3, 2);

        self.vbox = QtGui.QVBoxLayout()
        self.vbox.addLayout(self.layout)
        self.vbox.addSpacing(12)
        self.vbox.addStretch(1)

        self.buttonBox = QtGui.QDialogButtonBox(QtGui.QDialogButtonBox.Ok
                                            | QtGui.QDialogButtonBox.Cancel);
        self.buttonBox.rejected.connect(self.close)
        self.vbox.addWidget(self.buttonBox)

        self.setLayout(self.vbox)

    def combo_activated(self, value):
        self.active_calibration = self.calibration[value]
        print self.active_calibration

    def tag_constructor(self, loader, tag_suffix, node):
        return tag_suffix + ' ' + repr(node.value)

    def load_configs(self,dir):
        try:
            files = os.listdir(dir)
        except Exception, e:
            print e, ": Invalid directory"
            sys.exit(os.EX_OSFILE)

        for filename in files[:]:
            filepath = "%s/%s"%(dir,filename)
            print "Filepath: %s"%filepath
            yaml.add_multi_constructor('', self.tag_constructor)
            try:
                cfg = yaml.load(file(filepath, 'r'))
            except yaml.YAMLError, e:
                print "ERROR: configuration file:", e
            self.camera_combo.addItem(u'%s    [ %s ]'%(cfg['camera_name'],cfg['camera_mode']))
            self.calibration.append(filepath)

    def _input_dialog(self):
        filename, filter = QtGui.QFileDialog.getOpenFileName(parent=self, caption='Input Video File', dir=os.getenv("HOME"), filter="Video (*.mp4)")
        if os.path.exists(filename):
            self.input_file.setText(filename)

    def _output_dialog(self):
        filename, filter = QtGui.QFileDialog.getSaveFileName(parent=self, caption='Output Video File', dir=os.getenv("HOME"), filter="Video (*.mp4 *.ogg)")
        #if os.path.exists(filename):
        self.output_file.setText(filename)

def defisheye_usage():
    print """DeFisheye - Video Rectification Tool
------------------------------------------------------------
usage: defisheye [--version] COMMAND [ARGS]"""
    sys.exit(os.EX_USAGE)


################################
if __name__ == '__main__':
    script_dir = os.path.abspath(os.path.dirname(sys.argv[0]))
    os_name = platform.system()
    if (os_name != "Darwin" and os_name != "Linux"):
        print "Error: Unsupported System"
        sys.exit(os.EX_SOFTWARE)
    if (os_name == "Darwin"):
        config_dir = os.path.expanduser("%s/config"%(script_dir))
    if (os_name == "Linux"):
        config_dir = os.path.expanduser("%s/config"%(script_dir))

    parser = argparse.ArgumentParser(prog='defisheye',epilog='Video Correction Tool')
    parser.add_argument('-c','--config', help='Calibration File', nargs=1)
    parser.add_argument('--gui', help='QT GUI Mode', action='store_true')
    parser.add_argument('input', help='Input Video', nargs='?')
    parser.add_argument('output', help='Output Video', nargs='?')
    args = parser.parse_args()

    print "DeFisheye"
    print "---------"
    print "GUI:    %s"%repr(args.gui)
    print "Input:  %s"%repr(args.input)
    print "Output: %s"%repr(args.output)
    print "Config Dir: %s"%(config_dir)

    if (args.gui == True or sys.stdout.isatty() == False):
        app = QtGui.QApplication(sys.argv)
        win = gui()
        if (args.input == None):
            filename, filter = QtGui.QFileDialog.getOpenFileName(parent=win, caption='Input Video File', dir=os.getenv("HOME"), filter="Video (*.mp4)")
            if os.path.exists(filename):
                win.input_file.setText(filename)
            print filename
        else:
            win.input_file.setText(args.input)
        win.load_configs(config_dir)
        win.combo_activated(0)

        win.show()
        sys.exit(app.exec_())
    else:
        print "Run stuff here"


    #p = subprocess.Popen("DISPLAY=:0 /usr/bin/notify-send --icon='defisheye' 'Input File' '%s/%s'"%(repr(args.gui),args.input), stdout=subprocess.PIPE, shell=True)

