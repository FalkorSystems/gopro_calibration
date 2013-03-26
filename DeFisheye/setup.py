from distutils.core import setup
import os
import subprocess

setup(name = "defisheye",
    version = "0.0.1",
    description = "Fisheye Distortion Removal Tool",
    author = "I Heart Engineering",
    author_email = "code@iheartengineering.com",
    url = "http://www.iheartengineering.com",
    license = "BSD-3-clause",
    scripts = ["defisheye"],
    data_files=[('/usr/share/applications', ["DeFisheye.desktop"]),
                ('/usr/share/icons/hicolor/scalable/apps',["pixmaps/defisheye.svg"]),
               ],
    long_description = """This tool remove distortion from videos shot with a fisheye lens.""" 
    #classifiers = []     
) 
