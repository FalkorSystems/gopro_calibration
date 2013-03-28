#!/usr/bin/python
import platform

if (platform.system() == "Darwin"):
    from setuptools import setup

    APP = ['../defisheye']
    DATA_FILES = ['../config']
    OPTIONS = {'argv_emulation': True,
               'packages': 'PySide,yaml',
               'includes': 'cv,cv2',
               'argv_inject': '--gui',
               'iconfile': 'icon.icns',
               'plist': {
                   'CFBundleIdentifier': 'com.falkorsystems.defisheye',
                   'CFBundleName': 'DeFisheye'
               }
    }

    setup(
        name='DeFisheye',
        app=APP,
        data_files=DATA_FILES,
        options={'py2app': OPTIONS},
        setup_requires=['py2app'],
    )

else:
    from distutils.core import setup
    setup(name = "defisheye",
        version = "0.0.1",
        description = "Fisheye Distortion Removal Tool",
        author = "I Heart Engineering",
        author_email = "code@iheartengineering.com",
        url = "http://www.iheartengineering.com",
        license = "BSD-3-clause",
        scripts = ["defisheye"],
        data_files=[('/usr/share/applications', ["DeFisheye.desktop"]),
                    ('/usr/share/icons/hicolor/scalable/apps',
                        ["pixmaps/defisheye.svg"]),
                    ('/usr/share/icons/hicolor/128x128/apps',
                        ["pixmaps/128x128/defisheye.png"]),
                    ('/usr/share/icons/hicolor/64x64/apps',
                        ["pixmaps/64x64/defisheye.png"]),
                    ('/usr/share/icons/hicolor/32x32/apps',
                        ["pixmaps/32x32/defisheye.png"]),
                    ('/usr/share/icons/hicolor/16x16/apps',
                        ["pixmaps/16x16/defisheye.png"]),
                   ],
        long_description = """This tool remove distortion from videos shot with a fisheye lens.""" 
        #classifiers = []     
    ) 
