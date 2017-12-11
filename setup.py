#!/usr/bin/env python
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=[
        'rail_object_detector',
        # 'drfcn.bbox',
        # 'drfcn.core',
        # 'drfcn.mask',
        # 'drfcn.nms',
        # 'drfcn.operator_py',
        # 'drfcn.rpn',
        # 'drfcn.symbols',
        # 'drfcn.utils'
    ],
    package_dir={
        '': 'src',
        # 'drfcn': 'libs/drfcn',
    },
)
# TODO: Someday include the drfcn modules directly into the build process.
# Catkin is being stupid right now

setup(**setup_args)
