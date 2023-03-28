#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
     packages=['perception_ekf',
               'perception_ekf.ekf_utils',
               'perception_ekf.dynamic_models',
               'perception_ekf.measurement_models'],
     package_dir={'': 'scripts'}
)

setup(**setup_args)