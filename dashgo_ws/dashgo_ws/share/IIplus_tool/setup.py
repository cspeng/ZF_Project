"""
@author: Clarence/Trio
@compile: python setup.py build_ext -i
"""

from distutils.core import setup
from Cython.Build import cythonize

setup(name="pi_modbus", ext_modules=cythonize("pi_modbus.pyx"))
