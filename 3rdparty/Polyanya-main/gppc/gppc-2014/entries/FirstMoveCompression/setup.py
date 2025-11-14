"""
Setup script for pybind11 wrapper of C++ first move matrix extraction
"""

from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup, Extension
import pybind11
import os

# Get the directory where this setup.py is located
this_dir = os.path.dirname(os.path.abspath(__file__))

# Get all .cpp files except main.cpp, extract_first_move_matrix.cpp, and the wrapper itself
cpp_files = []
excluded_files = ['main.cpp', 'extract_first_move_matrix.cpp', 'extract_first_move_matrix_wrapper.cpp']
for f in os.listdir(this_dir):
    if f.endswith('.cpp') and f not in excluded_files:
        cpp_files.append(f)

# Define the extension module
ext_modules = [
    Pybind11Extension(
        "cpp_first_move_matrix",
        ["extract_first_move_matrix_wrapper.cpp"] + cpp_files,
        include_dirs=[
            this_dir,
            pybind11.get_include(),
        ],
        language='c++',
        cxx_std=11,
        extra_compile_args=['-O3', '-DNDEBUG'],
    ),
]

setup(
    name="cpp_first_move_matrix",
    version="0.1.0",
    author="",
    description="C++ First Move Matrix Extraction Python Wrapper",
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
    python_requires=">=3.6",
)
