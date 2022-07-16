#language_level=3
# all .pyx files in a folder
from setuptools import setup
from Cython.Build import cythonize
import numpy

setup(
  name = 'layered_planner',
  ext_modules = cythonize(["*.pyx"], language="c++",force=True),
  include_dirs=[numpy.get_include()],
  extra_compile_args=["-O3"]
)