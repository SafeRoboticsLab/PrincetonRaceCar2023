from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['part1', 'part2', 'part3'],
    package_dir={'': 'scripts'},
)

setup(**d)