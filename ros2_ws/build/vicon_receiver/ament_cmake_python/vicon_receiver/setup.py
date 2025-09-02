from setuptools import find_packages
from setuptools import setup

setup(
    name='vicon_receiver',
    version='0.1.1',
    packages=find_packages(
        include=('vicon_receiver', 'vicon_receiver.*')),
)
