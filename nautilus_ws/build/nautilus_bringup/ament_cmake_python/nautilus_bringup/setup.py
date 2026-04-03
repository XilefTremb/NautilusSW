from setuptools import find_packages
from setuptools import setup

setup(
    name='nautilus_bringup',
    version='0.0.0',
    packages=find_packages(
        include=('nautilus_bringup', 'nautilus_bringup.*')),
)
