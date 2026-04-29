from setuptools import find_packages
from setuptools import setup

setup(
    name='nautilus_controls',
    version='0.0.0',
    packages=find_packages(
        include=('nautilus_controls', 'nautilus_controls.*')),
)
