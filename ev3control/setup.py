from setuptools import setup
from setuptools import find_packages

setup(
    name='ev3control',
    packages=find_packages(),
    install_requires=[
        'paho-mqtt>=1.3',
    ],)
