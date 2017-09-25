from setuptools import setup
from setuptools import find_packages

setup(
    name='rick',
    install_requires=[
        "ev3control",
        "object_detection"
    ],
    packages=find_packages())
