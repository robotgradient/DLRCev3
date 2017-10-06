from setuptools import setup
from setuptools import find_packages

setup(
    name='dlrc_one_shot_learning',
    install_requires=[
        "Keras==2.0.8",
        "tensorflow==1.3.0"
    ],
    packages=find_packages())
