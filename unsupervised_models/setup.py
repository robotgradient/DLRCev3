from setuptools import setup
from setuptools import find_packages

setup(
    name='unsupervised_models',
    packages=find_packages(),
    install_requires=[
        'tensorflow',
        "numpy"
    ],)
