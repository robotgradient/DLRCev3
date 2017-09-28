from setuptools import setup
from setuptools import find_packages

setup(
    name='unsupervised_models',
    packages=find_packages(),
    install_requires=[
        "Keras==2.0.8",
        "scipy==0.19.1",
        "h5py==2.7.1"
    ],)
