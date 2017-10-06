from setuptools import setup
from setuptools import find_packages

setup(
    name='duckdata',
    packages=find_packages(),
    install_requires=[
    "scipy==0.19.1",
    "Pillow==4.2.1",
    "scikit-image==0.13.0",
    "toolz==0.8.2"
    ]
    )
