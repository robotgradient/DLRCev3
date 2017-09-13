from setuptools import setup

setup(
    name='ev3control',
    py_modules=['ev3control'],
    install_requires=[
        'paho-mqtt>=1.3',
    ],
)
