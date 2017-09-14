"""Example of a slave script.

The only thing you should need to change is the IP of the 'master' computer.
You shouldn't need to modify anything in the slave logic.
"""
from ev3control import run_slave

run_slave("localhost")

