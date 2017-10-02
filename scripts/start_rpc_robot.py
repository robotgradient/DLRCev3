"""Convenience script that sets up a robot for testing in the python console.

Run like this:
```
python -i start_rpc_robot.py
```

After that you can manipulate the robot through the `r` variable.
"""
from ev3control.rpc import Robot

r = Robot(None)
