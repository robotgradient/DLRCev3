from .controllers import act
from .robot import Robot
import pytest

robot = Robot({

    "leftMotor": "LargeMotor('outA')",
    "rightMotor": "LargeMotor('outB')",
    "gripper": "MediumMotor('outC')",

})



@pytest.mark.parametrize("coords,expected_direction", [
    ([200,300], "left"),
    ([500,300], "right"),
    ([316,300], "forward")
])
def test_act(coords, expected_direction):
    direction = act(robot, coords)

    assert direction == expected_direction
