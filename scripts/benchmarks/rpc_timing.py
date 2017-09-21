# execute with: cat dummy.py | python -m timeit -s "from ev3control.rpc import ev3; m = ev3.LargeMotor('outA')"
# m.position
# Try this too
# m.run_timed(speed_sp=100, time_sp=100)
# m.wait_until_not_moving()
# pass

import time

from ev3control.rpc import ev3

m = ev3.LargeMotor("outA")
iterations = 10000
total_time = 0
for _ in range(100):
    t0 = time.time()
    m.position + 3
    total_time += time.time() - t0

print("for {0} iterations, average time was {1} milliseconds".format(iterations, total_time/iterations * 1000))
