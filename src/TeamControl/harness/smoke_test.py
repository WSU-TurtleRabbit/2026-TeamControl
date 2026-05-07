import time
from TeamControl.harness.harness import Harness
from TeamControl.harness.constants import (
    ROBOT_ID, IS_YELLOW, DELAY,
    STEPS, VX, VY, VW
)

h = Harness(robot_id=ROBOT_ID, is_yellow=IS_YELLOW)
path = h.start("smoke_test")
print(f"Logging to: {path}")

# Give vision time to deliver the first packet
time.sleep(DELAY)

# Sanity check that vision is working
position0 = h.read_position()
print(f"Initial position: {position0}")
if position0 is None:
    print("WARNING: no vision position yet. Robot might not be on the field, or vision port is wrong.")

# Turn logging on, drive forward at 0.3 m/s for 2 seconds at 60 Hz
h.set_logging(True)
print("Driving forward for 2s...")
for _ in range(STEPS):
    h.send(vx= VX, vy=VY, w= VW)
    time.sleep(1/60)

# Stop the robot — send zero velocity for half a second
h.set_logging(False)
print("Braking...")
for _ in range(30):
    h.send(vx=0.0, vy=0.0, w=0.0)
    time.sleep(1/60)

# Final position
position1 = h.read_position()
print(f"Final position:   {position1}")

h.stop()
print(f"\nDone. Inspect the CSV at: {path}")
