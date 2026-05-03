import time
from TeamControl.harness.harness import Harness

h = Harness(robot_id=0, is_yellow=True)
path = h.start("smoke_test")
print(f"Logging to: {path}")

# Give vision ~200ms to deliver the first packet
time.sleep(0.2)

# Sanity check that vision is working
state0 = h.read_state()
print(f"Initial state: {state0}")
if state0 is None:
    print("WARNING: no vision state yet. Robot might not be on the field, or vision port is wrong.")

# Turn logging on, drive forward at 0.3 m/s for 2 seconds at 60 Hz
h.set_logging(True)
print("Driving forward for 2s...")
for _ in range(120):
    h.send(vx=0.3, vy=0.0, w=0.0)
    time.sleep(1/60)

# Stop the robot — send zero velocity for half a second
h.set_logging(False)
print("Braking...")
for _ in range(30):
    h.send(vx=0.0, vy=0.0, w=0.0)
    time.sleep(1/60)

# Final state
state1 = h.read_state()
print(f"Final state:   {state1}")

h.stop()
print(f"\nDone. Inspect the CSV at: {path}")
