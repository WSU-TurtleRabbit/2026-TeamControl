from TeamControl.network.robot_command import RobotCommand
from TeamControl.network.receiver import Receiver
from TeamControl.network.sender import Sender
from multiprocessing import Event

# --- Configure these ---
ROBOT_IP   = "192.168.0.73"  # robot's IP from ipconfig.yaml
ROBOT_PORT = 50514            # robot's listen port
LISTEN_PORT = 50515           # port on THIS PC to listen for robot replies (pick any free port)
# -----------------------

def robot_ping_test():
    is_running = Event()
    is_running.set()

    # Listener on this PC — robot should reply to this port if it sends anything back
    recv = Receiver(is_running, ip="0.0.0.0", port=LISTEN_PORT)
    print(f"[listener] Bound to 0.0.0.0:{LISTEN_PORT}, waiting for replies...")

    # Sender — fires a zeroed-out test command at the robot
    sender = Sender(device_ip=None)  # None = auto-detect this PC's LAN IP
    test_cmd = RobotCommand(robot_id=1, vx=1.0, vy=1.0, w=0, kick=0, dribble=0)
    payload = test_cmd.encode()
    sender.sock.sendto(payload, (ROBOT_IP, ROBOT_PORT))
    print(f"[sender]   Sent test packet to {ROBOT_IP}:{ROBOT_PORT} → {payload}")

    # Listen for a response (blocks for up to `timeout` seconds per call)
    # print(f"[listener] Waiting for response from robot...")
    # data, addr = recv.listen()
    # if data:
    #     print(f"[listener] Got reply from {addr}: {data}")
    # else:
    #     print(f"[listener] No reply received (timeout). Robot may not send responses, or the packet didn't arrive.")

robot_ping_test()