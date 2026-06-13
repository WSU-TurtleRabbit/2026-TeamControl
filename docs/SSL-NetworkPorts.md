# NETWORK

This files covers the default ports for major SSL communications

For Detail Protobuf files please see [proto2](/src/TeamControl/SSL/proto2/)

[SSL Folder](/src/TeamControl/SSL/) includes the different SSL software unique classes and sockets


--- 
DEFAULT IP AND PORTS : 
game Controller broadcast 
group : 224.5.23.1
port : 10003

---
Vision in SSL - Vision 
type : multicast UDP
group : 224.5.23.2
port : 10006

---
Vision in GRSim (Default) 
type : multicast UDP
group : 224.5.23.2
port : 10020

---
Vision Tracker Multicast 
group : 224.5.23.2 
port : 10010

Auto-Ref
ip : local
port : 10007
trusted-keeys-dir : config/trusted_keys/auto_ref


Team 
ip: local
port : 10008
trusted-keys-dir: config/trusted_keys/team


remote control 
ip : local 
port : 10011
trusted-keys-dir: config/trusted_keys/remote-control


game controller Continuous Integration (CI)
ip : local
port : 10009

---

## Robot Ping Test (`robot_recv_test.py`)

**Location:** `src/TeamControl/utils/robot_recv_test.py`

A manual debugging tool for verifying that a real robot can receive UDP command packets from the PC.

**What it does:**
1. Binds a listener on the PC (`0.0.0.0:LISTEN_PORT`) to catch any robot reply.
2. Sends a zeroed-out `RobotCommand` (all velocities/kick/dribble = 0) to the configured robot IP and port.
3. Waits for a reply and prints it, or reports a timeout if none arrives.

**Configuration** (edit the constants at the top of the file):
```
ROBOT_IP    — IP of the target robot (from ipconfig.yaml, e.g. 192.168.0.73)
ROBOT_PORT  — robot's UDP listen port (default: 50514)
LISTEN_PORT — port on this PC to listen for replies (default: 50515)
```

**Notes:**
- Most robot firmware does not send a reply — a timeout is normal and does not mean the packet failed to arrive. Use `tcpdump` or a packet capture on the robot side to confirm receipt.
- The sender auto-detects the PC's LAN IP (`device_ip=None`), so no manual IP config is needed for sending.
- To test a different robot, change `ROBOT_IP` to the target address (e.g. `192.168.1.4` for Yellow D / Blue B).