# Real Robot Networking Guide

This document covers everything learned from getting the codebase to communicate
with physical SSL robots on a local network. It distinguishes confirmed working
approaches from known issues still under investigation.

---

## Network Topology

The PC running the code and the physical robots must be on the same LAN subnet. Connect to RAM 5G and test by pinging the robot interface before testing code.

| Robot | IP | Port |
|-------|----|------|
| Yellow A, B | `192.168.0.x` (this is your IP) | `50514` |

Robots on different subnets (`192.168.0.x` vs `192.168.1.x`) require the PC to
have a network adapter on each subnet, or proper routing between them. Verify
reachability with `ping <robot-ip>` before running any code.

---

## Packet Format (Confirmed Working)

The dispatcher sends commands to real robots as **plain UTF-8 text** over UDP.
The format produced by `RobotCommand.__str__()` is:

```
robot_id vx vy w kick dribble timestamp
```

Example: `0 0.47 0.0 0.12 0 0 1718000000.123`

This is **not** the same as the grSim protobuf format. grSim uses
`grSim_Packet.SerializeToString()`. Confirmed: real robots accept the plain-text
format. The grSim protobuf path must **not** be used for real robot communication.

---

## ipconfig.yaml — Required Settings for Real Robot Runs

```yaml
send_to_grSim: false       # set true ONLY to also mirror commands into grSim
use_grSim_vision: true     # keep true — grSim provides vision frames

network:
  robot_ip: 192.168.0.72   # PC's LAN IP on the robot subnet (run ipconfig to find it)
  vision_ip: 192.168.0.72  # same IP — pins multicast subscription to the right adapter
```

**Why `robot_ip` and `vision_ip` must be the PC's real LAN IP:**
- `robot_ip` is passed to the `Sender` socket. Although the sender is unbound
  (so this does not directly affect outbound routing), leaving it as `0.0.0.0`
  causes `_obtain_sys_ip()` to auto-detect, which may return an incorrect adapter
  on multi-adapter machines.
- `vision_ip` is used in `IP_ADD_MEMBERSHIP` to specify which network adapter
  joins the multicast group `224.5.23.2`. Leaving it as `0.0.0.0` allows the OS
  to pick any interface, which non-deterministically selects the wrong one (e.g.
  WiFi instead of Ethernet). **Setting it to the specific Ethernet adapter IP
  improved reliability but has not fully eliminated intermittent drops.**

---

## Known Issues

### Vision drops intermittently (under investigation)
Even with `vision_ip` set to the PC's LAN IP, vision frames are occasionally
not received. When this happens the world model receives no frames, `get_latest_frame()`
returns `None`, and no robot commands are generated — the robot does not move.

Symptom: `[main] Waiting for first vision frame...` hangs or times out.

The vision barrier added in `main.py` and `sandbox.py` confirms when this
occurs (prints an error instead of silently failing). A full fix has not been
confirmed yet. Suspected causes:
- grSim multicast datagrams lost on the Windows network stack intermittently
- Windows Firewall blocking inbound UDP on port `10006` inconsistently

### grSim "Sending UDP datagram failed" messages
These appear in grSim's console and are **not** vision frames failing. They are
grSim attempting to send command acknowledgment/status packets back to the
ephemeral port of the PC's sender socket, which is not listening for replies.
These are harmless and do not affect robot control.

---

## Confirmed Working

- **Ping from PC to robot** — basic network reachability confirmed on `192.168.0.73`.
- **`robot_recv_test.py`** — manually sending a `RobotCommand` UDP packet to the
  robot causes the robot to execute the command. Plain-text format confirmed.
- **`main.py --mode 1v1`** — full pipeline (grSim vision → WMWorker → striker →
  dispatcher → real robot) confirmed working at least once with consistent runs
  after the vision barrier was added.
- **Multicast fix (`receiver.py`)** — `WinError 10049` crash on startup fully
  resolved by falling back to `0.0.0.0` as the multicast interface when the
  configured IP is rejected by Windows.
- **Vision readiness barrier** — starting foreground robot processes only after
  the first vision frame is confirmed made runs noticeably more consistent.

---

## Testing Tools

### `robot_recv_test.py`
Located at `src/TeamControl/utils/robot_recv_test.py`.

Sends a single test `RobotCommand` to a configured robot IP and listens for any
reply. Configure at the top of the file:

```python
ROBOT_IP    = "192.168.0.73"  # target robot
ROBOT_PORT  = 50514
LISTEN_PORT = 50515           # port on this PC for any robot reply
```

Note: most robot firmware does not send a reply. A timeout is normal. Confirm
receipt by observing the robot physically reacting to the command.

### One-liner UDP test (PowerShell)
Send a raw UDP packet to the robot without running any TeamControl code:

```powershell
python -c "import socket; s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.sendto(b'0 0.15 0.0 0.0 0 0 0.0', ('192.168.0.73 (robot IP)', 50514))"
```

### Vision reception test
Confirm grSim is broadcasting vision before running main code:

```python
import socket, struct
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('', 10006))
mreq = struct.pack("=4s4s", socket.inet_aton("224.5.23.2"), socket.inet_aton("0.0.0.0"))
s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
print("Listening on 224.5.23.2:10006 ...")
while True:
    data, addr = s.recvfrom(8192)
    print(f"Got {len(data)} bytes from {addr}")
```

If nothing arrives, grSim is not broadcasting or is using a different port/group.
Check grSim's vision settings panel.

---

## Launch Flags

```bash
# Standard run — requires Game Controller on the network
python main.py --mode 1v1

# Skip GC process — useful for robot testing without a live Game Controller
python main.py --mode 1v1 --skip-gc
```

The `--skip-gc` flag skips the `GCfsm` process. The BT/striker defaults to
`GamePhase.RUNNING` when no GC state is present, so robots move without needing
a Game Controller signal.
