from multiprocessing import Process, Queue, Event
from TeamControl.utils.Logger import LogSaver
from TeamControl.process_workers.worker import BaseWorker
from TeamControl.network.robot_command import RobotCommand
from TeamControl.network.sender import Sender
from TeamControl.network.ssl_sockets import grSimSender
from TeamControl.robot.constants import (
    ROBOT_CMD_INTERVAL, DISPATCH_LOOP_INTERVAL, CMD_STATS_INTERVAL,
)
import time
from pathlib import Path

import yaml
try:
    from yaml import CLoader as Loader
except ImportError as e:
    from yaml import Loader


class Dispatcher(BaseWorker):
    def __init__(self,is_running:Event ,logger:LogSaver, ):
        super().__init__(is_running,logger)
        self._last_sent_per_robot = {}
        self._send_counts = {}
        self.running_commands = {}
        self._info_q = None
        self._last_info_time = 0
        # Send-rate / command-age telemetry (reset each CMD_STATS_INTERVAL).
        self._stats_window_start = 0.0
        self._stats_sends = {}     # key -> real-robot sends this window
        self._stats_age_sum = {}   # key -> sum of (sent_at - command.time_set)
        
    
    def setup(self,*args):
        """
        takes dispatcher_q, preset_config, and optional info_q
        """
        if len(args) >= 3:
            q, config, self._info_q = args[0], args[1], args[2]
        else:
            q, config = args[0], args[1]
        self.q = q
        self.send_to_grSim = config.send_to_grSim
        self.yellow = config.yellow
        self.blue = config.blue
        self.r_sender = Sender(device_ip=config.robot_ip)
        if self.send_to_grSim is True:
            self.g_sender = grSimSender(*config.grSim_addr)
        else:
            self.g_sender = None

        # Build shell ID lookup caches for O(1) lookups
        self._yellow_shell_cache = {}
        if self.yellow:
            for robot_key, robot_dict in self.yellow.items():
                sid = robot_dict.get("shellID")
                if sid is not None:
                    self._yellow_shell_cache[sid] = robot_dict
        self._blue_shell_cache = {}
        if self.blue:
            for robot_key, robot_dict in self.blue.items():
                sid = robot_dict.get("shellID")
                if sid is not None:
                    self._blue_shell_cache[sid] = robot_dict

        # (shell_id, isYellow) — skip UDP/grSim for these while UI drives the robot
        self._manual_field_blocked = set()  # {(shell_id, isYellow), ...}
        self._manual_field_q = args[3] if len(args) >= 4 else None

        # self.g_sender = grSimSender()
        self.announce_initialisation()


    # Announce that the dispatcher has been created
    def announce_initialisation(self):
        print("Multi-robot dispatcher initialized!")
        print("Sender : ", self.r_sender)
        print("Sending to GrSim :", self.send_to_grSim)
    
    # Main processing loop
    def run(self):
        return super().run()
    
    def _drain_manual_field_q(self):
        if self._manual_field_q is None:
            return
        try:
            while True:
                op, sid, is_y = self._manual_field_q.get_nowait()
                key = (int(sid), bool(is_y))
                if op == "on":
                    self._manual_field_blocked.add(key)
                else:
                    self._manual_field_blocked.discard(key)
        except Exception:
            pass

    def step(self):
        self._drain_manual_field_q()
        self.check_new_commands()
        now = time.time()
        self.handle_commands(now)
        self.check_command_timeout(now)
        self._publish_info(now)
        self._log_rate_stats(now)
        # Cap the loop so it doesn't busy-spin a core (which floods grSim and
        # starves the control processes). Real-robot send rate is governed
        # separately by ROBOT_CMD_INTERVAL inside send_command.
        time.sleep(DISPATCH_LOOP_INTERVAL)


    def shutdown(self):
        print("reseting all robots to 0 ")
        self.reset_all_robots()
        self.handle_commands()
        print("Dispatcher has been shutdown")
        super().shutdown()
    # Get the next command from the queue and add it
    def check_new_commands(self):
        while not self.q.empty():
            queue_item = self.q.get_nowait()
            command, runtime = queue_item
            self.add(command, runtime)
        

    # Add a new command to the running commands and replace existing commands
    # for the robot with the same ID
    def add(self, command: RobotCommand, run_time: float):
        robot_id = command.robot_id
        isYellow = command.isYellow
        key = (robot_id, isYellow)
        self.running_commands[key] = {"isYellow": isYellow,"command": command, "runtime": run_time, "start_time": time.time()}
        self.logger.debug(f"[{robot_id=},{isYellow=}] New command added for {run_time}s , command: {command}")
        
    # Check if any commands have expired
    def check_command_timeout(self, now=None):
        if now is None:
            now = time.time()
        expired_commands = []

        for key, packet in self.running_commands.items():
            elapsed_time = now - packet["start_time"]
            if elapsed_time >= packet["runtime"]:
                self.logger.debug(f"[Robot {key}] Command expired after {elapsed_time:.2f}s")
                expired_commands.append(key)

        for key in expired_commands:
            self.reset_command(key)

    # Set a do nothing command for the specified robot
    def reset_command(self, key):
        isYellow = self.running_commands[key]["isYellow"]
        robot_id = self.running_commands[key]["command"].robot_id
        reset_command = RobotCommand(robot_id=robot_id, vx=0, vy=0, w=0, kick=0, dribble=0,isYellow=isYellow)
        self.logger.debug(f"[{isYellow=} {robot_id}] Reset to idle command")

        self.running_commands[key] = {"isYellow" : isYellow, "command": reset_command, "runtime": 9999999, "start_time": time.time()}

    def reset_all_robots(self):
        for key in list(self.running_commands.keys()):
            self.reset_command(key)
            
                
    # Handle all active commands for all robots
    def handle_commands(self, now=None):
        if now is None:
            now = time.time()
        for key, packet in self.running_commands.items():
            command = packet["command"]
            # if time.time() >= self.last_sent_time + 0.01:
            self.send_command(command, now)

    def send_command(self, command:RobotCommand, now=None):
        if now is None:
            now = time.time()
        shell_id = command.robot_id
        isYellow = command.isYellow
        if (shell_id, isYellow) in self._manual_field_blocked:
            return
        robot_dict = self.get_dict_from_shell(shell_id,isYellow)

        if self.send_to_grSim is True:
            self.g_sender.send_robot_command(command,override_id=robot_dict["grSimID"])

        key = (shell_id, isYellow)
        last = self._last_sent_per_robot.get(key, 0)
        if last + ROBOT_CMD_INTERVAL < now:
            self.r_sender.send(command,robot_dict["ip"],robot_dict["port"])
            self._last_sent_per_robot[key] = now
            self._send_counts[key] = self._send_counts.get(key, 0) + 1
            # Telemetry: how often we reach the robot, and how stale the
            # command is when it leaves (now - when the controller built it).
            self._stats_sends[key] = self._stats_sends.get(key, 0) + 1
            self._stats_age_sum[key] = (
                self._stats_age_sum.get(key, 0.0) + (now - command.time_set))

    
    def _log_rate_stats(self, now):
        """Periodically log the actual real-robot send rate (Hz) and mean
        command age (ms) per robot. Use this to (a) confirm the robot is
        receiving commands at ~ROBOT_CMD_HZ, and (b) spot commands going
        stale (age climbing = the controller stopped feeding fresh ones)."""
        if self._stats_window_start == 0.0:
            self._stats_window_start = now
            return
        elapsed = now - self._stats_window_start
        if elapsed < CMD_STATS_INTERVAL:
            return
        for key, count in self._stats_sends.items():
            if count <= 0:
                continue
            hz = count / elapsed
            mean_age_ms = (self._stats_age_sum.get(key, 0.0) / count) * 1000.0
            self.logger.info(
                f"[rate] robot {key}: {hz:.1f} Hz to real robot, "
                f"mean cmd age {mean_age_ms:.1f} ms")
        self._stats_window_start = now
        self._stats_sends.clear()
        self._stats_age_sum.clear()

    def _publish_info(self, now):
        if self._info_q is None or now - self._last_info_time < 0.25:
            return
        self._last_info_time = now
        try:
            cmds = {}
            for key, pkt in self.running_commands.items():
                cmd = pkt["command"]
                elapsed = now - pkt["start_time"]
                cmds[str(key)] = {
                    "robot_id": cmd.robot_id,
                    "isYellow": pkt["isYellow"],
                    "vx": round(cmd.vx, 3),
                    "vy": round(cmd.vy, 3),
                    "w": round(cmd.w, 3),
                    "kick": cmd.kick,
                    "dribble": cmd.dribble,
                    "runtime": round(pkt["runtime"], 2),
                    "elapsed": round(elapsed, 2),
                    "ip": self.get_dict_from_shell(cmd.robot_id, cmd.isYellow).get("ip", "?"),
                    "port": self.get_dict_from_shell(cmd.robot_id, cmd.isYellow).get("port", 0),
                    "sends": self._send_counts.get(key, 0),
                }
            info = {
                "commands": cmds,
                "send_to_grSim": self.send_to_grSim,
                "queue_size": self.q.qsize() if hasattr(self.q, "qsize") else -1,
                "yellow_shells": {sid: {"ip": d.get("ip"), "port": d.get("port"),
                                        "grSimID": d.get("grSimID")}
                                  for sid, d in self._yellow_shell_cache.items()},
                "blue_shells": {sid: {"ip": d.get("ip"), "port": d.get("port"),
                                      "grSimID": d.get("grSimID")}
                                for sid, d in self._blue_shell_cache.items()},
            }
            self._info_q.put_nowait(info)
        except Exception:
            pass

    def get_dict_from_shell(self,shell_id,isYellow) -> str:
        cache = self._yellow_shell_cache if isYellow is True else self._blue_shell_cache
        try:
            return cache[shell_id]
        except KeyError:
            raise ValueError(f"No robot with {shell_id=} found ")

    
# def run_dispatcher(is_running,q,use_sim,is_yellow):
#     d = dispatch(q=q,use_sim=use_sim,is_yellow=is_yellow)
#     d.process_q(is_running)
