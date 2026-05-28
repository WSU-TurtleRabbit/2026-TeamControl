#!/usr/bin/env python

import argparse
import sys
import time
from multiprocessing import Process, Queue, Event

from TeamControl.process_workers.vision_runner import VisionProcess
from TeamControl.process_workers.gcfsm_runner import GCfsm
from TeamControl.process_workers.wm_runner import WMWorker
from TeamControl.process_workers.robot_recv_runner import RobotRecv
from TeamControl.world.model_manager import WorldModelManager

from TeamControl.utils.Logger import LogSaver
from TeamControl.dispatcher.dispatch import Dispatcher
from TeamControl.utils.yaml_config import Config
from TeamControl.onboard_vision import build_ip_map

from TeamControl.robot.goalie import run_goalie
from TeamControl.robot.striker import run_striker
from TeamControl.robot.navigator import run_navigator, WAYPOINTS_A, WAYPOINTS_B
from TeamControl.robot.team import run_team
from TeamControl.robot.coop import run_coop


def main():
    parser = argparse.ArgumentParser(
        description="RoboCup SSL Team Control — multi-mode launcher",
    )
    parser.add_argument(
        "--mode",
        choices=["goalie", "1v1", "obstacle", "coop", "6v6"],
        default="goalie",
        help=(
            "goalie   — yellow goalie vs blue striker (default)\n"
            "1v1      — yellow striker vs blue striker\n"
            "obstacle — two robots chasing ball with obstacle avoidance\n"
            "coop     — two robots cooperate to score (pass + shoot)\n"
            "6v6      — full 6v6 match (1 goalie + 5 field per team)"
        ),
    )
    args = parser.parse_args()

    preset = Config()

    # ── Queues ────────────────────────────────────────────────
    vision_q = Queue()
    gc_q = Queue()
    dispatch_q = Queue()
    recv_q = Queue()
    ip_map = build_ip_map(preset)

    logger = None

    # ── Shared state ──────────────────────────────────────────
    is_running = Event()

    wm_manager = WorldModelManager()
    wm_manager.start()
    wm = wm_manager.WorldModel(us_yellow=preset.us_yellow, us_positive=preset.us_positive)
    
    # ── Background processes (always needed) ──────────────────
    background = [
        Process(target=VisionProcess.run_worker,
                args=(is_running, logger, vision_q,
                      preset.use_grSim_vision, preset.vision[1])),
        Process(target=GCfsm.run_worker,
                args=(is_running, logger, gc_q,
                      preset.us_yellow, preset.us_positive)),
        Process(target=WMWorker.run_worker,
                args=(is_running, logger, wm, vision_q, gc_q,
                      recv_q, ip_map)),
        Process(target=Dispatcher.run_worker,
                args=(is_running, logger, dispatch_q, preset)),
        Process(target=RobotRecv.run_worker,
                args=(is_running, logger, recv_q)),
    ]

    # ── Mode-specific foreground processes ────────────────────
    foreground = []

    if args.mode == "goalie":
        # Yellow goalie (robot 4) defends against blue striker (robot 0)
        foreground.append(
            Process(target=run_goalie,
                    args=(is_running, dispatch_q, wm,
                          3, preset.us_yellow)))
        foreground.append(
            Process(target=run_striker,
                    args=(is_running, dispatch_q, wm,
                          0, not preset.us_yellow)))

    elif args.mode == "1v1":
        # Yellow striker (robot 0) vs blue striker (robot 0)
        foreground.append(
            Process(target=run_striker,
                    args=(is_running, dispatch_q, wm,
                          0, True)))
        foreground.append(
            Process(target=run_striker,
                    args=(is_running, dispatch_q, wm,
                          0, False)))

    elif args.mode == "obstacle":
        # Two robots chasing ball with obstacle avoidance
        foreground.append(
            Process(target=run_navigator,
                    args=(is_running, dispatch_q, wm,
                          0, preset.us_yellow, WAYPOINTS_A)))
        foreground.append(
            Process(target=run_navigator,
                    args=(is_running, dispatch_q, wm,
                          1, preset.us_yellow, WAYPOINTS_B)))

    elif args.mode == "coop":
        # Cross-team coop: our bot (yellow) + opp bot (blue) cooperate
        # Both go left → right (attack_positive=True = score on +x goal)
        us_yellow = preset.us_yellow
        opp_yellow = not us_yellow
        foreground.append(
            Process(target=run_coop,
                    args=(is_running, dispatch_q, wm,
                          0, 0, us_yellow),
                    kwargs=dict(mate_is_yellow=opp_yellow,
                                attack_positive=True)))
        foreground.append(
            Process(target=run_coop,
                    args=(is_running, dispatch_q, wm,
                          0, 0, opp_yellow),
                    kwargs=dict(mate_is_yellow=us_yellow,
                                attack_positive=True)))

    elif args.mode == "6v6":
        # Full match: one coordinator per team, goalie = robot 0
        foreground.append(
            Process(target=run_team,
                    args=(is_running, dispatch_q, wm, True, 0)))
        foreground.append(
            Process(target=run_team,
                    args=(is_running, dispatch_q, wm, False, 0)))

    # ── Start everything ──────────────────────────────────────
    is_running.set()
    print(f"[main] Starting mode: {args.mode}")

    for p in background:
        p.start()
    for p in foreground:
        p.start()

    # ── Wait for exit ─────────────────────────────────────────
    while is_running.is_set():
        try:
            print("Type 'exit' to quit: ")
            user_input = input()
            if user_input.lower() == "exit":
                print("Shutdown signal received...")
                is_running.clear()
                break
        except KeyboardInterrupt:
            print("\nShutdown signal received...")
            is_running.clear()

        print("Waiting for processes to shut down...")
        time.sleep(1)

    # ── Join all processes ────────────────────────────────────
    for p in foreground:
        p.join(timeout=5)
    for p in background:
        p.join(timeout=5)

    print("All processes have been ended")


if __name__ == "__main__":
    main()
