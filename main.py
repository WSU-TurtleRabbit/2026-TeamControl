#!/usr/bin/env python
"""
TurtleRabbit Team Control — 2026 Season
Main entry point. Starts up the core services and waits for you to stop it.

Right now this just boots the base infrastructure (world model, config, cache).
Vision, dispatcher, and robot processes will be wired in later.
"""

import argparse
import sys
import time
from multiprocessing import Event

from teamcontrol.config import Config
from teamcontrol.world import WorldModelManager
from teamcontrol.utils.logger import LogSaver


def main():
    parser = argparse.ArgumentParser(
        description="TurtleRabbit SSL Team Control — 2026",
    )
    parser.add_argument(
        "--config", type=str, default=None,
        help="path to config.yaml (uses defaults if not given)",
    )
    parser.add_argument(
        "--mode", choices=["test", "goalie", "1v1", "6v6"],
        default="test",
        help="operating mode (only 'test' works right now)",
    )
    args = parser.parse_args()

    # load config
    cfg = Config(config_path=args.config)
    print(f"[main] config loaded: {cfg}")

    # shared state
    is_running = Event()

    # world model (shared across processes)
    wm_manager = WorldModelManager()
    wm_manager.start()
    wm = wm_manager.WorldModel()

    print(f"[main] world model online: {wm}")
    print(f"[main] mode: {args.mode}")

    # ── start ──────────────────────────────────────────────────
    is_running.set()
    print("[main] system ready")
    print("[main] type 'exit' to shut down\n")

    # ── wait for exit ──────────────────────────────────────────
    while is_running.is_set():
        try:
            user_input = input()
            if user_input.strip().lower() == "exit":
                print("[main] shutting down...")
                is_running.clear()
                break
        except (KeyboardInterrupt, EOFError):
            print("\n[main] shutting down...")
            is_running.clear()

    print("[main] all done")


if __name__ == "__main__":
    main()
