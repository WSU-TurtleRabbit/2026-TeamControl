"""Skill function package for the behaviour tree pipeline.

Each module in this package exposes a single skill function that takes a
Snapshot and robot-specific arguments and returns a MotionTarget.

Skill functions are pure: they read from the Snapshot and produce a
MotionTarget with no side effects.
"""
from __future__ import annotations
