"""
Multiprocessing manager for the WorldModel.

This lets multiple processes share a single WorldModel instance.
The manager creates a proxy object that forwards method calls
across process boundaries.

Usage:
    mgr = WorldModelManager()
    mgr.start()
    wm = mgr.WorldModel()
    # now pass wm to child processes — they all share the same model
"""

from multiprocessing.managers import BaseManager
from teamcontrol.world.model import WorldModel


class WorldModelManager(BaseManager):
    pass


WorldModelManager.register("WorldModel", WorldModel)
