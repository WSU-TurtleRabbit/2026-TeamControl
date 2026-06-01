"""TurtleRabbitBT behaviour tree package — integrated into 2026 TeamControl.

Lightweight by design: importing this package does NOT pull in the adapter
or the multiprocess runner (both of which depend on the rest of the
TeamControl stack). Import them explicitly when needed:

    from TeamControl.bt.adapter import build_snapshot_from_world_model
    from TeamControl.bt.run_bt_v2_process import run_bt_v2_process

See ``docs/bt_v2_integration.md``.
"""
