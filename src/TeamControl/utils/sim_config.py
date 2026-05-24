"""Loader for ``sim_6v6.yaml`` — 6v6 BT-vs-BT simulation config.

Kept separate from ``yaml_config.Config`` (which loads ``ipconfig.yaml``)
so the two concerns don't bleed into each other: network/team settings vs
sim scenario settings.
"""
from __future__ import annotations

from pathlib import Path

import yaml

try:
    from yaml import CLoader as Loader
except ImportError:
    from yaml import Loader

from TeamControl.bt.contracts.blackboard import RoleType


_ROLE_LOOKUP = {r.name.lower(): r for r in RoleType}


def _parse_roles(raw: dict) -> dict[int, RoleType]:
    out: dict[int, RoleType] = {}
    for key, value in raw.items():
        try:
            role = _ROLE_LOOKUP[str(value).lower()]
        except KeyError as e:
            raise ValueError(
                f"Unknown role {value!r} for robot {key} in sim_6v6.yaml "
                f"(valid: {list(_ROLE_LOOKUP)})"
            ) from e
        out[int(key)] = role
    return out


class Sim6v6Config:
    """In-memory view of ``sim_6v6.yaml``."""

    def __init__(self, config_filename: str = "sim_6v6.yaml") -> None:
        path = Path(__file__).resolve().parent / config_filename
        with open(path, "r") as f:
            raw = yaml.load(f, Loader)

        self.yellow_ids: list[int] = [int(x) for x in raw["yellow"]["robot_ids"]]
        self.blue_ids: list[int] = [int(x) for x in raw["blue"]["robot_ids"]]
        self.roles: dict[int, RoleType] = _parse_roles(raw["roles"])
        self.tick_period: float = float(raw.get("tick_period", 0.01))

    def __repr__(self) -> str:  # pragma: no cover - debugging only
        return (
            f"Sim6v6Config(yellow_ids={self.yellow_ids}, "
            f"blue_ids={self.blue_ids}, "
            f"roles={ {k: v.name for k, v in self.roles.items()} }, "
            f"tick_period={self.tick_period})"
        )


if __name__ == "__main__":
    print(Sim6v6Config())
