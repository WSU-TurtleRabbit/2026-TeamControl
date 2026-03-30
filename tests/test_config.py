"""Tests for config loading."""

from teamcontrol.config import Config


def test_defaults_load():
    cfg = Config()
    assert cfg.us_yellow is True
    assert cfg.robots_active == 6
    assert cfg.grsim_addr == ("127.0.0.1", 20011)


def test_vision_addr():
    cfg = Config()
    assert cfg.vision_addr == ("224.5.23.2", 10006)


def test_cache_defaults():
    cfg = Config()
    assert cfg.cache_max_frames == 120
    assert cfg.cache_ttl == 5.0
