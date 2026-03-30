"""Basic tests for the caching system."""

import time
import pytest
from teamcontrol.core.cache import Cache


def test_put_and_get():
    c = Cache(ttl=5.0)
    c.put("foo", 42)
    assert c.get("foo") == 42


def test_get_missing_key():
    c = Cache(ttl=5.0)
    assert c.get("nope") is None
    assert c.get("nope", default="fallback") == "fallback"


def test_expiry():
    c = Cache(ttl=0.1)  # 100ms TTL
    c.put("short_lived", "hello")
    assert c.get("short_lived") == "hello"

    time.sleep(0.15)
    assert c.get("short_lived") is None


def test_max_size_eviction():
    c = Cache(ttl=60.0, max_size=3)
    c.put("a", 1)
    c.put("b", 2)
    c.put("c", 3)
    c.put("d", 4)  # should evict "a"

    assert c.get("a") is None
    assert c.get("d") == 4
    assert c.size == 3


def test_namespace_get():
    c = Cache(ttl=5.0)
    c.put("robot.0.x", 100)
    c.put("robot.0.y", 200)
    c.put("robot.1.x", -50)
    c.put("ball.x", 0)

    robots = c.get_ns("robot")
    assert len(robots) == 3
    assert "ball.x" not in robots


def test_namespace_invalidate():
    c = Cache(ttl=5.0)
    c.put("robot.0.x", 100)
    c.put("robot.1.x", 200)
    c.put("ball.x", 0)

    c.invalidate_ns("robot")
    assert c.get("robot.0.x") is None
    assert c.get("ball.x") == 0


def test_peek():
    c = Cache(ttl=5.0)
    c.put("exists", True)
    assert c.peek("exists") is True
    assert c.peek("nah") is False


def test_contains():
    c = Cache(ttl=5.0)
    c.put("yep", 1)
    assert "yep" in c
    assert "nope" not in c


def test_clear():
    c = Cache(ttl=5.0)
    c.put("a", 1)
    c.put("b", 2)
    c.clear()
    assert c.size == 0
    assert c.get("a") is None


def test_hit_rate():
    c = Cache(ttl=5.0)
    c.put("x", 1)
    c.get("x")  # hit
    c.get("x")  # hit
    c.get("y")  # miss
    assert c.hit_rate == pytest.approx(2 / 3, abs=0.01)
