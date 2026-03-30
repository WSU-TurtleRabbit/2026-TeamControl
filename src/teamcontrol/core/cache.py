"""
Cache system for TRP 2026.

Dead simple key-value store with TTL. Keeps things in memory
so we're not re-parsing the same protobuf data 50 times a second.

Designed to be used from a single process — if you need cross-process
caching, stick the cache inside the WorldModel via the manager proxy.

Usage:
    cache = Cache(ttl=5.0)
    cache.put("ball_velocity", (vx, vy))
    vel = cache.get("ball_velocity")  # returns None if expired

    # or with a namespace
    cache.put("robot.0.position", (x, y))
    cache.put("robot.1.position", (x, y))
    cache.get_ns("robot")  # returns all keys starting with "robot."

@author TRP Team
"""

import time
from collections import OrderedDict
from threading import Lock


class CacheEntry:
    """Single cached value with a timestamp."""

    __slots__ = ("value", "created_at", "ttl")

    def __init__(self, value, ttl: float):
        self.value = value
        self.created_at = time.monotonic()
        self.ttl = ttl

    @property
    def expired(self) -> bool:
        return (time.monotonic() - self.created_at) > self.ttl

    @property
    def age(self) -> float:
        return time.monotonic() - self.created_at

    def __repr__(self):
        status = "EXPIRED" if self.expired else f"age={self.age:.2f}s"
        return f"CacheEntry({status})"


class Cache:
    """
    Thread-safe in-memory cache with TTL and optional size cap.

    put(key, value)        — store something
    get(key)               — get it back (None if missing or expired)
    get_or(key, fallback)  — get it, or return fallback if missing
    peek(key)              — check if key exists and isn't expired
    invalidate(key)        — manually remove a key
    clear()                — nuke everything
    cleanup()              — remove expired entries (called automatically
                             but you can call it yourself if you want)
    """

    def __init__(self, ttl: float = 5.0, max_size: int = 1024):
        self._store: OrderedDict[str, CacheEntry] = OrderedDict()
        self._default_ttl = ttl
        self._max_size = max_size
        self._lock = Lock()
        self._hits = 0
        self._misses = 0
        self._last_cleanup = time.monotonic()
        self._cleanup_every = 10.0  # seconds

    def put(self, key: str, value, ttl: float = None):
        """Store a value. Uses default TTL if none specified."""
        entry = CacheEntry(value, ttl if ttl is not None else self._default_ttl)
        with self._lock:
            # if key already exists, move it to the end (most recent)
            if key in self._store:
                self._store.move_to_end(key)
            self._store[key] = entry

            # evict oldest if we're over the limit
            while len(self._store) > self._max_size:
                self._store.popitem(last=False)

        self._maybe_cleanup()

    def get(self, key: str, default=None):
        """
        Get a cached value. Returns default if the key doesn't
        exist or has expired. Expired entries get removed on access.
        """
        with self._lock:
            entry = self._store.get(key)
            if entry is None:
                self._misses += 1
                return default
            if entry.expired:
                del self._store[key]
                self._misses += 1
                return default
            self._hits += 1
            return entry.value

    def get_or(self, key: str, fallback):
        """Same as get() but with a more explicit name for the default."""
        return self.get(key, default=fallback)

    def peek(self, key: str) -> bool:
        """Check if a key exists and isn't expired, without counting as a hit."""
        with self._lock:
            entry = self._store.get(key)
            if entry is None:
                return False
            return not entry.expired

    def invalidate(self, key: str):
        """Remove a specific key from the cache."""
        with self._lock:
            self._store.pop(key, None)

    def invalidate_ns(self, namespace: str):
        """Remove all keys that start with namespace + '.'"""
        prefix = namespace + "."
        with self._lock:
            keys_to_drop = [k for k in self._store if k.startswith(prefix)]
            for k in keys_to_drop:
                del self._store[k]

    def get_ns(self, namespace: str) -> dict:
        """
        Get all non-expired entries whose keys start with namespace.
        Returns a dict of {key: value} — handy for grabbing all
        robot positions at once, for example.
        """
        prefix = namespace + "."
        result = {}
        with self._lock:
            for key, entry in self._store.items():
                if key.startswith(prefix) and not entry.expired:
                    result[key] = entry.value
        return result

    def clear(self):
        """Remove everything."""
        with self._lock:
            self._store.clear()
            self._hits = 0
            self._misses = 0

    def cleanup(self):
        """Remove all expired entries. Called automatically every so often."""
        with self._lock:
            expired = [k for k, v in self._store.items() if v.expired]
            for k in expired:
                del self._store[k]
        self._last_cleanup = time.monotonic()

    def _maybe_cleanup(self):
        if (time.monotonic() - self._last_cleanup) > self._cleanup_every:
            self.cleanup()

    @property
    def size(self) -> int:
        return len(self._store)

    @property
    def hit_rate(self) -> float:
        total = self._hits + self._misses
        if total == 0:
            return 0.0
        return self._hits / total

    @property
    def stats(self) -> dict:
        return {
            "size": self.size,
            "hits": self._hits,
            "misses": self._misses,
            "hit_rate": f"{self.hit_rate:.1%}",
        }

    def __len__(self):
        return self.size

    def __contains__(self, key: str):
        return self.peek(key)

    def __repr__(self):
        return f"Cache(size={self.size}, ttl={self._default_ttl}s, hit_rate={self.hit_rate:.1%})"


# ── quick test ──────────────────────────────────────────────────
if __name__ == "__main__":
    c = Cache(ttl=1.0, max_size=5)

    c.put("ball.x", 100)
    c.put("ball.y", 200)
    c.put("robot.0.x", 50)
    c.put("robot.0.y", 75)
    c.put("robot.1.x", -50)

    print(c)
    print(f"ball.x = {c.get('ball.x')}")
    print(f"robot namespace: {c.get_ns('robot')}")
    print(f"stats: {c.stats}")

    # test eviction — max_size is 5 so this should push out ball.x
    c.put("robot.1.y", -75)
    print(f"\nafter eviction:")
    print(f"ball.x = {c.get('ball.x')}")  # should be None
    print(f"robot.1.y = {c.get('robot.1.y')}")
    print(c)
