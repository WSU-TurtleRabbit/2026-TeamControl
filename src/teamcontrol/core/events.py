"""
Simple event bus for decoupled communication.

Lets different parts of the system subscribe to events without
having to know about each other directly. Think of it like
a lightweight pub/sub within a single process.

Usage:
    bus = EventBus()
    bus.on("ball_detected", my_handler)
    bus.emit("ball_detected", x=100, y=200)

Not for cross-process stuff — use queues for that.
This is for keeping things tidy within a single worker.
"""

from collections import defaultdict
import logging

log = logging.getLogger(__name__)


class EventBus:
    def __init__(self):
        self._listeners = defaultdict(list)

    def on(self, event_name: str, callback):
        """Subscribe to an event."""
        self._listeners[event_name].append(callback)

    def off(self, event_name: str, callback):
        """Unsubscribe from an event."""
        try:
            self._listeners[event_name].remove(callback)
        except ValueError:
            pass  # wasn't subscribed, no big deal

    def emit(self, event_name: str, *args, **kwargs):
        """Fire an event. All registered callbacks get called."""
        for cb in self._listeners.get(event_name, []):
            try:
                cb(*args, **kwargs)
            except Exception as e:
                log.error(f"[EventBus] handler for '{event_name}' blew up: {e}")

    def clear(self, event_name: str = None):
        """Remove listeners. If no name given, clears everything."""
        if event_name:
            self._listeners.pop(event_name, None)
        else:
            self._listeners.clear()

    @property
    def events(self) -> list:
        """List all events that have at least one listener."""
        return [k for k, v in self._listeners.items() if v]

    def __repr__(self):
        count = sum(len(v) for v in self._listeners.values())
        return f"EventBus({count} listeners across {len(self.events)} events)"
