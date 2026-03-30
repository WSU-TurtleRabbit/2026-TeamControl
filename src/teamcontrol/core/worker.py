"""
Base worker class for multiprocessing.

Every background process in the system extends this. It handles:
- the run loop
- error counting (auto-shutdown if too many errors in a row)
- clean shutdown
- the class method to spawn it as a Process target

Same idea as the v1 worker but cleaned up a bit.
"""

import time
import logging

log = logging.getLogger(__name__)


class BaseWorker:
    """
    Subclass this, override setup() and step().
    Don't override run() unless you really know what you're doing.
    """

    def __init__(self, is_running, logger=None):
        self.is_running = is_running
        self.log = logger or log
        self._error_count = 0
        self._last_error_time = 0
        self._name = self.__class__.__name__

    def setup(self, *args, **kwargs):
        """
        Called once before the main loop starts.
        Override this to parse queues, world model, whatever you need.
        """
        time.sleep(0.2)  # give other processes a moment to get going
        self.log.info(f"[{self._name}] setup done")

    def step(self):
        """
        Called every iteration of the main loop.
        This is where your actual work goes.
        """
        self.log.info(f"[{self._name}] tick")
        time.sleep(1)

    def run(self):
        """Main loop. Calls step() until told to stop or too many errors."""
        while self.is_running.is_set():
            try:
                self.step()
            except KeyboardInterrupt:
                self.log.warning(f"[{self._name}] caught keyboard interrupt")
                break
            except Exception as e:
                self.log.error(f"[{self._name}] {type(e).__name__}: {e}")
                self._handle_error()
                if self._error_count >= 4:
                    self.log.error(f"[{self._name}] too many errors, bailing out")
                    break

        self.shutdown()

    def _handle_error(self):
        """Track errors — reset the counter if they're spread out."""
        now = time.time()
        if now - self._last_error_time > 4 or self._last_error_time == 0:
            self._last_error_time = now
            self._error_count = 1
        else:
            self._error_count += 1

    def shutdown(self):
        """Called once after the run loop ends. Override for cleanup."""
        self.log.info(f"[{self._name}] shutting down")

    @classmethod
    def run_worker(cls, is_running, logger, *args, **kwargs):
        """
        Entry point for multiprocessing.Process(target=...).
        Creates the worker, sets it up, and runs it.
        """
        worker = cls(is_running, logger)
        worker.setup(*args, **kwargs)
        worker.run()


if __name__ == "__main__":
    from multiprocessing import Process, Event

    is_running = Event()
    is_running.set()

    p = Process(target=BaseWorker.run_worker, args=(is_running, None))
    p.start()

    input("press enter to stop\n")
    is_running.clear()
    p.join(timeout=4)
    print("done")
