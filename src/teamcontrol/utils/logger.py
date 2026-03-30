"""
File Logger
Created by @JustinShirzad — carried over from v1 with some tweaks.

Creates timestamped log files per process. Each process gets its
own file so logs don't get tangled up in multiprocessing.

Format:
    [ INFO ] : 14:30:45 12-07-2025 : my_script - Line 11 : Process started
    [ ERROR ] : 14:30:46 12-07-2025 : my_script - Line 12 : Connection failed
"""

import os
import sys
import logging
import inspect
from datetime import datetime


class LogSaver:
    def __init__(
        self,
        log_dir="logs",
        process_name=None,
        id=None,
        show_timestamp=True,
        show_process_name=True,
        show_line_number=True,
        show_level=True,
    ):
        self.log_dir = log_dir
        self.process_name = process_name
        self.id = id
        self.show_timestamp = show_timestamp
        self.show_process_name = show_process_name
        self.show_line_number = show_line_number
        self.show_level = show_level

        self.log_file, self.process_name = self._create_log_file()
        self._setup_logger()

    def _create_log_file(self):
        os.makedirs(self.log_dir, exist_ok=True)

        # figure out a name for this logger
        if self.process_name is None:
            frame = inspect.currentframe().f_back.f_back
            name = frame.f_code.co_name
            if name == "<module>":
                script = os.path.basename(sys.argv[0])
                name = script.removesuffix(".py")
        else:
            name = self.process_name

        if self.id is not None:
            name = f"{name}_{self.id}"

        timestamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")
        filename = f"{name}_{timestamp}.log"
        filepath = os.path.join(self.log_dir, filename)

        with open(filepath, "w") as f:
            f.write(f"|=== START OF LOG FOR: {name} ===|\n")
            f.write(f"|=== Started: {datetime.now().strftime('%H:%M:%S %d-%m-%Y')} ===|\n")

        return filepath, name

    def _setup_logger(self):
        self.logger = logging.getLogger(self.process_name)
        self.logger.setLevel(logging.DEBUG)
        self.logger.handlers.clear()

        handler = logging.FileHandler(self.log_file)
        handler.setLevel(logging.DEBUG)

        fmt = "[ %(levelname)s ] : %(asctime)s : %(name)s - %(message)s"
        if not self.show_level:
            fmt = fmt.replace("[ %(levelname)s ] : ", "")
        if not self.show_timestamp:
            fmt = fmt.replace("%(asctime)s : ", "")
        if not self.show_process_name:
            fmt = fmt.replace("%(name)s", "")

        handler.setFormatter(logging.Formatter(fmt=fmt, datefmt="%H:%M:%S %d-%m-%Y"))
        self.logger.addHandler(handler)

    def _log(self, level, message):
        frame = inspect.currentframe().f_back.f_back
        lineno = frame.f_lineno
        if self.show_line_number:
            msg = f"Line {lineno} : {message}"
        else:
            msg = f" : {message}"
        getattr(self.logger, level)(msg)

    # shorthand
    def D(self, msg): self._log("debug", str(msg))
    def I(self, msg): self._log("info", str(msg))
    def W(self, msg): self._log("warning", str(msg))
    def E(self, msg): self._log("error", str(msg))
    def C(self, msg): self._log("critical", str(msg))

    # full names
    def debug(self, msg): self._log("debug", str(msg))
    def info(self, msg): self._log("info", str(msg))
    def warning(self, msg): self._log("warning", str(msg))
    def error(self, msg): self._log("error", str(msg))
    def critical(self, msg): self._log("critical", str(msg))


if __name__ == "__main__":
    logs = LogSaver()
    logs.info("This is an info message.")
    logs.debug("This is a debug message.")
    logs.warning("This is a warning message.")
    logs.error("This is an error message.")
    logs.critical("This is a critical message.")
