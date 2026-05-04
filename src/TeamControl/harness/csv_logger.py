import csv
import os
from datetime import datetime

class CSVLogger:
    def __init__(self, log_dir, columns):
        self.log_dir = log_dir
        self.columns = list(columns)
        os.makedirs(self.log_dir, exist_ok= True)
        self._file = None 
        self._writer = None 
        self._enabled = False # Boolean Toggle

    def start(self, test_description):
        if self._file is not None: 
            raise RuntimeError("CSVLogger already started - call stop() first") # Stop before starting again
        safe_desc = test_description.replace(" ", "_")
        if "/" in safe_desc or "\\" in safe_desc:
            raise ValueError("Test description must not contain slashes") # Test description contains no dangerous syntaxes
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self.log_dir, f"{ts}_{safe_desc}.csv")
        self._file = open(path, "w", newline = "", buffering = 1)
        self._writer = csv.writer(self._file)
        self._writer.writerow(self.columns)
        return path
    
    def set_enabled(self, on):
        self._enabled = bool(on)
    
    def log(self, **fields):
        if not self._enabled:
            return 
        if self._file is None: 
            raise RuntimeError("CSVLogger.log() called before start")
        ROUND = {"t_ms": 3, "x_pos": 2, "y_pos": 2, "theta_pos": 4}
        row = [round(fields[col], ROUND[col]) if col in fields and col in ROUND else fields[col] if col in fields else "" for col in self.columns]
        self._writer.writerow(row)
    
    def stop(self):
        if self._file is None: 
            return 
        self._file.close()
        self._file = None
        self._writer = None


