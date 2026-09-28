"""
logger.py — Centralised logging configuration for AGV KIM2A controller.

Call setup_logging() once from main.py before any tasks start.
Every other module creates its own logger via: logger = logging.getLogger(__name__)

Levels used across the codebase
--------------------------------
  DEBUG    — high-frequency data: PID loop values, raw sensor frames,
             RFID ignore-window skips, serial port discovery
  INFO     — normal operational events: connections, mode transitions,
             RFID commands, run start/stop
  WARNING  — recoverable fault conditions: tape loss, CAN timeout,
             hardware reconnects, unrecognised RFID tags
  ERROR    — communication failures and unhandled exceptions
  CRITICAL — emergency stop triggered

Output
------
  Console : INFO and above by default (pass console_level=logging.DEBUG to
            see PID loop output in the terminal during tuning sessions)
  File    : DEBUG and above, rotating (10 MB × 5 files) in ./logs/
            Each run creates a new file: logs/agv_YYYYMMDD_HHMMSS.log
"""

import logging
import logging.handlers
import os
import queue
import time
from collections import deque
from datetime import datetime

# In-memory ring buffer for WARNING+ records — read by the Flask /errors page
_error_log: deque = deque(maxlen=500)

# Background log dispatcher — keeps file/console I/O off the control loop thread
_listener = None


class _MemoryLogHandler(logging.Handler):
    """Appends WARNING and above records to the module-level deque."""
    def emit(self, record: logging.LogRecord) -> None:
        # formatTime lives on Formatter, not Handler — derive the timestamp
        # directly from the record so this works standalone (and behind the
        # QueueListener).
        ts = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(record.created))
        _error_log.append({
            "time":    f"{ts}.{record.msecs:03.0f}",
            "level":   record.levelname,
            "logger":  record.name,
            "message": record.getMessage(),
        })


def get_error_log() -> list:
    """Return a copy of the in-memory error/warning log (newest last)."""
    return list(_error_log)


def setup_logging(
    log_dir: str = "logs",
    console_level: int = logging.INFO,
) -> None:
    """
    Configure the root logger with a console handler and a rotating file handler.

    Args:
        log_dir:       Directory for log files (created if absent).
        console_level: Minimum level printed to stdout. Default is INFO.
                       Pass logging.DEBUG to see PID loop output on the console.
    """
    os.makedirs(log_dir, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path  = os.path.join(log_dir, f"agv_{timestamp}.log")

    fmt = logging.Formatter(
        fmt="%(asctime)s.%(msecs)03d [%(levelname)-8s] %(name)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    # File handler: INFO and above (was DEBUG — the 100 Hz PID debug line must
    # not hit disk every cycle), rotating at 10 MB, keep 5 backups
    file_handler = logging.handlers.RotatingFileHandler(
        log_path, maxBytes=10 * 1024 * 1024, backupCount=5, encoding="utf-8"
    )
    file_handler.setLevel(logging.INFO)
    file_handler.setFormatter(fmt)

    # Console handler: configurable level (default INFO)
    console_handler = logging.StreamHandler()
    console_handler.setLevel(console_level)
    console_handler.setFormatter(fmt)

    memory_handler = _MemoryLogHandler()
    memory_handler.setLevel(logging.WARNING)

    # Decouple logging I/O from the control loop: the root logger only enqueues
    # records (non-blocking); a background QueueListener thread runs the real
    # file/console/memory handlers, so an SD-card write stall cannot jitter the
    # asyncio control loop.
    global _listener
    log_queue: queue.Queue = queue.Queue(-1)
    queue_handler = logging.handlers.QueueHandler(log_queue)

    root = logging.getLogger()
    root.setLevel(logging.DEBUG)   # root captures all; handlers filter
    root.addHandler(queue_handler)

    _listener = logging.handlers.QueueListener(
        log_queue, file_handler, console_handler, memory_handler,
        respect_handler_level=True)
    _listener.start()

    # Silence third-party library noise
    logging.getLogger("werkzeug").setLevel(logging.ERROR)
    logging.getLogger("pymodbus").setLevel(logging.WARNING)
    logging.getLogger("can").setLevel(logging.WARNING)

    logging.getLogger(__name__).info(
        "Logging initialised — file: %s  console: %s",
        log_path,
        logging.getLevelName(console_level),
    )
