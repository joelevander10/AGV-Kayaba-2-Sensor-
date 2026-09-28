"""
Abstract base classes for hardware drivers.

Every sensor driver must implement:
  - run(state)       — async task that reads hardware and updates state
  - get_health()     — returns health dict consumed by safety_watchdog

Every actuator driver (output) only needs:
  - run(state)       — async task that consumes from a state queue and writes HW

The safety watchdog only monitors sensor/input drivers (DI, CAN, RFID) because
those are the ones whose staleness could cause the AGV to act on bad data.
Output drivers (DO, AO, SLMP) fail visibly as "motion not responding" and do
not need watchdog supervision.
"""

import time
from abc import ABC, abstractmethod
from collections import deque

# Window over which the observed frame rate is averaged. Long enough to be
# steady at 10 Hz, short enough to react within a couple of seconds.
_RATE_WINDOW_S = 2.0


class SensorDriver(ABC):

    def __init__(self, name: str):
        self._name    = name
        self._last_rx = time.time()  # epoch of last successful hardware read
        self._ok      = False        # True once first successful read
        self._rx_times = deque()     # rx epochs inside the rate window
        self._rx_total = 0           # frames since process start

    @abstractmethod
    async def run(self, state) -> None:
        """Async task body.  Should loop forever, updating state and
        calling self._record_rx() on every successful hardware read."""
        ...

    def _record_rx(self):
        """Call inside run() on every successful read to keep watchdog happy."""
        now = time.time()
        self._last_rx   = now
        self._ok        = True
        self._rx_total += 1
        self._rx_times.append(now)
        cutoff = now - _RATE_WINDOW_S
        while self._rx_times and self._rx_times[0] < cutoff:
            self._rx_times.popleft()

    def get_rate_hz(self) -> float:
        """Observed frames/s averaged over the last _RATE_WINDOW_S seconds.

        Returns 0.0 until two frames have landed inside the window, so a stalled
        driver reads as 0 rather than holding its last good rate.

        Deliberately does NOT mutate _rx_times: this is called from both the
        watchdog task and the Flask thread, while _record_rx() (which does the
        pruning) runs on the event loop. Reading a snapshot keeps that safe.
        """
        cutoff = time.time() - _RATE_WINDOW_S
        recent = [t for t in list(self._rx_times) if t >= cutoff]
        if len(recent) < 2:
            return 0.0
        span = recent[-1] - recent[0]
        return (len(recent) - 1) / span if span > 0 else 0.0

    def get_health(self) -> dict:
        """Called by safety_watchdog every 50 ms.

        Returns:
            {
                "ok":      bool   — False until first read, or after unrecoverable failure
                "last_rx": float  — epoch of last successful read  (0.0 = never)
                "detail":  str    — human-readable identifier for log messages
                "rate_hz": float  — observed frames/s over the last 2 s
                "total":   int    — frames received since process start
            }
        """
        return {
            "ok":      self._ok,
            "last_rx": self._last_rx,
            "detail":  self._name,
            "rate_hz": self.get_rate_hz(),
            "total":   self._rx_total,
        }


class ActuatorDriver(ABC):

    @abstractmethod
    async def run(self, state) -> None:
        """Async task body.  Should loop forever consuming from a state queue."""
        ...
