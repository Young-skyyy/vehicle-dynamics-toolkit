"""Small ctypes replay adapter, not a general-purpose FMI master."""
from __future__ import annotations
import ctypes
import _ctypes
import os
from pathlib import Path
from typing import Any
from .fmu import FMU_GUID, STATE_REFS
from .simulation import DriverInput


class FMUPlant:
    """Load an extracted reference-vehicle binary; close before removing its directory."""
    def __init__(self, binary: str | Path):
        self.lib = ctypes.CDLL(str(binary))
        self.component = None
        self.closed = False
        c = ctypes.c_void_p
        real = ctypes.c_double
        ref = ctypes.POINTER(ctypes.c_uint)
        signatures: dict[str, tuple[list[Any], Any]] = {
            "fmi2Instantiate": ([ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p,
                                  ctypes.c_char_p, c, ctypes.c_int, ctypes.c_int], c),
            "fmi2FreeInstance": ([c], None),
            "fmi2SetupExperiment": ([c, ctypes.c_int, real, real, ctypes.c_int, real], ctypes.c_int),
            "fmi2DoStep": ([c, real, real, ctypes.c_int], ctypes.c_int),
            "fmi2GetReal": ([c, ref, ctypes.c_size_t, ctypes.POINTER(real)], ctypes.c_int),
            "fmi2SetReal": ([c, ref, ctypes.c_size_t, ctypes.POINTER(real)], ctypes.c_int),
            "fmi2GetInteger": ([c, ref, ctypes.c_size_t, ctypes.POINTER(ctypes.c_int)], ctypes.c_int),
        }
        for name in ("fmi2Reset", "fmi2Terminate", "fmi2EnterInitializationMode", "fmi2ExitInitializationMode"):
            signatures[name] = ([c], ctypes.c_int)
        for name in ("fmi2GetVersion", "fmi2GetTypesPlatform"):
            signatures[name] = ([], ctypes.c_char_p)
        try:
            for name, (arguments, result) in signatures.items():
                function = getattr(self.lib, name)
                function.argtypes, function.restype = arguments, result
            self.component = self.lib.fmi2Instantiate(b"replay", 1, FMU_GUID.encode(), None, None, 0, 0)
            if not self.component:
                raise RuntimeError("FMU instantiate failed")
        except Exception:
            self.close()
            raise

    @staticmethod
    def _require_ok(status: int) -> None:
        if status != 0:
            raise RuntimeError(f"FMU returned status {status}")

    def initialize(self, initial_vx: float = 0., start: float = 0.) -> None:
        self._require_ok(self.lib.fmi2Reset(self.component))
        self.set_reals({19: initial_vx})
        self._require_ok(self.lib.fmi2SetupExperiment(self.component, 0, 0, start, 0, 0))
        self._require_ok(self.lib.fmi2EnterInitializationMode(self.component))
        self._require_ok(self.lib.fmi2ExitInitializationMode(self.component))

    def set_reals(self, values: dict[int, float]) -> None:
        n = len(values)
        refs = (ctypes.c_uint * n)(*values)
        data = (ctypes.c_double * n)(*values.values())
        self._require_ok(self.lib.fmi2SetReal(self.component, refs, n, data))

    def state(self) -> dict[str, float | int]:
        n = len(STATE_REFS)
        refs = (ctypes.c_uint * n)(*STATE_REFS.values())
        data = (ctypes.c_double * n)()
        self._require_ok(self.lib.fmi2GetReal(self.component, refs, n, data))
        state: dict[str, float | int] = dict(zip(STATE_REFS, data))
        gear = (ctypes.c_int * 1)()
        self._require_ok(self.lib.fmi2GetInteger(self.component, (ctypes.c_uint * 1)(6), 1, gear))
        state["gear"] = gear[0]
        return state

    def step(self, dt: float, command: DriverInput) -> dict[str, float | int]:
        self.set_reals({1: command.throttle, 2: command.brake, 8: command.steer})
        self._require_ok(self.lib.fmi2DoStep(self.component, self.state()["t"], dt, 1))
        return self.state()

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        if self.component:
            self.lib.fmi2FreeInstance(self.component)
            self.component = None
        if os.name == "nt":
            _ctypes.FreeLibrary(self.lib._handle)
        else:
            getattr(_ctypes, "dlclose")(self.lib._handle)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
