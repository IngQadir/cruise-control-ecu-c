"""ctypes wrapper around the real C cruise control library.

The control logic runs in C (cc_init / cc_step from src/cc_statemachine.c).
Python only supplies inputs and reads outputs. Build the library first:

    cmake -S . -B build && cmake --build build      # creates build/libcc.so

Set the environment variable CC_LIB to load the library from another place.
"""
import ctypes
import os
import sys
from pathlib import Path

STATE_NAMES = ("OFF", "STANDBY", "ACTIVE", "FAULT")
REPO_ROOT = Path(__file__).resolve().parent.parent


# --- mirrors of the C structs (src/pi_controller.h, src/cc_statemachine.h) ---
class PiController(ctypes.Structure):
    _fields_ = [
        ("kp", ctypes.c_float),
        ("ki", ctypes.c_float),
        ("integral", ctypes.c_float),
        ("out_min", ctypes.c_float),
        ("out_max", ctypes.c_float),
    ]


class CcContext(ctypes.Structure):
    _fields_ = [
        ("state", ctypes.c_uint),
        ("target_kmh", ctypes.c_float),
        ("pi", PiController),
    ]


class CcInputs(ctypes.Structure):
    _fields_ = [
        ("main_switch", ctypes.c_uint8),
        ("set_pressed", ctypes.c_uint8),
        ("cancel_pressed", ctypes.c_uint8),
        ("brake_pressed", ctypes.c_uint8),
        ("speed_valid", ctypes.c_uint8),
        ("speed_kmh", ctypes.c_float),
    ]


class CcOutputs(ctypes.Structure):
    _fields_ = [("throttle_pct", ctypes.c_float)]


def default_library_path():
    env = os.environ.get("CC_LIB")
    if env:
        return Path(env)
    name = {"win32": "cc.dll", "darwin": "libcc.dylib"}.get(sys.platform, "libcc.so")
    return REPO_ROOT / "build" / name


class CruiseControl:
    """One instance of the C cruise control module."""

    def __init__(self, library_path=None):
        path = Path(library_path) if library_path else default_library_path()
        if not path.exists():
            raise FileNotFoundError(
                f"{path} not found. Build it first:\n"
                "    cmake -S . -B build && cmake --build build"
            )
        self._lib = ctypes.CDLL(str(path))
        self._lib.cc_init.argtypes = [ctypes.POINTER(CcContext)]
        self._lib.cc_init.restype = None
        self._lib.cc_step.argtypes = [
            ctypes.POINTER(CcContext),
            ctypes.POINTER(CcInputs),
            ctypes.POINTER(CcOutputs),
            ctypes.c_float,
        ]
        self._lib.cc_step.restype = None
        self._ctx = CcContext()
        self.reset()

    def reset(self):
        self._lib.cc_init(ctypes.byref(self._ctx))

    def step(self, speed_kmh, dt_s, *, main_switch=0, set_pressed=0,
             cancel_pressed=0, brake_pressed=0, speed_valid=1):
        """Run one cycle. Returns (state_index, throttle_pct)."""
        inputs = CcInputs(int(main_switch), int(set_pressed), int(cancel_pressed),
                          int(brake_pressed), int(speed_valid), speed_kmh)
        outputs = CcOutputs()
        self._lib.cc_step(ctypes.byref(self._ctx), ctypes.byref(inputs),
                          ctypes.byref(outputs), dt_s)
        return self._ctx.state, outputs.throttle_pct

    @property
    def state(self):
        return int(self._ctx.state)

    @property
    def state_name(self):
        return STATE_NAMES[self.state]

    @property
    def target_kmh(self):
        return float(self._ctx.target_kmh)
    