"""Smoke tests for the ctypes wrapper: they check that Python really drives the C code.

Run:  python3 tools/test_cc_wrapper.py        (or: pytest tools)
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from cc_wrapper import CruiseControl  # noqa: E402
from simulation import SCENARIOS, Simulation  # noqa: E402

DT = 0.01


def make_active(speed=80.0):
    cc = CruiseControl()
    cc.step(speed, DT, main_switch=1)                      # OFF -> STANDBY
    cc.step(speed, DT, main_switch=1, set_pressed=1)       # STANDBY -> ACTIVE
    return cc


def test_starts_in_off():
    assert CruiseControl().state_name == "OFF"


def test_main_switch_gives_standby():
    cc = CruiseControl()
    cc.step(80.0, DT, main_switch=1)
    assert cc.state_name == "STANDBY"


def test_set_activates_and_stores_target():
    cc = make_active(80.0)
    assert cc.state_name == "ACTIVE"
    assert abs(cc.target_kmh - 80.0) < 1e-3


def test_set_below_30_kmh_is_ignored():
    cc = CruiseControl()
    cc.step(25.0, DT, main_switch=1)
    cc.step(25.0, DT, main_switch=1, set_pressed=1)
    assert cc.state_name == "STANDBY"


def test_throttle_is_limited_to_100_percent():
    cc = make_active(100.0)
    _, throttle = cc.step(31.0, DT, main_switch=1)
    assert throttle == 100.0


def test_brake_switches_off():
    cc = make_active()
    state, throttle = cc.step(80.0, DT, main_switch=1, brake_pressed=1)
    assert cc.state_name == "OFF" and throttle == 0.0


def test_invalid_speed_signal_gives_fault():
    cc = make_active()
    cc.step(80.0, DT, main_switch=1, speed_valid=0)
    assert cc.state_name == "FAULT"


def test_demo_scenario_visits_all_states():
    sim = Simulation(SCENARIOS["demo"])
    states = {s.state for s in sim.advance(sim.total_steps)}
    assert states == {0, 1, 2, 3}


def test_steep_hill_saturates_the_throttle():
    sim = Simulation(SCENARIOS["steep_hill"])
    samples = sim.advance(sim.total_steps)
    assert max(s.throttle for s in samples) == 100.0


if __name__ == "__main__":
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_")]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print(f"PASS  {name}")
        except AssertionError as err:
            failed += 1
            print(f"FAIL  {name}  {err}")
    print(f"\n{len(tests) - failed} passed, {failed} failed")
    sys.exit(1 if failed else 0)
    