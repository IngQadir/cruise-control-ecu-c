"""Vehicle model and driving scenarios around the C cruise control.

The vehicle model and the "demo" scenario are the same as in src/main.c, so the
Python run reproduces the CSV written by ./build/cruise_demo.
"""
from collections import namedtuple
from dataclasses import dataclass

from cc_wrapper import CruiseControl, STATE_NAMES

DT_S = 0.01
STEPS_PER_SECOND = 100

# Vehicle model (km/h and km/h per second), same constants as src/main.c
THROTTLE_GAIN = 0.10
DRAG_GAIN = 0.01
DRIVER_THROTTLE_PCT = 8.0
BRAKE_DECEL = 5.0

Sample = namedtuple("Sample", "t speed target throttle state brake")


@dataclass(frozen=True)
class Scenario:
    name: str
    description: str
    duration_s: float
    main_switch: tuple          # (on_from_s, off_at_s)
    set_press: tuple            # (from_s, to_s)
    uphill: tuple = None        # (from_s, to_s, decel km/h per s)
    brake: tuple = None         # (from_s, to_s)
    fault: tuple = None         # (from_s, to_s) speed signal invalid
    start_speed: float = 80.0


SCENARIOS = {
    "demo": Scenario(
        name="demo",
        description="Activate at 80 km/h, 15 s uphill, brake at 45 s, sensor fault at 55 s",
        duration_s=60.0,
        main_switch=(1.0, 58.0),
        set_press=(2.0, 2.2),
        uphill=(20.0, 35.0, 0.5),
        brake=(45.0, 47.0),
        fault=(55.0, 56.0),
    ),
    "steep_hill": Scenario(
        name="steep_hill",
        description="Steep hill saturates the throttle at 100 %, then the controller recovers",
        duration_s=45.0,
        main_switch=(1.0, 46.0),
        set_press=(2.0, 2.2),
        uphill=(20.0, 30.0, 12.0),
    ),
}


def _in_window(step, window):
    if window is None:
        return False
    return round(window[0] * STEPS_PER_SECOND) <= step < round(window[1] * STEPS_PER_SECOND)


class Simulation:
    """Steps the C controller and a simple vehicle model together."""

    def __init__(self, scenario, controller=None):
        self.scenario = scenario
        self.cc = controller if controller is not None else CruiseControl()
        self.speed = scenario.start_speed
        self.step_index = 0
        self.total_steps = round(scenario.duration_s * STEPS_PER_SECOND)

    @property
    def done(self):
        return self.step_index >= self.total_steps

    def step(self):
        sc = self.scenario
        k = self.step_index

        state, throttle_request = self.cc.step(
            self.speed, DT_S,
            main_switch=_in_window(k, sc.main_switch),
            set_pressed=_in_window(k, sc.set_press),
            brake_pressed=_in_window(k, sc.brake),
            speed_valid=not _in_window(k, sc.fault),
        )

        active = STATE_NAMES[state] == "ACTIVE"
        throttle = throttle_request if active else DRIVER_THROTTLE_PCT
        accel = THROTTLE_GAIN * throttle - DRAG_GAIN * self.speed
        if sc.uphill is not None and _in_window(k, sc.uphill[:2]):
            accel -= sc.uphill[2]
        brake = _in_window(k, sc.brake)
        if brake:
            accel -= BRAKE_DECEL
        self.speed = max(0.0, self.speed + accel * DT_S)

        sample = Sample(
            t=k * DT_S,
            speed=self.speed,
            target=self.cc.target_kmh if active else float("nan"),
            throttle=throttle_request,
            state=state,
            brake=int(brake),
        )
        self.step_index += 1
        return sample

    def advance(self, n_steps):
        samples = []
        for _ in range(n_steps):
            if self.done:
                break
            samples.append(self.step())
        return samples
    