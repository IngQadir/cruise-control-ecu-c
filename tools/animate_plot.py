"""Animated plot of the C cruise control driven through ctypes.

Live window:   python3 tools/animate_plot.py
Other scenario: python3 tools/animate_plot.py --scenario steep_hill
Save a GIF:    python3 tools/animate_plot.py --save docs/cruise_demo_live.gif

The control decisions come from the real C library (build/libcc.so).
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from cc_wrapper import CruiseControl, STATE_NAMES  # noqa: E402
from simulation import DT_S, SCENARIOS, Simulation  # noqa: E402


class LivePlot:
    """Figure with speed, throttle and state; update() advances the simulation."""

    def __init__(self, scenario, simulation, steps_per_frame):
        import matplotlib.pyplot as plt

        self.sim = simulation
        self.steps_per_frame = steps_per_frame
        self.t, self.speed, self.target = [], [], []
        self.throttle, self.state = [], []

        self.fig, (self.ax_v, self.ax_t, self.ax_s) = plt.subplots(
            3, 1, sharex=True, figsize=(9, 6.5),
            gridspec_kw={"height_ratios": [3, 2, 1.4]})
        fig, ax_v, ax_t, ax_s = self.fig, self.ax_v, self.ax_t, self.ax_s

        ax_v.set_xlim(0, scenario.duration_s)
        ax_v.set_ylim(0, 120)
        ax_v.set_ylabel("Speed [km/h]")
        (self.l_speed,) = ax_v.plot([], [], lw=2, color="tab:blue", label="Vehicle speed")
        (self.l_target,) = ax_v.plot([], [], "--", lw=1.6, color="tab:red",
                                     label="Target (while ACTIVE)")
        ax_v.legend(loc="lower right")

        ax_t.set_ylim(-5, 108)
        ax_t.set_ylabel("Throttle request [%]")
        ax_t.axhline(100, ls=":", color="gray", lw=1)
        (self.l_throttle,) = ax_t.plot([], [], lw=2, color="tab:orange")

        ax_s.set_ylim(-0.5, 3.5)
        ax_s.set_yticks(range(len(STATE_NAMES)))
        ax_s.set_yticklabels(STATE_NAMES)
        ax_s.set_ylabel("State")
        ax_s.set_xlabel("Time [s]")
        (self.l_state,) = ax_s.plot([], [], lw=2, color="tab:green", drawstyle="steps-post")

        for ax in (ax_v, ax_t, ax_s):
            ax.grid(True, alpha=0.4)

        # Mark the events of the scenario
        if scenario.uphill:
            for ax in (ax_v, ax_t):
                ax.axvspan(scenario.uphill[0], scenario.uphill[1], color="tab:brown", alpha=0.12)
            ax_v.text(scenario.uphill[0] + 0.3, 112, "uphill", fontsize=8, color="tab:brown")
        if scenario.brake:
            for ax in (ax_v, ax_t):
                ax.axvspan(scenario.brake[0], scenario.brake[1], color="tab:red", alpha=0.12)
            ax_v.text(scenario.brake[0] + 0.3, 112, "brake", fontsize=8, color="tab:red")
        if scenario.fault:
            for ax in (ax_v, ax_t):
                ax.axvspan(scenario.fault[0], scenario.fault[1], color="black", alpha=0.12)
            ax_v.text(scenario.fault[0] - 4.0, 112, "speed signal invalid", fontsize=8)

        fig.suptitle("Cruise control (C code via ctypes): " + scenario.description,
                     fontsize=9, y=0.985)
        self.status = fig.text(0.5, 0.925, "", ha="center", fontsize=11, fontweight="bold")
        fig.tight_layout(rect=(0, 0, 1, 0.90))

    def update(self, _frame=None):
        for s in self.sim.advance(self.steps_per_frame):
            self.t.append(s.t)
            self.speed.append(s.speed)
            self.target.append(s.target)
            self.throttle.append(s.throttle)
            self.state.append(s.state)
        if not self.t:
            return
        self.l_speed.set_data(self.t, self.speed)
        self.l_target.set_data(self.t, self.target)
        self.l_throttle.set_data(self.t, self.throttle)
        self.l_state.set_data(self.t, self.state)
        self.status.set_text(
            f"t = {self.t[-1]:5.1f} s    state = {STATE_NAMES[self.state[-1]]:<8}"
            f"  speed = {self.speed[-1]:5.1f} km/h    throttle = {self.throttle[-1]:5.1f} %")


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--scenario", choices=sorted(SCENARIOS), default="demo")
    p.add_argument("--lib", help="path to libcc.so (default: build/libcc.so or $CC_LIB)")
    p.add_argument("--save", metavar="FILE", help="write a .gif (or .mp4 with ffmpeg) instead of opening a window")
    p.add_argument("--speed", type=float, default=2.0,
                   help="live playback speed, 1 = real time (default 2)")
    p.add_argument("--steps-per-frame", type=int, default=None,
                   help="simulation steps (10 ms each) per frame; default 10 live, 20 when saving")
    p.add_argument("--fps", type=int, default=12, help="frames per second of a saved file")
    return p.parse_args()


def main():
    args = parse_args()

    import matplotlib
    if args.save:
        matplotlib.use("Agg")          # no window needed, also works headless
    from matplotlib.animation import FuncAnimation, PillowWriter

    try:
        controller = CruiseControl(args.lib)
    except FileNotFoundError as err:
        print(err, file=sys.stderr)
        return 1

    scenario = SCENARIOS[args.scenario]
    sim = Simulation(scenario, controller)
    spf = args.steps_per_frame or (20 if args.save else 10)
    plot = LivePlot(scenario, sim, spf)
    frames = -(-sim.total_steps // spf)            # ceiling division

    if args.save:
        anim = FuncAnimation(plot.fig, plot.update, frames=frames, repeat=False)
        out = Path(args.save)
        out.parent.mkdir(parents=True, exist_ok=True)
        if out.suffix.lower() == ".gif":
            anim.save(str(out), writer=PillowWriter(fps=args.fps), dpi=70)
        else:
            anim.save(str(out), fps=args.fps, dpi=100)
        print(f"saved {out} ({frames} frames)")
        return 0

    import matplotlib.pyplot as plt
    interval_ms = spf * DT_S * 1000.0 / args.speed
    anim = FuncAnimation(plot.fig, plot.update, frames=frames,
                         interval=interval_ms, repeat=False)
    plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
    