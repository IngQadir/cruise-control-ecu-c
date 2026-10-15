"""2D road view of the C cruise control (pygame).

Live window:    python3 tools/road_view.py
Other scenario: python3 tools/road_view.py --scenario steep_hill
Save a GIF:     python3 tools/road_view.py --save docs/road_view.gif      (no window)

Keys:  SPACE pause   R restart   1 / 2 scenario   UP / DOWN playback speed   ESC quit

The control decisions come from the real C library (build/libcc.so);
pygame only draws the vehicle, the road and the instruments.
"""
import argparse
import math
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from cc_wrapper import CruiseControl, STATE_NAMES  # noqa: E402
from simulation import DT_S, SCENARIOS, Simulation  # noqa: E402

W, H = 960, 540
HORIZON_Y = 250
ROAD_Y = 380            # y of the road centre line
ROAD_H = 110
PX_PER_M = 14.0
CAR_X = 330

SKY_TOP, SKY_BOTTOM = (92, 154, 214), (200, 228, 245)
GRASS = (88, 140, 78)
ASPHALT = (62, 64, 70)
STATE_COLORS = {
    "OFF": (120, 120, 120),
    "STANDBY": (220, 170, 40),
    "ACTIVE": (60, 170, 90),
    "FAULT": (210, 60, 60),
}


def lerp(a, b, t):
    return a + (b - a) * t


class RoadView:
    def __init__(self, scenario_name, library=None):
        import pygame
        self.pg = pygame
        self.library = library
        self.font = pygame.font.SysFont("dejavusans,arial,helvetica", 18)
        self.font_big = pygame.font.SysFont("dejavusans,arial,helvetica", 40, bold=True)
        self.font_small = pygame.font.SysFont("dejavusans,arial,helvetica", 14)
        self.sky = self._make_sky()
        self.scene = pygame.Surface((W * 2, 560), pygame.SRCALPHA)
        self.load(scenario_name)

    # ---- simulation -------------------------------------------------------
    def load(self, scenario_name):
        self.scenario_name = scenario_name
        self.scenario = SCENARIOS[scenario_name]
        self.sim = Simulation(self.scenario, CruiseControl(self.library))
        self.sample = None
        self.distance_m = 0.0
        self.angle = 0.0
        self.budget = 0.0

    def advance(self, dt_real, playback):
        """Run the C controller for dt_real * playback seconds of simulated time."""
        self.budget += dt_real * playback
        steps = int(self.budget / DT_S)
        self.budget -= steps * DT_S
        for _ in range(steps):
            if self.sim.done:
                break
            self.sample = self.sim.step()
            self.distance_m += self.sample.speed / 3.6 * DT_S

    # ---- drawing ----------------------------------------------------------
    def _make_sky(self):
        pg = self.pg
        sky = pg.Surface((W, H))
        for y in range(H):
            t = min(1.0, y / HORIZON_Y)
            sky.fill(tuple(int(lerp(SKY_TOP[i], SKY_BOTTOM[i], t)) for i in range(3)),
                     (0, y, W, 1))
        return sky

    def _slope_target_deg(self):
        sc, k = self.scenario, self.sim.step_index
        if sc.uphill and sc.uphill[0] * 100 <= k < sc.uphill[1] * 100:
            return min(15.0, 2.5 + sc.uphill[2] * 0.8)
        return 0.0

    def draw(self, screen):
        pg = self.pg
        s = self.sample
        self.angle += (self._slope_target_deg() - self.angle) * 0.04

        screen.blit(self.sky, (0, 0))
        self._draw_hills(screen)
        self._draw_scene(screen)
        self._draw_hud(screen, s)

    def _draw_hills(self, screen):
        pg = self.pg
        off = self.distance_m * PX_PER_M * 0.05
        pts = [(0, H)]
        for x in range(0, W + 20, 20):
            y = HORIZON_Y - 38 - 26 * math.sin((x + off) / 120.0) - 14 * math.sin((x + off) / 47.0)
            pts.append((x, y))
        pts.append((W, H))
        pg.draw.polygon(screen, (120, 160, 150), pts)

    def _draw_scene(self, screen):
        """Road, markings, posts and car drawn flat, then tilted by the slope."""
        pg = self.pg
        sc = self.scene
        sc.fill((0, 0, 0, 0))
        cx, cy = sc.get_width() // 2, sc.get_height() // 2

        # grass and road band
        pg.draw.rect(sc, GRASS, (0, cy - 40, sc.get_width(), 400))
        pg.draw.rect(sc, ASPHALT, (0, cy - 20, sc.get_width(), ROAD_H))
        pg.draw.rect(sc, (235, 235, 235), (0, cy - 20, sc.get_width(), 3))
        pg.draw.rect(sc, (235, 235, 235), (0, cy - 20 + ROAD_H - 3, sc.get_width(), 3))

        # lane dashes and roadside posts scroll with the travelled distance
        scroll = (self.distance_m * PX_PER_M)
        dash_len, dash_gap = 5 * PX_PER_M, 7 * PX_PER_M
        period = dash_len + dash_gap
        x = -(scroll % period) - period
        while x < sc.get_width():
            pg.draw.rect(sc, (240, 220, 120), (x, cy + ROAD_H // 2 - 22, dash_len, 4))
            x += period
        post_period = 30 * PX_PER_M
        x = -(scroll % post_period) - post_period
        while x < sc.get_width():
            pg.draw.rect(sc, (225, 225, 225), (x, cy - 62, 5, 42))
            pg.draw.rect(sc, (210, 60, 60), (x, cy - 62, 5, 9))
            x += post_period

        # car: wheels centred on the road, sits at CAR_X on screen
        self._draw_car(sc, cx - (W // 2 - CAR_X), cy - 20 + 18)

        rotated = pg.transform.rotozoom(sc, self.angle, 1.0)
        # rotate about the car so that it stays at the same screen position
        car_screen = (CAR_X, ROAD_Y)
        car_in_scene = pg.math.Vector2(cx - (W // 2 - CAR_X), cy - 20 + 18 + 30)
        rel = car_in_scene - pg.math.Vector2(cx, cy)
        rel = rel.rotate(-self.angle)
        rcx, rcy = rotated.get_width() / 2, rotated.get_height() / 2
        pos = (car_screen[0] - (rcx + rel.x), car_screen[1] - (rcy + rel.y))
        screen.blit(rotated, pos)

    def _draw_car(self, surf, x, y):
        pg = self.pg
        s = self.sample
        braking = bool(s and s.brake)
        body = (196, 38, 46)
        # body, cabin, windows
        pg.draw.rect(surf, body, (x - 70, y + 6, 140, 34), border_radius=8)
        pg.draw.polygon(surf, body, [(x - 38, y + 8), (x - 20, y - 18), (x + 26, y - 18), (x + 46, y + 8)])
        pg.draw.polygon(surf, (170, 210, 235), [(x - 30, y + 6), (x - 17, y - 13), (x - 2, y - 13), (x - 2, y + 6)])
        pg.draw.polygon(surf, (170, 210, 235), [(x + 3, y + 6), (x + 3, y - 13), (x + 22, y - 13), (x + 38, y + 6)])
        # lights
        pg.draw.rect(surf, (255, 240, 180), (x + 62, y + 12, 8, 8), border_radius=2)
        tail = (255, 60, 40) if braking else (130, 25, 25)
        pg.draw.rect(surf, tail, (x - 72, y + 12, 8, 10), border_radius=2)
        if braking:
            pg.draw.circle(surf, (255, 90, 60, 90), (x - 76, y + 17), 16)
        # wheels
        spin = self.distance_m * 3.0
        for wx in (x - 42, x + 44):
            pg.draw.circle(surf, (25, 25, 28), (wx, y + 40), 16)
            pg.draw.circle(surf, (150, 150, 155), (wx, y + 40), 8)
            ax, ay = math.cos(spin) * 7, math.sin(spin) * 7
            pg.draw.line(surf, (60, 60, 65), (wx - ax, y + 40 - ay), (wx + ax, y + 40 + ay), 2)

    # ---- instruments ------------------------------------------------------
    def _text(self, screen, text, pos, font=None, color=(255, 255, 255), anchor="topleft"):
        img = (font or self.font).render(text, True, color)
        rect = img.get_rect(**{anchor: pos})
        screen.blit(img, rect)
        return rect

    def _draw_hud(self, screen, s):
        pg = self.pg
        state = STATE_NAMES[s.state] if s else "OFF"
        speed = s.speed if s else self.scenario.start_speed
        throttle = s.throttle if s else 0.0
        t = s.t if s else 0.0
        target = s.target if s else float("nan")

        # top bar
        bar = pg.Surface((W, 44), pg.SRCALPHA)
        bar.fill((20, 24, 32, 200))
        screen.blit(bar, (0, 0))
        self._text(screen, "Cruise control: C code via ctypes", (14, 11))
        self._text(screen, f"{self.scenario.name}   t = {t:5.1f} s", (W - 14, 11), anchor="topright")

        # state badge
        col = STATE_COLORS[state]
        badge = pg.Rect(14, 56, 150, 38)
        pg.draw.rect(screen, col, badge, border_radius=10)
        self._text(screen, state, badge.center, self.font,
                   color=(255, 255, 255), anchor="center")

        # event banners
        banners = []
        if s:
            k = self.sim.step_index
            sc = self.scenario
            if sc.uphill and sc.uphill[0] * 100 <= k < sc.uphill[1] * 100:
                banners.append(("UPHILL", (150, 100, 50)))
            if s.brake:
                banners.append(("BRAKE PEDAL", (200, 50, 50)))
            if sc.fault and sc.fault[0] * 100 <= k < sc.fault[1] * 100:
                banners.append(("SPEED SIGNAL INVALID", (60, 60, 60)))
        bx = 176
        for text, bcol in banners:
            r = self._text(screen, text, (bx + 10, 66), self.font_small)
            pg.draw.rect(screen, bcol, r.inflate(20, 12), border_radius=8)
            self._text(screen, text, (bx + 10, 66), self.font_small)
            bx += r.width + 34

        # speedometer
        gx, gy, gr = W - 150, H - 150, 105
        pg.draw.circle(screen, (24, 28, 36), (gx, gy), gr + 12)
        pg.draw.circle(screen, (44, 50, 62), (gx, gy), gr + 12, 3)
        vmax = 140.0
        a0, a1 = math.radians(225), math.radians(-45)

        def ang(v):
            return lerp(a0, a1, max(0.0, min(vmax, v)) / vmax)

        for v in range(0, 141, 10):
            major = v % 20 == 0
            a = ang(v)
            r_out, r_in = gr, gr - (14 if major else 8)
            pg.draw.line(screen, (230, 230, 230) if major else (140, 140, 140),
                         (gx + r_in * math.cos(a), gy - r_in * math.sin(a)),
                         (gx + r_out * math.cos(a), gy - r_out * math.sin(a)), 2)
            if major:
                self._text(screen, str(v), (gx + (gr - 28) * math.cos(a), gy - (gr - 28) * math.sin(a)),
                           self.font_small, anchor="center")
        if state == "ACTIVE" and not math.isnan(target):
            a = ang(target)
            pg.draw.line(screen, (255, 90, 80),
                         (gx + (gr - 18) * math.cos(a), gy - (gr - 18) * math.sin(a)),
                         (gx + (gr + 8) * math.cos(a), gy - (gr + 8) * math.sin(a)), 5)
        a = ang(speed)
        pg.draw.line(screen, (90, 190, 255), (gx, gy),
                     (gx + (gr - 20) * math.cos(a), gy - (gr - 20) * math.sin(a)), 4)
        pg.draw.circle(screen, (90, 190, 255), (gx, gy), 7)
        self._text(screen, f"{speed:5.1f}", (gx, gy + 38), self.font_big, anchor="center")
        self._text(screen, "km/h", (gx, gy + 66), self.font_small, color=(170, 170, 170), anchor="center")
        if state == "ACTIVE" and not math.isnan(target):
            self._text(screen, f"SET {target:.0f}", (gx, gy - 45), self.font_small,
                       color=(255, 120, 110), anchor="center")

        # throttle bar
        bx0, by0, bw, bh = W - 340, H - 250, 26, 200
        pg.draw.rect(screen, (24, 28, 36), (bx0 - 6, by0 - 6, bw + 12, bh + 12), border_radius=8)
        fill = int(bh * max(0.0, min(100.0, throttle)) / 100.0)
        tcol = (255, 150, 40) if throttle < 99.9 else (230, 60, 60)
        pg.draw.rect(screen, tcol, (bx0, by0 + bh - fill, bw, fill), border_radius=4)
        self._text(screen, "Throttle", (bx0 + bw // 2, by0 - 22), self.font_small, anchor="center")
        self._text(screen, f"{throttle:.0f} %", (bx0 + bw // 2, by0 + bh + 18), self.font_small, anchor="center")

        # footer hint
        hint = "SPACE pause   R restart   1/2 scenario   UP/DOWN speed   ESC quit"
        self._text(screen, hint, (14, H - 22), self.font_small, color=(30, 40, 50))
        if self.sim.done:
            self._text(screen, "end of scenario: press R", (W // 2, 120), self.font, color=(30, 40, 50),
                       anchor="center")


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--scenario", choices=sorted(SCENARIOS), default="demo")
    p.add_argument("--lib", help="path to libcc.so (default: build/libcc.so or $CC_LIB)")
    p.add_argument("--speed", type=float, default=2.0, help="live playback speed, 1 = real time (default 2)")
    p.add_argument("--save", metavar="FILE.gif", help="write a GIF without opening a window")
    p.add_argument("--screenshot", metavar="FILE.png", help="save one PNG and exit")
    p.add_argument("--at", type=float, default=30.0, help="simulation time in s for --screenshot")
    p.add_argument("--fps", type=int, default=15, help="frames per second of a saved GIF")
    p.add_argument("--gif-speed", type=float, default=4.0, help="simulation speed in a saved GIF (default 4)")
    return p.parse_args()


def run_offline(view, args):
    """No window: render frames from the simulation, used for GIF and PNG export."""
    import pygame
    screen = pygame.Surface((W, H))
    if args.screenshot:
        view.advance(args.at, 1.0)
        view.angle = view._slope_target_deg()
        view.draw(screen)
        pygame.image.save(screen, args.screenshot)
        print(f"saved {args.screenshot}")
        return 0

    from PIL import Image
    frames = []
    dt = 1.0 / args.fps
    while not view.sim.done:
        view.advance(dt, args.gif_speed)
        view.draw(screen)
        raw = pygame.image.tostring(screen, "RGB")
        frames.append(Image.frombytes("RGB", (W, H), raw).resize((W // 2, H // 2), Image.LANCZOS))
    out = Path(args.save)
    out.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(out, save_all=True, append_images=frames[1:], duration=int(1000 / args.fps),
                   loop=0, optimize=True)
    print(f"saved {out} ({len(frames)} frames)")
    return 0


def main():
    args = parse_args()
    offline = bool(args.save or args.screenshot)
    if offline:
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

    import pygame
    pygame.display.init()      # no audio needed
    pygame.font.init()

    try:
        view_screen = None if offline else pygame.display.set_mode((W, H))
        view = RoadView(args.scenario, args.lib)
    except FileNotFoundError as err:
        print(err, file=sys.stderr)
        return 1

    if offline:
        return run_offline(view, args)

    pygame.display.set_caption("Cruise control road view (C code via ctypes)")
    clock = pygame.time.Clock()
    playback, paused, running = args.speed, False, True
    while running:
        dt = clock.tick(60) / 1000.0
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE:
                    running = False
                elif ev.key == pygame.K_SPACE:
                    paused = not paused
                elif ev.key == pygame.K_r:
                    view.load(view.scenario_name)
                elif ev.key == pygame.K_1:
                    view.load("demo")
                elif ev.key == pygame.K_2:
                    view.load("steep_hill")
                elif ev.key == pygame.K_UP:
                    playback = min(8.0, playback * 1.5)
                elif ev.key == pygame.K_DOWN:
                    playback = max(0.25, playback / 1.5)
        if not paused:
            view.advance(dt, playback)
        view.draw(view_screen)
        pygame.display.flip()
    pygame.quit()
    return 0


if __name__ == "__main__":
    sys.exit(main())
    