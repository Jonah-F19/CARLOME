"""
intersection.py - an improved version of example_intersection.py.

Car 1 (red, the "ego" car) follows a scripted plan. Every tick a safety layer
predicts every road user a couple of seconds ahead, using only what the ego
car can observe, and brakes just hard enough to stop short of any predicted
collision. It resumes the plan once the path has stayed clear.

Usage:
    python3 intersection.py                  # watch it in a window
    python3 intersection.py --speed 2        # 2x real time
    python3 intersection.py --no-safety      # baseline: ego ignores everyone
    python3 intersection.py --seed 7         # a randomized version of the scene
    python3 intersection.py --headless       # no window, print a summary
    python3 intersection.py --eval 100       # 100 randomized runs, safety on vs off
"""

import argparse
import bisect
import math
import random
import time
from dataclasses import dataclass, field

from agents import Car, Pedestrian, RectangleBuilding, Painting
from entities import first_conflict
from geometry import Point
from world import World

DT = 0.1                 # seconds per tick
SIZE = 120               # the world is SIZE x SIZE meters
MAX_TIME = 60.0          # seconds before a run is stopped
HORIZON_S = 2.0          # how far ahead (seconds) collisions are predicted
SAFETY_MARGIN = 0.5      # meters added around every shape when predicting
STOP_BUFFER = 1.0        # meters to stop short of a predicted contact
MAX_BRAKE = 6.0          # strongest deceleration the ego car may use, m/s^2
MAX_ACCEL = 1.5          # strongest throttle the ego car may use to get clear
CLEAR_TICKS = 5          # ticks the path must stay clear before resuming
CLEARED_Y = 100.0        # the ego car has made it through once past this y
MODE_COLORS = {'CRUISE': 'lime green', 'WAIT': 'gold', 'YIELD': 'orange red', 'HOLD': 'red',
               'BRAKE': 'red', 'ESCAPE': 'magenta', 'OFF': 'gray60'}


def wrap_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# --- Drivers: scripted controls as data ------------------------------------
class Plan:
    """{tick: (steering, throttle)}; each entry holds until the next one."""

    def __init__(self, events):
        self.ticks = sorted(events)
        self.controls = [events[t] for t in self.ticks]

    def control(self, tick, agent=None):
        return self.controls[max(bisect.bisect_right(self.ticks, tick) - 1, 0)]


class TurningDriver:
    """Drives straight, starts turning at a trigger x position and straightens
    out once it faces the target heading, so randomized speeds stay on the road."""

    def __init__(self, cruise, turn_at_x, turn, exit, target_heading):
        self.cruise, self.turn, self.exit = cruise, turn, exit
        self.turn_at_x, self.target_heading = turn_at_x, target_heading
        self.phase = 'approach'

    def control(self, tick, agent):
        if self.phase == 'approach' and agent.x <= self.turn_at_x:
            self.phase = 'turn'
        if self.phase == 'turn' and \
                wrap_angle(self.target_heading - agent.heading) * math.copysign(1, self.turn[0]) <= 0.02:
            self.phase = 'exit'
        return {'approach': self.cruise, 'turn': self.turn, 'exit': self.exit}[self.phase]


def build_world(rng=None, headless=True):
    """The original intersection scene; with an rng the other road users'
    speed, position and timing are randomized a little."""
    pick = lambda default, low, high: rng.uniform(low, high) if rng else default
    w = World(DT, width=SIZE, height=SIZE, ppm=6, headless=headless)
    # (sidewalk center, sidewalk size, building center): each building sits on its sidewalk
    for walk, size, building in [((71.5, 106.5), (97, 27), (72.5, 107.5)), ((8.5, 106.5), (17, 27), (7.5, 107.5)),
                                 ((8.5, 41), (17, 82), (7.5, 40)), ((71.5, 41), (97, 82), (72.5, 40))]:
        w.add(Painting(Point(*walk), Point(*size), 'gray80'))
        w.add(RectangleBuilding(Point(*building), Point(size[0] - 2, size[1] - 2)))
    for x in range(18, 23):
        w.add(Painting(Point(x, 81), Point(0.5, 2), 'white'))               # zebra crossing

    ego = Car(Point(20, 20), math.pi / 2, 'red')
    plan = Plan({0: (0, pick(0.35, 0.30, 0.40)), 100: (0, 0), 200: (0, -0.02), 325: (0, 0.8)})
    car2 = Car(Point(118, 90), math.pi, 'blue')
    car2.velocity = Point(-pick(3.0, 2.4, 3.6), 0)
    car2_driver = TurningDriver(cruise=(0, 0.05), turn_at_x=pick(25.6, 25.0, 26.5),
                                turn=(-0.45, 0.3), exit=(0, 0.1), target_heading=math.pi / 2)
    ped = Pedestrian(Point(pick(28, 26, 32), 81), math.pi)
    ped.max_speed = 10.0
    for agent in (ego, car2, ped):
        w.add(agent)
    return w, ego, plan, [(car2, car2_driver), (ped, Plan({0: (0, pick(0.22, 0.21, 0.25))}))]


# --- Perception: estimate others' motion from what can be seen --------------
class Observer:
    """Tracks each agent's speed change and turning rate tick to tick (it never
    reads their controls) and extrapolates their paths from that."""

    def __init__(self, agents):
        self.state = {a: (a.speed, float(a.heading), 0., 0.) for a in agents}

    def update(self):
        for a, (speed, heading, _, _) in self.state.items():
            self.state[a] = (a.speed, float(a.heading), (a.speed - speed) / DT,
                             wrap_angle(a.heading - heading) / DT)

    def predict(self, steps):
        return [(a, a.extrapolate(steps, DT, speed, accel, yaw))
                for a, (speed, _, accel, yaw) in self.state.items()]


# --- Decision making --------------------------------------------------------
@dataclass
class Decision:
    mode: str                  # CRUISE, WAIT, YIELD, HOLD, BRAKE, ESCAPE or OFF
    steer: float
    throttle: float
    ttc: float = None          # predicted seconds until contact
    conflict_xy: tuple = None
    ego_path: list = field(default_factory=list)
    other_paths: list = field(default_factory=list)


class SafetyController:
    def __init__(self, ego, plan, steps, margin):
        self.ego, self.plan, self.steps, self.margin = ego, plan, steps, margin
        self.clear_count = CLEAR_TICKS

    def _path(self, control):
        return self.ego.predict(self.steps, DT, control)

    def decide(self, tick, other_paths):
        ego, margin = self.ego, self.margin
        steer, throttle = self.plan.control(tick)
        ego_path = self._path(lambda k: self.plan.control(tick + k))   # the ego knows its own plan
        d = Decision('CRUISE', steer, throttle, ego_path=ego_path, other_paths=other_paths)
        hit = first_conflict(ego, ego_path, other_paths, margin)
        if hit is None:
            self.clear_count += 1
            if self.clear_count < CLEAR_TICKS:                         # don't jump straight back
                d.mode, d.throttle = 'WAIT', min(throttle, 0.)
            return d

        k = hit[0]
        d.ttc, d.conflict_xy = (k + 1) * DT, ego_path[k][:2]
        self.clear_count = 0
        # If even braking hard can't avoid it, compare stopping with speeding
        # clear and take whichever avoids (or delays) contact the longest.
        brake_hit = first_conflict(ego, self._path(lambda _: (steer, ego.friction - MAX_BRAKE)),
                                   other_paths, margin)
        if brake_hit is not None:
            go_hit = first_conflict(ego, self._path(lambda _: (steer, MAX_ACCEL)), other_paths, margin)
            if go_hit is None or go_hit[0] > brake_hit[0]:
                d.mode, d.throttle = 'ESCAPE', MAX_ACCEL
            else:
                d.mode, d.throttle = 'BRAKE', ego.friction - MAX_BRAKE
        elif ego.speed < 0.05:
            d.mode, d.throttle = 'HOLD', -1.                           # stay parked
        else:                                                          # stop STOP_BUFFER short of contact
            room = sum(step for *_, step in ego_path[:k]) - STOP_BUFFER
            decel = min(ego.speed ** 2 / (2 * max(room, 0.05)), MAX_BRAKE)
            d.mode, d.throttle = 'YIELD', min(throttle, ego.friction - decel)
        return d


# --- Simulation -------------------------------------------------------------
@dataclass
class Stats:
    ego_collision_ticks: int = 0
    first_ego_collision_t: float = None
    other_collision_ticks: int = 0
    min_gap: float = math.inf
    cleared_t: float = None
    yield_ticks: int = 0
    max_decel: float = 0.


class Simulation:
    def __init__(self, rng=None, safety=True, horizon_s=HORIZON_S, margin=SAFETY_MARGIN,
                 max_time=MAX_TIME, headless=True):
        self.world, self.ego, self.plan, self.drivers = build_world(rng, headless)
        self.others = [a for a, _ in self.drivers]
        self.steps = max(1, round(horizon_s / DT))
        self.safety, self.max_time = safety, max_time
        self.controller = SafetyController(self.ego, self.plan, self.steps, margin)
        self.observer = Observer(self.others)
        self.tick, self.stats, self.decision = 0, Stats(), None

    @property
    def t(self):
        return self.tick * DT

    @property
    def done(self):
        return self.t >= self.max_time or self.ego.y > SIZE + 5

    def step(self):
        for agent, driver in self.drivers:
            agent.set_control(*driver.control(self.tick, agent))
        other_paths = self.observer.predict(self.steps)
        if self.safety:
            d = self.controller.decide(self.tick, other_paths)
        else:
            d = Decision('OFF', *self.plan.control(self.tick), other_paths=other_paths,
                         ego_path=self.ego.predict(self.steps, DT, lambda k: self.plan.control(self.tick + k)))
        self.decision = d
        self.ego.set_control(d.steer, d.throttle)
        speed_before = self.ego.speed
        self.world.tick()
        self.observer.update()
        self.tick += 1

        st = self.stats
        st.yield_ticks += d.mode in ('YIELD', 'HOLD', 'WAIT', 'BRAKE')
        st.max_decel = max(st.max_decel, (speed_before - self.ego.speed) / DT)
        if st.cleared_t is None and self.ego.y > CLEARED_Y:
            st.cleared_t = self.t
        st.min_gap = min([st.min_gap] + [float(self.ego.distanceTo(o)) for o in self.others])
        if self.world.collision_exists(self.ego):
            st.ego_collision_ticks += 1
            st.first_ego_collision_t = st.first_ego_collision_t or self.t
        st.other_collision_ticks += any(self.world.collision_exists(a, ignore=[self.ego]) for a in self.others)

    def run(self):
        while not self.done:
            self.step()
        return self.stats

    def summary(self):
        st = self.stats
        crash = (f"YES, first contact at {st.first_ego_collision_t:.1f} s "
                 f"({st.ego_collision_ticks} ticks touching)" if st.ego_collision_ticks else "no")
        return [f"Safety layer: {'on' if self.safety else 'off'}",
                f"Ego car crashed: {crash}",
                f"Closest gap to another road user: {st.min_gap:.2f} m",
                f"Made it through the intersection: "
                + (f"{st.cleared_t:.1f} s" if st.cleared_t is not None else "did not make it through"),
                f"Time spent yielding or waiting: {st.yield_ticks * DT:.1f} s",
                f"Hardest braking: {st.max_decel:.1f} m/s^2",
                f"Other road users hitting something: {st.other_collision_ticks} ticks"]


# --- Window -----------------------------------------------------------------
def draw(sim):
    sim.world.render(flush=False)
    v, d, st = sim.world.visualizer, sim.decision, sim.stats
    v.path('ego', [(sim.ego.x, sim.ego.y)] + [p[:2] for p in d.ego_path], MODE_COLORS[d.mode], width=3)
    for a, path in d.other_paths:
        v.path(a, [(a.x, a.y)] + [p[:2] for p in path], 'gold', dash=(3, 3))
    v.marker('conflict', d.conflict_xy)
    v.text('hud', 160, 14, f"t = {sim.t:5.1f} s    red car {sim.ego.speed:4.1f} m/s    {d.mode}\n"
                           f"time to predicted contact: {f'{d.ttc:.1f} s' if d.ttc else '-'}\n"
                           f"closest gap so far: {st.min_gap:5.2f} m\n"
                           f"red car collisions: {st.ego_collision_ticks} ticks")
    v.text('legend', 160, 690, 'thick line = red car predicted path, dashed = others predicted',
           size=11, color='gray85')
    v.flush()


def run_window(sim, speed=1.0, close_when_done=False):
    from tkinter import TclError
    period, next_frame = DT / max(speed, 0.01), time.perf_counter()
    try:
        while not sim.done:
            sim.step()
            draw(sim)
            if sim.world.visualizer.closed:
                return
            next_frame += period
            delay = next_frame - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                next_frame = time.perf_counter()        # running behind: don't try to catch up
        v = sim.world.visualizer
        v.banner(sim.summary() + ['', 'Close this window to exit.'])
        if close_when_done:
            v.close()
        while not v.closed:                              # keep the result on screen
            v.flush()
            time.sleep(0.05)
    except TclError:
        pass                                             # window closed mid-frame


# --- Batch evaluation -------------------------------------------------------
def evaluate(runs, seed, horizon_s, margin):
    started = time.perf_counter()
    rows = {safety: [Simulation(random.Random(seed + i), safety, horizon_s, margin).run()
                     for i in range(runs)] for safety in (True, False)}
    avg = lambda vals: (sum(v for v in vals if v is not None) / max(1, sum(v is not None for v in vals)))
    print(f"{runs} randomized scenarios each (seeds {seed}-{seed + runs - 1}), horizon {horizon_s} s, "
          f"margin {margin} m, took {time.perf_counter() - started:.1f} s\n")
    print(f"{'':12}{'red car crashed':>17}{'closest gap':>13}{'made it through':>17}"
          f"{'avg time to clear':>19}{'avg yielding':>14}")
    for safety, res in rows.items():
        cleared = [s.cleared_t for s in res]
        print(f"{'safety on' if safety else 'safety off':12}"
              f"{f'{sum(1 for s in res if s.ego_collision_ticks)}/{runs}':>17}"
              f"{f'{avg([s.min_gap for s in res]):.2f} m':>13}"
              f"{f'{sum(c is not None for c in cleared)}/{runs}':>17}"
              f"{f'{avg(cleared):.1f} s':>19}"
              f"{f'{avg([s.yield_ticks * DT for s in res]):.1f} s':>14}")
    other = sum(1 for s in rows[True] if s.other_collision_ticks)
    if other:
        print(f"\nNote: in {other} runs another road user hit something on its own.")


def main():
    p = argparse.ArgumentParser(description='CARLO intersection with a predictive safety layer.')
    p.add_argument('--speed', type=float, default=1.0, help='playback speed (2 = twice real time)')
    p.add_argument('--no-safety', action='store_true', help='ego car ignores everyone (baseline)')
    p.add_argument('--seed', type=int, help='randomize the scene with this seed')
    p.add_argument('--headless', action='store_true', help='no window, just print the result')
    p.add_argument('--eval', type=int, metavar='N', help='run N randomized scenes, safety on vs off')
    p.add_argument('--horizon', type=float, default=HORIZON_S, help='prediction horizon in seconds')
    p.add_argument('--margin', type=float, default=SAFETY_MARGIN, help='safety margin in meters')
    p.add_argument('--time', type=float, default=MAX_TIME, help='seconds to simulate')
    p.add_argument('--close-when-done', action='store_true', help='close the window at the end')
    args = p.parse_args()

    if args.eval:
        return evaluate(args.eval, args.seed or 0, args.horizon, args.margin)
    rng = random.Random(args.seed) if args.seed is not None else None
    sim = Simulation(rng, not args.no_safety, args.horizon, args.margin, args.time, headless=args.headless)
    started = time.perf_counter()
    if args.headless:
        sim.run()
    else:
        run_window(sim, args.speed, args.close_when_done)
    print('\n'.join(sim.summary()))
    if args.headless:
        print(f"({sim.tick} ticks in {time.perf_counter() - started:.2f} s)")


if __name__ == '__main__':
    main()
