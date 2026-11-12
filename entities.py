import copy
import math
from typing import Union
from geometry import Point, Rectangle, Circle, Ring, overlaps

TWO_PI = 2 * math.pi


def rect_corners(cx, cy, heading, w, h):
    """Corners of a w x h rectangle centered at (cx, cy), in order around it."""
    c, s = math.cos(heading), math.sin(heading)
    ux, uy = c * w / 2, s * w / 2         # half length along the heading
    vx, vy = -s * h / 2, c * h / 2        # half width across it
    return [(cx + ux + vx, cy + uy + vy), (cx - ux + vx, cy - uy + vy),
            (cx - ux - vx, cy - uy - vy), (cx + ux - vx, cy + uy - vy)]


class Entity:
    def __init__(self, center: Point, heading: float, movable: bool = True, friction: float = 0):
        self.center = center # this is x, y
        self.heading = heading
        self.movable = movable
        self.color = 'ghost white'
        self.collidable = True
        if movable:
            self.friction = friction
            self.velocity = Point(0,0) # this is xp, yp
            self.acceleration = 0 # this is vp (or speedp)
            self.angular_velocity = 0 # this is headingp
            self.inputSteering = 0
            self.inputAcceleration = 0
            self.max_speed = math.inf
            self.min_speed = 0

    @property
    def speed(self) -> float:
        return math.hypot(self.velocity.x, self.velocity.y) if self.movable else 0

    def set_control(self, inputSteering: float, inputAcceleration: float):
        self.inputSteering = inputSteering
        self.inputAcceleration = inputAcceleration

    @property
    def rear_dist(self) -> float: # distance between the rear wheels and the center of mass. This is needed to implement the kinematic bicycle model dynamics
        if isinstance(self, RectangleEntity):
            # only for this function, we assume
            # (i) the longer side of the rectangle is always the nominal direction of the car
            # (ii) the center of mass is the same as the geometric center of the RectangleEntity.
            return max(self.size.x, self.size.y) / 2.
        elif isinstance(self, CircleEntity):
            return self.radius
        elif isinstance(self, RingEntity):
            return (self.inner_radius + self.outer_radius) / 2.
        raise NotImplementedError

    def _step(self, x, y, heading, speed, steering, throttle, dt):
        """One step of the kinematic bicycle model. Used by both tick() and
        predict(), so predictions always match what really happens.

        Based on "Kinematic and Dynamic Vehicle Models for Autonomous Driving
        Control Design" by Kong, Pfeiffer, Schildbach and Borrelli. The center
        of mass is assumed to be the geometric center (lf == lr).
        Returns (x, y, heading, speed, distance moved)."""
        lr = self.rear_dist
        beta = math.atan(0.5 * math.tan(steering))
        new_speed = min(max(speed + (throttle - self.friction) * dt, self.min_speed), self.max_speed)
        new_heading = heading + ((speed + new_speed) / lr) * math.sin(beta) * dt / 2.
        angle = (heading + new_heading) / 2. + beta
        dist = (speed + new_speed) * dt / 2.
        return x + dist * math.cos(angle), y + dist * math.sin(angle), new_heading, new_speed, dist

    def tick(self, dt: float):
        if not self.movable:
            return
        speed = self.speed
        x, y, heading, new_speed, _ = self._step(self.center.x, self.center.y, self.heading, speed,
                                                 self.inputSteering, self.inputAcceleration, dt)
        self.center = Point(x, y)
        self.heading = heading % TWO_PI # wrap the heading angle between 0 and +2pi
        self.velocity = Point(new_speed * math.cos(heading), new_speed * math.sin(heading))
        self.acceleration = self.inputAcceleration - self.friction
        self.angular_velocity = speed * self.inputSteering
        self.buildGeometry()

    def predict(self, steps: int, dt: float, control=None):
        """Future poses over the next `steps` ticks, without moving the agent.
        control(k) gives (steering, throttle) for step k; by default the
        current controls are kept. Returns [(x, y, heading, distance), ...]."""
        x, y, heading, speed = self.center.x, self.center.y, float(self.heading), self.speed
        path = []
        for k in range(steps):
            steering, throttle = control(k) if control else (self.inputSteering, self.inputAcceleration)
            x, y, heading, speed, dist = self._step(x, y, heading, speed, steering, throttle, dt)
            heading %= TWO_PI
            path.append((x, y, heading, dist))
        return path

    def extrapolate(self, steps: int, dt: float, speed: float, accel: float = 0., yaw_rate: float = 0.):
        """Future poses from observed motion alone: keeps the given speed
        change and turning rate (useful for agents whose controls are unknown)."""
        x, y, heading = self.center.x, self.center.y, float(self.heading)
        path = []
        for _ in range(steps):
            speed = max(0., speed + accel * dt)
            heading += yaw_rate * dt
            x += speed * math.cos(heading) * dt
            y += speed * math.sin(heading) * dt
            path.append((x, y, heading, speed * dt))
        return path

    def sim_tick(self, st: float, w=None, steps: int = 400):
        """Predicted shapes over the next `steps` ticks with the current controls."""
        if self.movable:
            return [self.footprint(x, y, h) for x, y, h, _ in self.predict(steps, st)]

    def buildGeometry(self): # builds the obj
        raise NotImplementedError

    def footprint(self, x: float, y: float, heading: float, margin: float = 0.):
        """The agent's shape at another pose, grown by margin on every side."""
        raise NotImplementedError

    def bounding_radius(self, margin: float = 0.) -> float:
        """Radius of a circle around the center that contains the whole shape."""
        raise NotImplementedError

    def buildTempGeometry(self, temp_trajectory_data):
        center, heading = temp_trajectory_data
        return self.footprint(center.x, center.y, heading)

    def collidesWith(self, other: Union['Point','Entity']) -> bool:
        if isinstance(other, Entity):
            return self.obj.intersectsWith(other.obj)
        elif isinstance(other, Point):
            return self.obj.intersectsWith(other)
        raise NotImplementedError

    def distanceTo(self, other: Union['Point','Entity']) -> float:
        if isinstance(other, Entity):
            return self.obj.distanceTo(other.obj)
        elif isinstance(other, Point):
            return self.obj.distanceTo(other)
        raise NotImplementedError

    def copy(self):
        return copy.deepcopy(self)

    @property
    def x(self):
        return self.center.x

    @property
    def y(self):
        return self.center.y

    @property
    def xp(self):
        return self.velocity.x

    @property
    def yp(self):
        return self.velocity.y


class RectangleEntity(Entity):
    def __init__(self, center: Point, heading: float, size: Point, movable: bool = True, friction: float = 0):
        super(RectangleEntity, self).__init__(center, heading, movable, friction)
        self.size = size
        self.buildGeometry()

    @property
    def corners(self):
        return [Point(x, y) for x, y in rect_corners(self.center.x, self.center.y, self.heading,
                                                     self.size.x, self.size.y)]

    def buildGeometry(self):
        self.obj = Rectangle(*self.corners[:3])

    def footprint(self, x, y, heading, margin=0.):
        pts = rect_corners(x, y, heading, self.size.x + 2 * margin, self.size.y + 2 * margin)
        return Rectangle(*(Point(px, py) for px, py in pts[:3]))

    def bounding_radius(self, margin=0.):
        return 0.5 * math.hypot(self.size.x, self.size.y) + margin


class CircleEntity(Entity):
    def __init__(self, center: Point, heading: float, radius: float, movable: bool = True, friction: float = 0):
        super(CircleEntity, self).__init__(center, heading, movable, friction)
        self.radius = radius
        self.buildGeometry()

    def buildGeometry(self):
        self.obj = Circle(self.center, self.radius)

    def footprint(self, x, y, heading, margin=0.):
        return Circle(Point(x, y), self.radius + margin)

    def bounding_radius(self, margin=0.):
        return self.radius + margin


class RingEntity(Entity):
    def __init__(self, center: Point, heading: float, inner_radius: float, outer_radius: float, movable: bool = True, friction: float = 0):
        super(RingEntity, self).__init__(center, heading, movable, friction)
        self.inner_radius = inner_radius
        self.outer_radius = outer_radius
        self.buildGeometry()

    def buildGeometry(self):
        self.obj = Ring(self.center, self.inner_radius, self.outer_radius)

    def footprint(self, x, y, heading, margin=0.):
        return Ring(Point(x, y), max(self.inner_radius - margin, 0.), self.outer_radius + margin)

    def bounding_radius(self, margin=0.):
        return self.outer_radius + margin


def first_conflict(agent, path, other_paths, margin=0.):
    """First step where `agent`, following `path`, would overlap another agent
    following its own path. other_paths is [(other, path), ...], all paths as
    returned by predict() or extrapolate(). Returns (step, other) or None.
    A cheap bounding-circle test skips pairs that are clearly far apart."""
    r = agent.bounding_radius(margin)
    reach = {o: r + o.bounding_radius(margin) for o, _ in other_paths}
    for k, (x, y, heading, _) in enumerate(path):
        shape = None
        for other, other_path in other_paths:
            ox, oy, oh, _ = other_path[k]
            if (x - ox) ** 2 + (y - oy) ** 2 > reach[other] ** 2:
                continue
            shape = shape or agent.footprint(x, y, heading, margin)
            if overlaps(shape, other.footprint(ox, oy, oh, margin)):
                return k, other
    return None
