from graphics import GraphWin
from entities import RectangleEntity, CircleEntity, RingEntity


class Visualizer:
    """Draws each agent once and afterwards only moves the agents that can
    move. The window is refreshed once per frame (flush), not after every shape.

    Besides agents it can draw overlays, identified by a key and updated in
    place on every call: path(), marker(), text() and banner()."""

    def __init__(self, width: float, height: float, ppm: int):
        # width (meters)
        # height (meters)
        # ppm is the number of pixels per meters
        self.ppm = ppm
        self.display_width, self.display_height = int(width*ppm), int(height*ppm)
        self.window_created = False
        self.items = {}     # agent -> canvas item
        self.overlays = {}  # key -> canvas item(s)

    def create_window(self, bg_color: str = 'gray80'):
        if not self.window_created or self.win.isClosed():
            self.win = GraphWin('CARLO', self.display_width, self.display_height, autoflush=False)
            self.win.setBackground(bg_color)
            self.window_created = True
            self.items, self.overlays = {}, {}

    @property
    def closed(self) -> bool:
        return not self.window_created or self.win.isClosed()

    def px(self, x: float, y: float):
        """World meters -> window pixels (y points up in the world, down on screen)."""
        return self.ppm * x, self.display_height - self.ppm * y

    def _coords(self, agent):
        if isinstance(agent, RectangleEntity):
            return [v for c in agent.corners for v in self.px(c.x, c.y)]
        if isinstance(agent, CircleEntity):
            r = agent.radius
        elif isinstance(agent, RingEntity):
            r = (agent.inner_radius + agent.outer_radius) / 2.
        else:
            raise NotImplementedError
        x, y = self.px(agent.center.x, agent.center.y)
        return [x - r * self.ppm, y - r * self.ppm, x + r * self.ppm, y + r * self.ppm]

    def _create(self, agent):
        coords = self._coords(agent)
        if isinstance(agent, RectangleEntity):
            return self.win.create_polygon(*coords, fill=agent.color, outline='black')
        if isinstance(agent, CircleEntity):
            return self.win.create_oval(*coords, fill=agent.color, outline='black')
        width = (agent.outer_radius - agent.inner_radius) * self.ppm # a ring is a thick circle outline
        return self.win.create_oval(*coords, fill='', outline=agent.color, width=width)

    def update_agents(self, agents: list):
        alive = set()
        for agent in agents:
            alive.add(agent)
            item = self.items.get(agent)
            if item is None:
                self.items[agent] = self._create(agent)
            elif agent.movable:
                self.win.coords(item, *self._coords(agent))
        for agent in [a for a in self.items if a not in alive]: # removed from the world
            self.win.delete(self.items.pop(agent))

    def path(self, key, points, color: str = 'gold', width: int = 1, dash=None):
        """A line through world points [(x, y), ...]."""
        coords = [v for x, y in points for v in self.px(x, y)]
        coords = coords if len(coords) >= 4 else [-10, -10, -10, -10]
        if key not in self.overlays:
            style = {'dash': dash} if dash else {}
            self.overlays[key] = self.win.create_line(*coords, fill=color, width=width, **style)
        else:
            self.win.coords(self.overlays[key], *coords)
            self.win.itemconfig(self.overlays[key], fill=color)

    def marker(self, key, center, radius: float = 1.5, color: str = 'red', width: int = 3):
        """A ring around a world point, or hidden when center is None."""
        if key not in self.overlays:
            self.overlays[key] = self.win.create_oval(0, 0, 0, 0, outline=color, width=width)
        item = self.overlays[key]
        if center is None:
            self.win.itemconfig(item, state='hidden')
            return
        x, y = self.px(*center)
        r = radius * self.ppm
        self.win.coords(item, x - r, y - r, x + r, y + r)
        self.win.itemconfig(item, state='normal', outline=color)

    def text(self, key, x: float, y: float, text: str, size: int = 13, color: str = 'white',
             background: str = 'black', anchor: str = 'nw'):
        """Text at window pixel position (x, y) on a box that fits it."""
        if key not in self.overlays:
            box = self.win.create_rectangle(0, 0, 0, 0, fill=background, outline='')
            label = self.win.create_text(x, y, anchor=anchor, fill=color, font=('Menlo', size))
            self.overlays[key] = (box, label)
        box, label = self.overlays[key]
        self.win.itemconfig(label, text=text)
        x1, y1, x2, y2 = self.win.bbox(label)
        self.win.coords(box, x1 - 6, y1 - 4, x2 + 6, y2 + 4)

    def banner(self, lines):
        """A message box in the middle of the window."""
        self.text('banner', self.display_width / 2, self.display_height / 2, '\n'.join(lines),
                  anchor='center')
        self.flush()

    def flush(self):
        self.win.update()

    def close(self):
        self.window_created = False
        self.win.close()
        self.items, self.overlays = {}, {}
