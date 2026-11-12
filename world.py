from entities import Entity


class World:
    def __init__(self, dt: float, width: float, height: float, ppm: float = 8, headless: bool = False):
        self.dynamic_agents = []
        self.static_agents = []
        self.t = 0 # simulation time
        self.dt = dt # simulation time step
        self.width, self.height, self.ppm = width, height, ppm
        self.headless = headless # True: never open a window (fast batch runs)
        self.visualizer = None # created on the first render()

    def add(self, entity: Entity):
        if entity.movable:
            self.dynamic_agents.append(entity)
        else:
            self.static_agents.append(entity)

    def remove(self, entity: Entity):
        if entity.movable:
            self.dynamic_agents.remove(entity)
        else:
            self.static_agents.remove(entity)

    def tick(self):
        for agent in self.dynamic_agents:
            agent.tick(self.dt)
        self.t += self.dt

    def sim_tick(self, agent=None, steps: int = 400):
        """Predicted shapes of every moving agent over the next `steps` ticks,
        assuming each keeps its current controls: {agent: [shape, ...]}."""
        return {a: a.sim_tick(self.dt, steps=steps) for a in self.dynamic_agents}

    def render(self, flush: bool = True):
        if self.headless:
            return
        if self.visualizer is None:
            from visualizer import Visualizer # graphics.py opens Tk on import, so load it only when drawing
            self.visualizer = Visualizer(self.width, self.height, ppm=self.ppm)
        self.visualizer.create_window(bg_color='gray')
        self.visualizer.update_agents(self.agents)
        if flush:
            self.visualizer.flush()

    @property
    def agents(self):
        return self.static_agents + self.dynamic_agents

    @staticmethod
    def _touching(a, b) -> bool:
        if not (a.collidable and b.collidable):
            return False
        reach = a.bounding_radius() + b.bounding_radius() # skip the exact test when clearly apart
        if (a.x - b.x) ** 2 + (a.y - b.y) ** 2 > reach * reach:
            return False
        return a.collidesWith(b)

    def collision_exists(self, agent=None, ignore=()):
        """With no agent: is anything colliding anywhere? With an agent: is
        that agent colliding with anything (other than the agents in ignore)?"""
        if agent is not None:
            return any(self._touching(agent, o) for o in self.agents if o is not agent and o not in ignore)
        dynamic = self.dynamic_agents
        for i, a in enumerate(dynamic):
            if any(self._touching(a, b) for b in dynamic[i + 1:] + self.static_agents):
                return True
        return False

    def close(self):
        self.reset()
        self.static_agents = []
        if self.visualizer is not None and self.visualizer.window_created:
            self.visualizer.close()

    def reset(self):
        self.dynamic_agents = []
        self.t = 0
