import numpy as np
from world import World
from agents import Car, RectangleBuilding, Pedestrian, Painting
from geometry import Point
import time

human_controller = False
simcrash = False
dt = 0.1 # time steps in terms of seconds. In other words, 1/dt is the FPS.]
st = 0.1
w = World(dt, width = 120, height = 120, ppm = 6) # The world is 120 meters by 120 meters. ppm is the pixels per meter.

# Let's add some sidewalks and RectangleBuildings.
# A Painting object is a rectangle that the vehicles cannot collide with. So we use them for the sidewalks.
# A RectangleBuilding object is also static -- it does not move. But as opposed to Painting, it can be collided with.
# For both of these objects, we give the center point and the size.
w.add(Painting(Point(71.5, 106.5), Point(97, 27), 'gray80')) # We build a sidewalk.
w.add(RectangleBuilding(Point(72.5, 107.5), Point(95, 25))) # The RectangleBuilding is then on top of the sidewalk, with some margin.

# Let's repeat this for 4 different RectangleBuildings.
w.add(Painting(Point(8.5, 106.5), Point(17, 27), 'gray80'))
w.add(RectangleBuilding(Point(7.5, 107.5), Point(15, 25)))

w.add(Painting(Point(8.5, 41), Point(17, 82), 'gray80'))
w.add(RectangleBuilding(Point(7.5, 40), Point(15, 80)))

w.add(Painting(Point(71.5, 41), Point(97, 82), 'gray80'))
w.add(RectangleBuilding(Point(72.5, 40), Point(95, 80)))

# Let's also add some zebra crossings, because why not.
w.add(Painting(Point(18, 81), Point(0.5, 2), 'white'))
w.add(Painting(Point(19, 81), Point(0.5, 2), 'white'))
w.add(Painting(Point(20, 81), Point(0.5, 2), 'white'))
w.add(Painting(Point(21, 81), Point(0.5, 2), 'white'))
w.add(Painting(Point(22, 81), Point(0.5, 2), 'white'))

# A Car object is a dynamic object -- it can move. We construct it using its center location and heading angle.
c1sim = Car(Point(20,20), np.pi/2)
w.add(c1sim)

c2sim = Car(Point(118,90), np.pi, 'blue')
c2sim.velocity = Point(3.0,0) # We can also specify an initial velocity just like this.
w.add(c2sim)

# Pedestrian is almost the same as Car. It is a "circle" object rather than a rectangle.
p1sim = Pedestrian(Point(28,81), np.pi)
p1sim.max_speed = 10.0 # We can specify min_speed and max_speed of a Pedestrian (and of a Car). This is 10 m/s, almost Usain Bolt.
w.add(p1sim)


#sim = sim_tick(c1, dt, 10, w)
if not human_controller:
    # Let's implement some simple scenario with all agents
    
    early_crash = False
    crash = []

    
    w.render()

    p1sim.set_control(0, 0.22) # The pedestrian will have 0 steering and 0.22 throttle. So it will not change its direction.
    c1sim.set_control(0, 0.35)
    c2sim.set_control(0, 0.05)

    c1_sim_ticks = 0
    c2_sim_ticks = 0
    p1_sim_ticks = 0

    c1_ticks = 0
    c2_ticks = 0
    p1_ticks = 0

    old_control = None
    car1 = w.sim_tick(c1sim)
    car2 = w.sim_tick(c2sim)
    Ped1 = w.sim_tick(p1sim)

    while c1_sim_ticks != 400 and c2_sim_ticks != 400 and p1_sim_ticks != 400:

        # All movable objects will keep their control the same as long as we don't change it.
        if c1_sim_ticks == 100: # Let's say the first Car will release throttle (and start slowing down due to friction)
            c1sim.set_control(0, 0)
        elif c1_sim_ticks == 200: # The first Car starts pushing the brake a little bit. The second Car starts turning right with some throttle.
            c1sim.set_control(0, -0.02)
        elif c1_sim_ticks == 325:
            c1sim.set_control(0, 0.8)

        if old_control != None:
            c1sim.set_control(old_control[0], old_control[1])

        if c2_sim_ticks == 325:
            c2sim.set_control(-0.45, 0.3)
        elif c2_sim_ticks == 367: # The second Car stops turning.
            c2sim.set_control(0, 0.1)

        is_future_collision = False
        sim_geometry_trace = w.sim_tick(c1sim)
        for obj1, obj2 in zip(sim_geometry_trace[c1sim], sim_geometry_trace[c2sim]):
            if obj1.intersectsWith(obj2):
                is_future_collision = True

        for obj1, obj3 in zip(sim_geometry_trace[c1sim], sim_geometry_trace[p1sim]):
            if obj1.intersectsWith(obj3):
                is_future_collision = True
        # if car1.collidesWithSim(car2, sim_geometry_trace[car1], sim_geometry_trace[car2]) or car1[c1_sim_ticks].collidesWith(Ped1[c1_sim_ticks]):
        #     crash = []
        #     crash[c1_sim_ticks] = True
        #     is_future_collision = True

        if c1_sim_ticks <= 20 and is_future_collision:
            early_crash = True
        if is_future_collision:
            print("Oh crap your gonna crash")
            # print(f'stopping! crash at {tick} ticks in the future')
        else:
            old_control = None
            c1_sim_ticks += 1
        
        crash.append(is_future_collision)
        c2_sim_ticks += 1
        p1_sim_ticks += 1
        w.tick()
        time.sleep(dt/100)
        w.render()

        # if w.collision_exists(): # We can check if there is any collision at all.
        #     print('Collision exists somewhere...')
    
    w.render()

    w.remove(c1sim)
    w.remove(c2sim)
    w.remove(p1sim)

    c1 = Car(Point(20,20), np.pi/2)
    w.add(c1)

    c2 = Car(Point(118,90), np.pi, 'blue')
    c2.velocity = Point(3.0,0) # We can also specify an initial velocity just like this.
    w.add(c2)

    # Pedestrian is almost the same as Car. It is a "circle" object rather than a rectangle.
    p1 = Pedestrian(Point(28,81), np.pi)
    p1.max_speed = 10.0 # We can specify min_speed and max_speed of a Pedestrian (and of a Car). This is 10 m/s, almost Usain Bolt.
    w.add(p1)





    p1.set_control(0, 0.22) # The pedestrian will have 0 steering and 0.22 throttle. So it will not change its direction.
    c1.set_control(0, 0.35)
    c2.set_control(0, 0.05)
    checker = 0
    while c1_ticks != 400 and c2_ticks != 400 and p1_ticks != 400:

        # All movable objects will keep their control the same as long as we don't change it.

        #say something 20 ticks before collision happens slow down, or put it in the array itself, do like a -20
        cur_ticks = c1_ticks + 40
        for cur_ticks in range(400):
            b4_ticks = cur_ticks
        else:
            b4_ticks = c1_ticks
        if early_crash == True and c1_ticks <= 20:
             c1.set_control(0,-1.0)
        elif c1_ticks >= 20 and crash[b4_ticks] == True and checker < 10:
                print("slowing down")
                c1.set_control(0,-5.0)
                checker += 1
        else:
            if c1_ticks == 100: # Let's say the first Car will release throttle (and start slowing down due to friction)
                c1.set_control(0, 0)
            elif c1_ticks == 200: # The first Car starts pushing the brake a little bit. The second Car starts turning right with some throttle.
                c1.set_control(0, -0.02)
            elif c1_ticks == 325:
                c1.set_control(0, 0.8)

            if old_control != None:
                c1.set_control(old_control[0], old_control[1])

            if c2_ticks == 325:
                c2.set_control(-0.45, 0.3)
            elif c2_ticks == 367: # The second Car stops turning.
                c2.set_control(0, 0.1)

            tick = w.sim_tick(c1)
            c1_ticks += 1
            checker = 0
        
        c2_ticks += 1
        p1_ticks += 1

        w.tick()
        w.render()
        time.sleep(dt/10)

        if w.collision_exists(): # We can check if there is any collision at all.
            print('Collision exists somewhere...')

    w.close()

        # TODO: add graphics