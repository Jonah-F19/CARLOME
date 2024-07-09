import numpy as np
from geometry import Point, Rectangle, Circle, Ring
from typing import Union
import copy


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
            self.max_speed = np.inf
            self.min_speed = 0
    
    @property
    def speed(self) -> float:
        return self.velocity.norm(p = 2) if self.movable else 0
    
    def set_control(self, inputSteering: float, inputAcceleration: float):
        self.inputSteering = inputSteering
        self.inputAcceleration = inputAcceleration
    
    @property
    def rear_dist(self) -> float: # distance between the rear wheels and the center of mass. This is needed to implement the kinematic bicycle model dynamics
        if isinstance(self, RectangleEntity):
            # only for this function, we assume
            # (i) the longer side of the rectangle is always the nominal direction of the car
            # (ii) the center of mass is the same as the geometric center of the RectangleEntity.
            return np.maximum(self.size.x, self.size.y) / 2.
        elif isinstance(self, CircleEntity):
            return self.radius
        elif isinstance(self, RingEntity):
            return (self.inner_radius + self.outer_radius) / 2.
        raise NotImplementedError
    
    def sim_tick(self, st: float, w=None): #define simulate ticks, where it 
        if self.movable:
            speed = self.speed
            heading = self.heading
            center = self.center
            velocity = self.velocity
            acceleration = self.acceleration
            angular_velocity = self.angular_velocity
            sim_geometry_trace = []
            for i in range(400):

                # Kinematic bicycle model dynamics based on, Another model to use: https://en.wikipedia.org/wiki/Ackermann_steering_geometry#Design_and_choice_of_geometry
                # "Kinematic and Dynamic Vehicle Models for Autonomous Driving Control Design" by
                # Jason Kong, Mark Pfeiffer, Georg Schildbach, Francesco Borrelli
                lr = self.rear_dist
                lf = lr # we assume the center of mass is the same as the geometric center of the entity
                beta = np.arctan(lr / (lf + lr) * np.tan(self.inputSteering))
                
                new_angular_velocity = speed * self.inputSteering # this is not needed and used for this model, but let's keep it for consistency (and to avoid if-else statements)
                new_acceleration = self.inputAcceleration - self.friction
                new_speed = np.clip(speed + new_acceleration * st, self.min_speed, self.max_speed)
                new_heading = heading + ((speed + new_speed)/lr)*np.sin(beta)*st/2.
                angle = (heading + new_heading)/2. + beta
                new_center = self.center + (speed + new_speed)*Point(np.cos(angle), np.sin(angle))*st / 2.
                new_velocity = Point(new_speed * np.cos(new_heading), new_speed * np.sin(new_heading))

                
                center = new_center
                heading = np.mod(new_heading, 2*np.pi) # wrap the heading angle between 0 and +2pi
                #sim_speed.append(new_velocity)
                acceleration = new_acceleration
                angular_velocity = new_angular_velocity
                
                # if w.collision_exists():
                #     return True, i

                '''
                # Point-mass dynamics based on
                # "Active Preference-Based Learning of Reward Functions" by
                # Dorsa Sadigh, Anca D. Dragan, S. Shankar Sastry, Sanjit A. Seshia
                
                new_angular_velocity = speed * self.inputSteering
                new_acceleration = self.inputAcceleration - self.friction * speed
                
                new_heading = heading + (self.angular_velocity + new_angular_velocity) * dt / 2.
                new_speed = np.clip(speed + (self.acceleration + new_acceleration) * dt / 2., self.min_speed, self.max_speed)
                
                new_velocity = Point(((speed + new_speed) / 2.) * np.cos((new_heading + heading) / 2.),
                                        ((speed + new_speed) / 2.) * np.sin((new_heading + heading) / 2.))
                
                new_center = self.center + (self.velocity + new_velocity) * dt / 2.
                
                '''
                # append goemetry instead
                temp_geo = self.buildTempGeometry((new_center, new_heading))
                sim_geometry_trace.append(temp_geo)

            return sim_geometry_trace
    #def stop():

    def tick(self, dt: float): #define simulate ticks, where it 
        if self.movable:
            speed = self.speed
            heading = self.heading
        
            # Kinematic bicycle model dynamics based on, Another model to use: https://en.wikipedia.org/wiki/Ackermann_steering_geometry#Design_and_choice_of_geometry
            # "Kinematic and Dynamic Vehicle Models for Autonomous Driving Control Design" by
            # Jason Kong, Mark Pfeiffer, Georg Schildbach, Francesco Borrelli
            lr = self.rear_dist
            lf = lr # we assume the center of mass is the same as the geometric center of the entity
            beta = np.arctan(lr / (lf + lr) * np.tan(self.inputSteering))
            
            new_angular_velocity = speed * self.inputSteering # this is not needed and used for this model, but let's keep it for consistency (and to avoid if-else statements)
            new_acceleration = self.inputAcceleration - self.friction
            new_speed = np.clip(speed + new_acceleration * dt, self.min_speed, self.max_speed)
            new_heading = heading + ((speed + new_speed)/lr)*np.sin(beta)*dt/2.
            angle = (heading + new_heading)/2. + beta
            new_center = self.center + (speed + new_speed)*Point(np.cos(angle), np.sin(angle))*dt / 2.
            new_velocity = Point(new_speed * np.cos(new_heading), new_speed * np.sin(new_heading))
            
            '''
            # Point-mass dynamics based on
            # "Active Preference-Based Learning of Reward Functions" by
            # Dorsa Sadigh, Anca D. Dragan, S. Shankar Sastry, Sanjit A. Seshia
            
            new_angular_velocity = speed * self.inputSteering
            new_acceleration = self.inputAcceleration - self.friction * speed
            
            new_heading = heading + (self.angular_velocity + new_angular_velocity) * dt / 2.
            new_speed = np.clip(speed + (self.acceleration + new_acceleration) * dt / 2., self.min_speed, self.max_speed)
            
            new_velocity = Point(((speed + new_speed) / 2.) * np.cos((new_heading + heading) / 2.),
                                    ((speed + new_speed) / 2.) * np.sin((new_heading + heading) / 2.))
            
            new_center = self.center + (self.velocity + new_velocity) * dt / 2.
            
            '''
            
            self.center = new_center
            self.heading = np.mod(new_heading, 2*np.pi) # wrap the heading angle between 0 and +2pi
            self.velocity = new_velocity
            self.acceleration = new_acceleration
            self.angular_velocity = new_angular_velocity
            
            self.buildGeometry()
    
    def buildGeometry(self): # builds the obj
        raise NotImplementedError
    
    def buildTempGeometry(self, temp_trajectory_data):
        raise NotImplementedError
        
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
    def edge_centers(self):
        edge_centers = np.zeros((4,2), dtype=np.float32)
        x = self.center.x
        y = self.center.y
        w = self.size.x
        h = self.size.y
        edge_centers[0] = [x + w / 2. * np.cos(self.heading), y + w / 2. * np.sin(self.heading)]
        edge_centers[1] = [x - h / 2. * np.sin(self.heading), y + h / 2. * np.cos(self.heading)]
        edge_centers[2] = [x - w / 2. * np.cos(self.heading), y - w / 2. * np.sin(self.heading)]
        edge_centers[3] = [x + h / 2. * np.sin(self.heading), y - h / 2. * np.cos(self.heading)]
        return edge_centers
        
    @property
    def corners(self):
        ec = self.edge_centers
        c = np.array([self.center.x, self.center.y])
        corners = []
        corners.append(Point(*(ec[1] + ec[0] - c)))
        corners.append(Point(*(ec[2] + ec[1] - c)))
        corners.append(Point(*(ec[3] + ec[2] - c)))
        corners.append(Point(*(ec[0] + ec[3] - c)))
        return corners
        
    def buildGeometry(self):
        C = self.corners
        self.obj = Rectangle(*C[:-1])

    def buildTempGeometry(self, temp_trajectory_data):
        temp_center = temp_trajectory_data[0]
        temp_heading = temp_trajectory_data[1]
        temp_edge_centers = np.zeros((4,2), dtype=np.float32)
        x = temp_center.x
        y = temp_center.y
        w = self.size.x
        h = self.size.y
        temp_edge_centers[0] = [x + w / 2. * np.cos(temp_heading), y + w / 2. * np.sin(temp_heading)]
        temp_edge_centers[1] = [x - h / 2. * np.sin(temp_heading), y + h / 2. * np.cos(temp_heading)]
        temp_edge_centers[2] = [x - w / 2. * np.cos(temp_heading), y - w / 2. * np.sin(temp_heading)]
        temp_edge_centers[3] = [x + h / 2. * np.sin(temp_heading), y - h / 2. * np.cos(temp_heading)]
        
        ec = temp_edge_centers
        c = np.array([temp_center.x, temp_center.y])
        corners = []
        corners.append(Point(*(ec[1] + ec[0] - c)))
        corners.append(Point(*(ec[2] + ec[1] - c)))
        corners.append(Point(*(ec[3] + ec[2] - c)))
        corners.append(Point(*(ec[0] + ec[3] - c)))
        C = corners
        return Rectangle(*C[:-1])
        
class CircleEntity(Entity):
    def __init__(self, center: Point, heading: float, radius: float, movable: bool = True, friction: float = 0):
        super(CircleEntity, self).__init__(center, heading, movable, friction)
        self.radius = radius
        self.buildGeometry()
        
    def buildGeometry(self):
        self.obj = Circle(self.center, self.radius)

    def buildTempGeometry(self, temp_trajectory_data):
        temp_center = temp_trajectory_data[0]
        return Circle(temp_center, self.radius)
                    
class RingEntity(Entity):
    def __init__(self, center: Point, heading: float, inner_radius: float, outer_radius: float, movable: bool = True, friction: float = 0):
        super(RingEntity, self).__init__(center, heading, movable, friction)
        self.inner_radius = inner_radius
        self.outer_radius = outer_radius
        self.buildGeometry()
        
    def buildGeometry(self):
        self.obj = Ring(self.center, self.inner_radius, self.outer_radius)

    def buildTempGeometry(self, temp_trajectory_data):
        temp_center = temp_trajectory_data[0]
        return Ring(temp_center, self.inner_radius, self.outer_radius)
    