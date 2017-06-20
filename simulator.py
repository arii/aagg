""" For patrick.  If you like this and want to build off it I will comment it
"""
from Box2D import *
from Box2D.b2 import *
import numpy as np
import pygame

# this just makes pygame show what's going on
class guiWorld:
    def __init__(self, fps):
        self.SCREEN_WIDTH, self.SCREEN_HEIGHT = 640, 480
        self.TARGET_FPS = fps
        self.PPM = 20.0 # pixels per meter
        self.screen = pygame.display.set_mode((self.SCREEN_WIDTH, self.SCREEN_HEIGHT), 0, 32)
        pygame.display.set_caption('Patrick\'s simulator world!!')
        self.clock = pygame.time.Clock()
        self.screen_origin = b2Vec2(self.SCREEN_WIDTH/(2*self.PPM), self.SCREEN_HEIGHT/(self.PPM*2))
        self.colors = {
            b2_staticBody : (102,51,0,255), #brown
            b2_dynamicBody : (150,150,150,255)
            }

    def draw(self, bodies, bg_color=(64,64,64,0)):
    #def draw(self, bodies, bg_color=(0,0,0,0)):
        def my_draw_polygon(polygon, body, fixture):
            vertices=[(self.screen_origin + body.transform*v)*self.PPM for v in polygon.vertices]
            vertices=[(v[0], self.SCREEN_HEIGHT-v[1]) for v in vertices]
            color = self.colors[body.type]
            if body.userData == "hand":
                color = (0,0,0,0)

            pygame.draw.polygon(self.screen, color, vertices)
        def my_draw_circle(circle, body, fixture):
            position=(self.screen_origin + body.transform*circle.pos)*self.PPM
            position=(position[0], self.SCREEN_HEIGHT-position[1])
            pygame.draw.circle(self.screen, self.colors[body.type], [int(x) for x in
            position], int(circle.radius*self.PPM))

        b2PolygonShape.draw=my_draw_polygon
        b2CircleShape.draw=my_draw_circle
        # draw the world
        self.screen.fill(bg_color)
        self.clock.tick(self.TARGET_FPS) 
        for body in bodies:
            for fixture in body.fixtures:
                fixture.shape.draw(body,fixture)
        pygame.display.flip()

# this is the interface to pybox2d
class b2WorldInterface:
    def __init__(self, do_gui=True):
        self.world = b2World(gravity=(0.0,0.0), doSleep=True)
        self.do_gui = do_gui
        self.TARGET_FPS = 100
        self.TIME_STEP = 1.0/self.TARGET_FPS
        self.VEL_ITERS, self.POS_ITERS =10,10
        self.bodies = []

        if do_gui:
            self.gui_world  = guiWorld(self.TARGET_FPS)
            #raw_input()
        else:
            self.gui_world = None

    def initialize_gui(self):
        if self.gui_world == None:
            self.gui_world = guiWorld(self.TARGET_FPS)
        self.do_gui = True
    def stop_gui(self):
        self.do_gui = False

    def add_bodies(self, new_bodies):
        """ add a single b2Body or list of b2Bodies to the world"""
        if type(new_bodies) == list:
            self.bodies += new_bodies
        else:
            self.bodies.append(new_bodies)
    def step(self, show_display=True):
        self.world.Step(self.TIME_STEP, self.VEL_ITERS, self.POS_ITERS)
        if show_display and self.do_gui:
            self.gui_world.draw(self.bodies)

class end_effector:
    def __init__(self, b2world_interface, init_pos):
        world= b2world_interface.world
        hand_size = (.5,.5)
        self.hand = world.CreateDynamicBody(position=init_pos)
        self.hand.CreatePolygonFixture(
            box=hand_size,
            density = .1,
            friction = .1
            )
        self.hand.userData = "hand"
        b2world_interface.add_bodies(self.hand)

    def apply_wrench(self, force=(0,0), torque=0):
        self.hand.ApplyForce(force, self.hand.position,wake=True)
        if torque != 0:
            self.hand.ApplyTorque(torque, wake=True)

    def get_state(self, verbose=False):
        state = list(self.hand.position) + [ self.hand.angle] +  \
                list(self.hand.linearVelocity) + [self.hand.angularVelocity]
        if verbose:
            print_state = ["%.3f" % x for x in state]
            print "position, velocity: (%s), (%s) " % \
                ((", ").join(print_state[:3]), (", ").join(print_state[3:]) )
    
        return state

def make_thing(b2world_interface, thing_size, joint_type):
    world = b2world_interface.world
    base = world.CreateStaticBody(position = (0,0))
    link = world.CreateDynamicBody(position=(thing_size[0]/2.0, 0))
    link.CreatePolygonFixture(
            box=thing_size,
            density = .1,
            friction = .1
            )

    if joint_type == "revolute":
        constructor = world.CreateRevoluteJoint
    elif joint_type =="prismatic":
        constructor = world.CreatePrismaticJoint
    elif joint_type =="free":
        constructor = None
    else:
        raise Exception("%s is not a correct joint" % joint_type)
    
    if constructor is not None:
        joint = constructor(
                bodyA = base,
                bodyB = link,
                anchor = base.position,
                )

    b2world_interface.add_bodies([base,link])
    return link

def attach_hand_to_thing(b2world_interface, thing, robot):
    world = b2world_interface.world
    world.CreateWeldJoint(
            bodyA = robot.hand,
            bodyB = thing,
            anchor = robot.hand.position,
            )


def test(joint_type,  force):
    world = b2WorldInterface(True)
    simulation_steps = 300
    door_size = (5, 1)
    thing = make_thing(world, door_size, joint_type)
    robot = end_effector(world, (7.5,0))
    attach_hand_to_thing(world,thing, robot)


    for i in range(simulation_steps):
        robot.apply_wrench(force)
        world.step()
        if not i % 50:
            robot.get_state(verbose=True)

if __name__ == "__main__":
    left_push = (-1, 0)
    up_push = (0, 1)

    for joint in ["prismatic", "revolute", "free"]:

        print "\n%s left push" % joint
        test(joint, left_push)

        print "\n%s up push" % joint
        test(joint, up_push)

