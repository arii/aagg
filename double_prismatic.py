from simulator import *

# link sizes
wr = 4
wl = 4
p = 5
width = 0.5


b2world = b2WorldInterface(True)

wall_l = b2world.world.CreateStaticBody(position=(-2*wl,0) )
wall_r = b2world.world.CreateStaticBody(position=( 2*wr,0) )

wall_l.CreatePolygonFixture(
        box=(wl, width),
        density = 1,
        friction = 1
        )

wall_r.CreatePolygonFixture(
        box=(wr, width),
        density = 1,
        friction = 1
        )


pris = b2world.world.CreateDynamicBody(position=(0,0))
pris.CreatePolygonFixture(
        box=(p, width),
        density = 1,
        friction = 1
        )

right_joint = b2world.world.CreatePrismaticJoint(
        bodyA = wall_r,
        bodyB = pris,
        anchor = (2*wr, 0),
        )

left_joint = b2world.world.CreatePrismaticJoint(
        bodyA = wall_l,
        bodyB = pris,
        anchor = (-2*wl,0),
        )

right_joint.limitEnabled = True
right_joint.limits  = (-2*wr,2*wr)


left_joint.limitEnabled = True
left_joint.limits  = (-2*wl,2*wl)


b2world.add_bodies([wall_r, wall_l, pris])
b2world.step()

def apply_control(t, tau_l, tau_r):
    for i in range(t):
        #TODO  apply impulse on left and right side by tau amounts
        b2world.step()
        import pdb; pdb.set_trace()


apply_control(100, 50, 0)
raw_input("done")
