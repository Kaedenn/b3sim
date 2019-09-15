#!/usr/bin/env python

"""
Various simulation scenarios. More than one scenario can be applied at once.
"""

import math
import os
import random
import sys

from pyb3 import pybullet as p
from utility import *

class ScenarioBase(object): # {{{0
  """Scenario base class

  All scenarios must define (at least) the following:
    __init__(self)
    setup(self, app)

  __init__ must call the superclass __init__:
    super(MyNewScenario, self).__init__(self, name=None, **kwargs)

  setup(self, app) must call the superclass setup:
    objs = super(MyNewScenario, self).setup(self, app)

  setup(self, app) must return a list of objects created:
    return objs
  """
  def __init__(self, name, **kwargs):
    self._name = name
    self._args = kwargs

  @property
  def name(self):
    "Scenario name"
    return self._name

  @property
  def args(self):
    "Scenario arguments (as passed to __init__ **kwargs)"
    return self._args

  def get(self, k, default=None, typeclass=None):
    "Get a value from self.args"
    val = self._args.get(k, default)
    if typeclass is not None:
      return typeclass(val)
    return val

  def set(self, k, v):
    "Set a key to a value in self.args"
    self._args[k] = v

  def has(self, k):
    "Determine if self.args has a given key"
    return k in self._args

  def _applyTransforms(self, app, objs):
    "Apply requested transformations on the objects"
    if self.get("rand"):
      # Adjust velocities by +/- 1%
      for oid in objs:
        velOld = app.getBodyVelocity(oid)[0]
        vel = (
          velOld[0] + (random.random() - 0.5) * (velOld[0] / 100),
          velOld[1] + (random.random() - 0.5) * (velOld[1] / 100),
          velOld[2]
        )
        p.resetBaseVelocity(oid, linearVelocity=vel)
    return objs

  def setup(self, app):
    "(abstract) Create bodies and return the bodies created"
    return []

  def __repr__(self):
    "Convert scenario to a string for printing"
    return "Scenario({!r}, {!r})".format(self._name, self._args)
# 0}}}

def _name(obj, name): # {{{0
  "Determine what name to use for obj"
  if name is None:
    return obj.__class__.__name__.replace("Scenario", "")
  return name
# 0}}}

class EmptyPlaneScenario(ScenarioBase): # {{{0
  def __init__(self, name=None, **kwargs):
    super(EmptyPlaneScenario, self).__init__(_name(self, name), **kwargs)
  def setup(self, app):
    objs = super(EmptyPlaneScenario, self).setup(app)
    wt = np.float(app.wallThickness())
    ws = np.float(app.worldSize())
    exts = [app.worldSizeMax()*ws+wt, app.worldSizeMax()*ws+wt, wt]
    b = app.createBox(mass=0, pos=app.worldFloor(), exts=exts, rgba=C_NONE, restitution=1)
    objs.append(b)
    return self._applyTransforms(app, objs)
# 0}}}

class BoxedScenario(ScenarioBase): # {{{0
  """
  Create walls around the simulation objects

  Walls enclose a cube with side length self._ws centered around 0, 0, with
  the bottom face co-planar to the world plane.

  Available interior area is exactly
    -(self._ws - self._wt) < {x,y,z} < self._ws - self._wt
  """
  def __init__(self, name=None, **kwargs):
    super(BoxedScenario, self).__init__(_name(self, name), **kwargs)
  def setup(self, app):
    objs = super(BoxedScenario, self).setup(app)
    a = app.getSlider("wallAlpha")
    wt = np.float(app.wallThickness())
    ws = np.float(app.worldSize())
    wcx = np.array([wt, ws+wt, ws+wt])
    wcy = np.array([ws+wt, wt, ws+wt])
    wcz = np.array([ws+wt, ws+wt, wt])
    if app.hasArg("bouncy"):
      kw = {"restitution": 1}
    else:
      kw = {"restitution": 0}
    walls = [
      app.createBox(mass=0, pos=ws * AXIS_X, exts=wcx, rgba=withAlpha(C3_RED, a), **kw),
      app.createBox(mass=0, pos=-ws * AXIS_X, exts=wcx, rgba=withAlpha(C3_BLU, a), **kw),
      app.createBox(mass=0, pos=ws * AXIS_Y, exts=wcy, rgba=withAlpha(C3_GRN, a), **kw),
      app.createBox(mass=0, pos=-ws * AXIS_Y, exts=wcy, rgba=withAlpha(C3_MAG, a), **kw),
      app.createBox(mass=0, pos=ws * AXIS_Z, exts=wcz, rgba=withAlpha(C3_GRN, a), **kw),
      app.createBox(mass=0, pos=-ws * AXIS_Z, exts=wcz, rgba=withAlpha(C3_BLU, a), **kw),
    ]
    objs.extend(walls)
    return self._applyTransforms(app, objs)
# 0}}}

class BrickTower1Scenario(ScenarioBase): # {{{0
  """
  Add bricks in an alternating tower
  """
  def __init__(self, name=None, **kwargs):
    super(BrickTower1Scenario, self).__init__(_name(self, name), **kwargs)
  def setup(self, app):
    objs = super(BrickTower1Scenario, self).setup(app)
    n = int(round(app.numObjects() ** 0.5))
    layers = int(round(app.getSlider("numBrickLayers")))
    bw = app.brickWidth() * app.brickScale()
    bl = app.brickLength() * app.brickScale()
    bh = app.brickHeight() * app.brickScale()
    bd = app.brickDist() * app.brickScale()
    b1e = np.array([bw/2, bl/2, bh/2])
    b2e = np.array([bl/2, bw/2, bh/2])
    bx = lambda i: bd * (i + (1.0-n)/2)
    by = lambda i: bd * (i + (1.0-n)/2)
    bz = lambda i: i * bh
    boxes = []
    for i in range(n):
      for j in range(n):
        for k in range(layers):
          boxes.append((b1e, [bx(i), by(j), bz(2*k)]))
          boxes.append((b2e, [bx(i), by(j), bz(2*k+1)]))
    basePos = app.worldFloor()
    baseMass = app.objectMass()
    for bc, bp in boxes:
      objs.append(app.createBox(mass=baseMass, pos=bp+basePos, exts=bc))
    return self._applyTransforms(app, objs)
# 0}}}

class BrickTower2Scenario(ScenarioBase): # {{{0
  """
  Add pillars of bricks
  """
  def __init__(self, name=None, **kwargs):
    super(BrickTower2Scenario, self).__init__(_name(self, name), **kwargs)
  def setup(self, app):
    objs = super(BrickTower2Scenario, self).setup(app)
    n = int(round(app.numObjects() ** 0.5))
    layers = int(round(app.getSlider("numBrickLayers")))
    bw = app.brickWidth() * app.brickScale() / 2
    bl = app.brickLength() * app.brickScale() / 2
    bs = min(bw, bl)
    bh = app.brickHeight() * app.brickScale() * 2
    bd = bs * 3
    bext = np.array([bs, bs, bh/2])
    bx = lambda i: bd * (i + (1.0-n)/2)
    by = lambda i: bd * (i + (1.0-n)/2)
    bz = lambda i: i * bh
    boxes = []
    for i in range(n):
      for j in range(n):
        for k in range(2 * layers):
          boxes.append((bext, [bx(i), by(j), bz(k)]))
    basePos = app.worldFloor()
    baseMass = app.objectMass()
    for bc, bp in boxes:
      objs.append(app.createBox(mass=baseMass, pos=bp+basePos, exts=bc))
    return self._applyTransforms(app, objs)
# 0}}}

class BrickTower3Scenario(ScenarioBase): # {{{0
  """
  Create a brick tower with reinforcing plates
  """
  def __init__(self, name=None, **kwargs):
    super(BrickTower3Scenario, self).__init__(_name(self, name), **kwargs)
  def setup(self, app):
    objs = super(BrickTower3Scenario, self).setup(app)
    n = int(round(app.numObjects() ** 0.5))
    layers = int(round(app.getSlider("numBrickLayers")))
    bw = app.brickWidth() * app.brickScale()
    bl = app.brickLength() * app.brickScale()
    bh = app.brickHeight() * app.brickScale()
    bd = app.brickDist() * app.brickScale()
    b1e = np.array([bw/2, bl/2, bh/2])
    b2e = np.array([bl/2, bw/2, bh/2])
    b3e = np.array([n*bd/2, n*bd/2, bh/8])
    bx = lambda i: bd * (i + (1.0-n)/2)
    by = lambda i: bd * (i + (1.0-n)/2)
    bz = lambda i: i * bh
    boxes = []
    for k in range(layers):
      for i in range(n):
        for j in range(n):
          boxes.append((b1e, [bx(i), by(j), bz(3*k)]))
          boxes.append((b2e, [bx(i), by(j), bz(3*k+1)]))
      boxes.append((b3e, [0, 0, bz(3*k+2)]))
    basePos = app.worldFloor()
    baseMass = app.objectMass()
    for bc, bp in boxes:
      objs.append(app.createBox(mass=baseMass, pos=bp+basePos, exts=bc))
    return self._applyTransforms(app, objs)
# 0}}}

class ProjectileScenario(ScenarioBase): # {{{0
  """
  Fire projectiles at the origin
  """
  SHAPE_SPHERE = 1
  SHAPE_BOX = 2
  SHAPE_CAPSULE = 3

  def __init__(self, name=None, **kwargs):
    super(ProjectileScenario, self).__init__(_name(self, name), **kwargs)
    self._vel = self.get("velCoeff", 20, float)
    self._shape = self.get("projShape", ProjectileScenario.SHAPE_SPHERE)

  def _make(self, app, x, y, z):
    spos = app.worldFloor() + np.array([x, y, app.worldSize()/4 + z])
    svel = self._vel * -unit(np.array([spos[0], spos[1], 0]))
    kw = {}
    if app.hasArg("bouncy"):
      kw["restitution"] = 1
    mass = app.objectMass()*100
    size = self.get("projSize", app.objectRadius()/2, float)
    if self._shape == ProjectileScenario.SHAPE_SPHERE:
      s = app.createSphere(mass, spos, radius=size, **kw)
    elif self._shape == ProjectileScenario.SHAPE_BOX:
      s = app.createBox(mass=mass, exts=size*np.array([1,1,1]), pos=spos, **kw)
    elif self._shape == ProjectileScenario.SHAPE_CAPSULE:
      s = app.createCapsule(mass=mass, pos=spos, radii=(size, size/4), **kw)
    else:
      sys.stderr.write("Unknown shape {}; using spheres".format(self._shape))
      s = app.createSphere(mass, spos, radius=size, **kw)
    p.resetBaseVelocity(s, linearVelocity=svel)
    return s

  def setup(self, app):
    objs = super(ProjectileScenario, self).setup(app)
    ws = app.worldSize()
    wsq = ws / 4.0
    objs.append(self._make(app,  3*wsq,  0, 0))
    objs.append(self._make(app, -3*wsq,  0, wsq))
    objs.append(self._make(app,  3*wsq, -3*wsq, 2*wsq))
    objs.append(self._make(app, -3*wsq, -3*wsq, 3*wsq))
    return self._applyTransforms(app, objs)
# 0}}}

class BallpitScenario(ScenarioBase): # {{{0
  """
  Create what looks like a ball pit
  """
  def __init__(self, name=None, **kwargs):
    super(BallpitScenario, self).__init__(_name(self, name), **kwargs)
  def setup(self, app):
    objs = super(BallpitScenario, self).setup(app)
    floor = app.worldFloor()
    offset = floor + np.array([0, 0, app.getSlider("sphereZOffset")])
    wmin, wmax = app.worldXMin(), app.worldXMax()
    ymin, ymax = app.worldYMin(), app.worldYMax()
    zmin, zmax = app.worldZMin(), app.worldZMax()
    for i in range(app.numObjects()):
      spos = np.array([
        random.uniform(wmin, wmax),
        random.uniform(wmin, wmax),
        random.uniform(zmin, zmax)])
      s = app.createSphere(app.objectMass(), spos, app.objectRadius(), restitution=1)
      objs.append(s)
    return self._applyTransforms(app, objs)
# 0}}}

# XXX: Doesn't quite work; the created bunny is a rigid body.
class BunnyScenario(ScenarioBase): # {{{0
  """
  Load a soft body bunny into the world

  bunnyZ: height of the bunny above the world floor (5)
  bunnySize: bunny scale (2)
  bunnyMass: bunny mass (app.objectMass() * 10)
  """
  def __init__(self, name=None, **kwargs):
    super(BunnyScenario, self).__init__(_name(self, name), **kwargs)
  def setup(self, app):
    objs = super(BunnyScenario, self).setup(app)
    pos = app.worldFloor() + np.array([0, 0, self.get("bunnyZ", 5, float)])
    orn = p.getQuaternionFromEuler([math.pi/2, 0, 0])
    scale = self.get("bunnySize", 2, float)
    mass = self.get("bunnyMass", app.objectMass() * 10, float)
    b = app.loadSoftBody("data/bunny.obj", pos=pos, orn=orn, scale=scale, mass=mass)
    objs.append(b)
    return self._applyTransforms(app, objs)
# 0}}}

