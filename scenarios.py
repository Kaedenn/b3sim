#!/usr/bin/env python

"""
Various simulation scenarios. Multiple scenarios can be applied at once.
"""

import math
import os
import random
import sys

import pyb3
from pyb3 import pybullet as p
from utility import *

logger = pyb3.getLogger()

def _name(obj, name): # {{{0
  "Determine what name to use for obj"
  if name is None:
    return obj.__class__.__name__.replace("Scenario", "")
  return name
# 0}}}

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
    val = self.args.get(k, default)
    if typeclass is not None:
      return typeclass(val)
    return val

  def set(self, k, v):
    "Set a key to a value in self.args"
    self.args[k] = v

  def has(self, k):
    "Determine if self.args has a given key"
    return k in self.args

  def _applyTransforms(self, app, objs):
    "Apply requested transformations on the objects"
    if self.get("rand"):
      # Adjust velocities by +/- 1%
      for oid in objs:
        v0 = app.getBodyVelocity(oid)[0]
        if np.linalg.norm(v0) > 0:
          p.resetBaseVelocity(oid, linearVelocity=(
            v0[0] + (random.random() - 0.5) * (v0[0] / 100),
            v0[1] + (random.random() - 0.5) * (v0[1] / 100),
            v0[2]
          ))
    for oid in objs:
      dynamics = {}
      if self.get("bouncy"):
        # Set everything's restitution to 1
        dynamics["restitution"] = 1
      if self.get("frictionless"):
        # Make everything frictionless
        dynamics["lateralFriction"] = 0
        dynamics["rollingFriction"] = 0
        dynamics["spinningFriction"] = 0
      if dynamics:
        p.changeDynamics(oid, -1, **dynamics)
    return objs

  def setup(self, app):
    "(abstract) Create bodies and return the bodies created"
    return []

  def __call__(self, app):
    "Convenience alias for self.setup(app)"
    self.setup(app)

  def __repr__(self):
    "Convert scenario to a string for printing or serialization"
    return "Scenario({!r}, {!r})".format(self._name, self._args)
# 0}}}

class EmptyPlaneScenario(ScenarioBase): # {{{0
  "Create an empty plane"
  PLANE_THICKNESS = 0.1

  def __init__(self, name=None, **kwargs):
    super(EmptyPlaneScenario, self).__init__(_name(self, name), **kwargs)

  def setup(self, app):
    objs = super(EmptyPlaneScenario, self).setup(app)
    tex = None
    if app.hasArg("planeTexture"):
      tex = app.getArg("planeTexture")
    elif os.path.exists("data/white.png"):
      tex = "data/white.png"
    t = EmptyPlaneScenario.PLANE_THICKNESS
    pos = app.worldFloor() - V3(z=t)
    exts = V3(app.worldSizeMax(), app.worldSizeMax(), t)
    objs.append(app.createBox(mass=0, pos=pos, exts=exts, rgba=C4_WHT, texture=tex))
    return self._applyTransforms(app, objs)
# 0}}}

class BoxedScenario(ScenarioBase): # {{{0
  """
  Create walls around the simulation objects

  Walls enclose a cube with side length app._ws centered around (0, 0) with
  the bottom face co-planar to the world plane.

  Available interior area is exactly
    -(app._ws - app._wt) < {x,y} < app._ws - app._wt
    -app._ws < z
  """
  def __init__(self, name=None, **kwargs):
    super(BoxedScenario, self).__init__(_name(self, name), **kwargs)

  def setup(self, app):
    objs = super(BoxedScenario, self).setup(app)
    a = app.getSlider("Wall Opacity")
    wt = np.float(app.wallThickness())
    ws = np.float(app.worldSize())
    wcx = V3(wt, ws+wt, ws+wt)
    wcy = V3(ws+wt, wt, ws+wt)
    wcz = V3(ws+wt, ws+wt, wt)
    objs.extend([
      app.createBox(mass=0, pos= ws * V3_X, exts=wcx, rgba=withAlpha(C3_RED, a)),
      app.createBox(mass=0, pos=-ws * V3_X, exts=wcx, rgba=withAlpha(C3_BLU, a)),
      app.createBox(mass=0, pos= ws * V3_Y, exts=wcy, rgba=withAlpha(C3_GRN, a)),
      app.createBox(mass=0, pos=-ws * V3_Y, exts=wcy, rgba=withAlpha(C3_MAG, a)),
      app.createBox(mass=0, pos= ws * V3_Z, exts=wcz, rgba=withAlpha(C3_GRN, a)),
      app.createBox(mass=0, pos=-ws * V3_Z, exts=wcz, rgba=withAlpha(C3_BLU, a)),
    ])
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
    layers = int(round(app.getSlider("Brick Layers")))
    bw = app.brickWidth() * app.brickScale()
    bl = app.brickLength() * app.brickScale()
    bh = app.brickHeight() * app.brickScale()
    bd = app.brickDist() * app.brickScale()
    b1e = V3(bw/2, bl/2, bh/2)
    b2e = V3(bl/2, bw/2, bh/2)
    bx = lambda i: bd * (i + (1.0-n)/2)
    by = lambda i: bd * (i + (1.0-n)/2)
    bz = lambda i: bh * i + bh/2
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
    layers = int(round(app.getSlider("Brick Layers")))
    bw = app.brickWidth() * app.brickScale() / 2
    bl = app.brickLength() * app.brickScale() / 2
    bs = min(bw, bl)
    bh = app.brickHeight() * app.brickScale() * 2
    bd = bs * 3
    bext = V3(bs, bs, bh/2)
    bx = lambda i: bd * (i + (1.0-n)/2)
    by = lambda i: bd * (i + (1.0-n)/2)
    bz = lambda i: bh * i + bh/2
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
    layers = int(round(app.getSlider("Brick Layers")))
    bw = app.brickWidth() * app.brickScale()
    bl = app.brickLength() * app.brickScale()
    bh = app.brickHeight() * app.brickScale()
    bd = app.brickDist() * app.brickScale()
    b1e = V3(bw/2, bl/2, bh/2)
    b2e = V3(bl/2, bw/2, bh/2)
    b3e = V3(n*bd/2, n*bd/2, bh/8)
    bx = lambda i: bd * (i + (1.0-n)/2)
    by = lambda i: bd * (i + (1.0-n)/2)
    bz = lambda i: bh * i + bh/2
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

  velCoeff: velocity coefficient (20)
  projShape: shape ID (see table below)
  projMass: projectile mass (100)
  projTargetX: projectile target x (0)
  projTargetY: projectile target y (0)
  projTargetZ: projectile target z (0)
  projSize: proectile size (objectRadius()/2)

  Shape IDs:
    1   Sphere
    2   Box
    3   Capsule
    4   Cylinder
  """
  SHAPE_SPHERE = 1
  SHAPE_BOX = 2
  SHAPE_CAPSULE = 3
  SHAPE_CYLINDER = 4

  def __init__(self, name=None, **kwargs):
    super(ProjectileScenario, self).__init__(_name(self, name), **kwargs)
    self._vel = self.get("velCoeff", 20, float)
    self._shape = self.get("projShape", ProjectileScenario.SHAPE_SPHERE)
    self._massCoeff = self.get("projMass", 100, float)
    targetX = self.get("projTargetX", 0, float)
    targetY = self.get("projTargetY", 0, float)
    targetZ = self.get("projTargetZ", 0, float)
    self._target = V3(targetX, targetY, targetZ)

  def _make(self, app, x, y, z):
    spos = app.worldFloor() + V3(x, y, app.worldSize()/4 + z)
    svel = self._vel * unit(self._target - V3(spos[0], spos[1], 0))
    mass = app.objectMass() * self._massCoeff
    size = self.get("projSize", app.objectRadius()/2, float)
    if self._shape == ProjectileScenario.SHAPE_SPHERE:
      s = app.createSphere(mass, spos, radius=size)
    elif self._shape == ProjectileScenario.SHAPE_BOX:
      s = app.createBox(mass=mass, exts=size * V3_XYZ, pos=spos)
    elif self._shape == ProjectileScenario.SHAPE_CAPSULE:
      s = app.createCapsule(mass=mass, pos=spos, radii=(size, size/4))
    elif self._shape == ProjectileScenario.SHAPE_CYLINDER:
      s = app.createCylinder(mass=mass, pos=spos, radii=(size, size/4))
    else:
      sys.stderr.write("Unknown shape {}; using spheres".format(self._shape))
      s = app.createSphere(mass, spos, radius=size)
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

  objShape: shape ID (see ProjectileScenario)
  """
  def __init__(self, name=None, **kwargs):
    super(BallpitScenario, self).__init__(_name(self, name), **kwargs)
    self._shape = self.get("objShape", ProjectileScenario.SHAPE_SPHERE)

  def setup(self, app):
    objs = super(BallpitScenario, self).setup(app)
    mass = app.objectMass()
    rad = app.objectRadius()
    dx = app.numObjects() ** 0.5 / rad
    kws = {"restitution": 1}
    for i in range(app.numObjects()):
      spos = randomVec(-dx, dx)
      if self._shape == ProjectileScenario.SHAPE_SPHERE:
        s = app.createSphere(mass=mass, pos=spos, radius=rad, **kws)
      elif self._shape == ProjectileScenario.SHAPE_BOX:
        s = app.createBox(mass=mass, pos=spos, exts=rad * V3_XYZ, **kws)
      elif self._shape == ProjectileScenario.SHAPE_CAPSULE:
        s = app.createCapsule(mass=mass, pos=spos, radii=(rad, rad*4), **kws)
      elif self._shape == ProjectileScenario.SHAPE_CYLINDER:
        s = app.createCylinder(mass=mass, pos=spos, radii=(rad, rad*4), **kws)
      objs.append(s)
    return self._applyTransforms(app, objs)
# 0}}}

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
    # Create bunny
    kws = {
      "pos": app.worldFloor() + V3(z=self.get("bunnyZ", 5, float)),
      "orn": p.getQuaternionFromEuler((math.pi/2, 0, math.pi/2)),
      "scale": self.get("bunnySize", 10, float),
      "mass": self.get("bunnyMass", app.objectMass() * 100, float),
      "margin": self.get("margin", 0.5, float)
    }
    b = app.loadSoftBody("data/bunny.obj", **kws)
    objs.append(b)
    # Create box above bunny
    exts = V3(kws["scale"], kws["scale"], 1)
    pos = kws["pos"] + V3(z=kws["scale"] * 2.5)
    mass = kws["mass"] * 10
    b = app.createBox(mass=mass, pos=pos, exts=exts, rgba=withAlpha(C3_WHT, 0.5))
    objs.append(b)
    app.setBodyTexture(b, "data/white.png")
    return self._applyTransforms(app, objs)
# 0}}}

class RandomUrdfScenario(ScenarioBase): # {{{0
  """
  Add a bunch of random objects to the world
  """
  def __init__(self, name=None, **kwargs):
    super(RandomUrdfScenario, self).__init__(_name(self, name), **kwargs)

  def setup(self, app):
    objs = super(RandomUrdfScenario, self).setup(app)
    size = app.getSlider("URDF Size")
    xmin, xmax = app.worldXMin(), app.worldXMax()
    ymin, ymax = app.worldYMin(), app.worldYMax()
    zmin, zmax = app.worldZMin(), app.worldZMax()
    for i in range(app.numObjects()):
      spos = V3(
        random.uniform(xmin, xmax),
        random.uniform(ymin, ymax),
        random.uniform(zmin, zmax))
      f = pyb3.getRandomUrdf()
      if f:
        objs.append(app.loadURDF(f, basePosition=spos, globalScaling=size))
    return objs
# 0}}}

class ChainScenario(ScenarioBase): # {{{0
  """
  Add what looks vaguely like a chain
  """
  def __init__(self, name=None, **kwargs):
    super(ChainScenario, self).__init__(_name(self, name), **kwargs)

  def setup(self, app):
    objs = super(ChainScenario, self).setup(app)
    #base = app.worldFloor() + something...
    exts = app.objectRadius() * V3(1, 1, 1)
    anchor = app.createBox(mass=0, pos=V3(), exts=exts)
    objs.append(anchor)
    for i in range(app.numObjects()):
      pass
    return objs
# 0}}}
