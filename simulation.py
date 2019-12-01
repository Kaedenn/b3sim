#!/usr/bin/env python

"""
PyBullet Simulation: Driver Application

This module defines specific pybullet simulations with the scenarios defined
in scenarios.py. See bulletbase.py, pyb3.py, and the Simulation.__init__
docstring for more information.
"""

from __future__ import print_function

import argparse
import datetime
import glob
import logging
import random
import sys
import textwrap
import time
import numpy as np
import pyb3
from pyb3 import pybullet as p
from bulletbase import *
from keypressmgr import *
from scenarios import *
from symbols import *
from utility import *

logger = pyb3.getLogger()

# Time step (seconds) between subsequent config/input handling events
INPUT_SLEEP = 0.05

# Starting camera position and orientation (degrees) {{{0
CAM_START_TARGET = V3(0, 0, -80)
CAM_START_PITCH = 340
CAM_START_YAW = 0
CAM_START_DIST = 100
# 0}}}

# Default key step amounts {{{0
KEY_STEP_DIST = 0.25
KEY_STEP_PITCH = 5.0
KEY_STEP_YAW = 5.0
KEY_STEP_X = 0.5
KEY_STEP_Y = 0.5
KEY_STEP_Z = 0.5
KEY_FAST_MULT = 10.0
# 0}}}

# Keybind help text {{{0
# Unused:
#   e n q r t u x y z 1 2 3 4 5 6 7 8 9 0 , ; ' [ ] - =
#   Num-1 Num-3 F2 F3 F4 F5 F6 F7 F8 F9 F10 F11
# Broken: c i j k l o
# Unusable:
#   Bksp  Maps to B3G_F9
#   Tab   Maps to B3G_F10
KEYBIND_HELP_TEXT = r"""
Available keybinds:
h       Print the keybind help text (this list)
b       Print information on every body in the simulation
f       Fire a projectile towards the target
F       Fire several projectiles towards the target
?       Print camera position and orientation information 
`       Toggle built-in keyboard shortcuts
\       Pause or un-pause the simulation
.       Advance simulation by one client timestep
F12     Start/end rendering to an mp4 file (see below)
C-o     Present an open-file dialog for loading URDF files
C-e     If debugging is enabled, inject an error into the client

Built-in keybinds (X = may not work):
Esc     Close simulation
F1      Start/end rendering frames to PNGs
a       Toggle AABB (during wireframe)
c       Toggle drawing of contact points
d       Toggle deactivation
g       Toggle rendering of the grid and GUI
i   X   Pause simulation
j       Toggle frame counter drawing (requires custom Bullet)
k       Toggle drawing of constraints
l       Toggle drawing of constraint limits
m       Toggle mouse picking
o   X   Single-step simulation
p       Begin/end logging of timings (to {timings_path})
s       Toggle shadow map (object shading)
v       Toggle rendering visual geometry
w       Toggle wireframe drawing

Key modifiers:
S-<k>   Both <k> and Shift pressed
M-<k>   Both <k> and Alt pressed
C-<k>   Both <k> and Ctrl pressed
^<k>    Identical to C-<k>

Client timesteps may be a multiple of the server simulation rate; the server
attempts to perform simulation calculations on the order of hundreds per
second while the client only renders at 30 or 60 frames per second. This may
result in the "advance simulation by one client timestep" key advancing the
simulation by multiple frames.

This difference also affects exporting to mp4.

The mp4 file will have the following format: pyb3-YYYYMMDDHHMISS.mp4
  YYYY: Current year
  MM: Current month number
  DD: Current day-of-month
  HH: Current hour, in 24-hour format
  MI: Current minute
  SS: Current second
and will be saved to the current directory, presently {cwd}.

Inputs are handled every {df} seconds ({fps} times per second). This includes
keys, camera movement, GUI slider configuration, and GUI buttons.
""".format(timings_path="/tmp/timings",
           cwd=os.getcwd(),
           df=INPUT_SLEEP,
           fps=1/INPUT_SLEEP)
# 0}}}

# Camera movement help text {{{0
CAMERA_HELP_TEXT = """
Camera movement:
Up      Increase pitch: angle camera to point down
Down    Decrease pitch: angle camera to point up
Left    Decrease yaw: rotate scene to the right
Right   Increase yaw: rotate scene to the left
C-Up    Decrease distance to target (shift to move 10x)
C-Down  Increase distance to target (shift to move 10x)
Num-8   Move target position north (towards positive y) (shift to move 10x)
Num-2   Move target position south (towards negative y) (shift to move 10x)
Num-6   Move target position east (towards positive x) (shift to move 10x)
Num-4   Move target position west (towards negative x) (shift to move 10x)
Num-9   Move target position up (towards positive z) (shift to move 10x)
Num-7   Move target position down (towards negative z) (shift to move 10x)
Num-5   Reset target position to the center of the world floor
S-Num-5 Reset camera to the default position and orientation

Camera starting position and orientation:
  target x: {}
  target y: {}
  target z: {}
  pitch: {} degrees
  yaw: {} degrees
  distance: {} units

Camera movement rates in units per frame:
  target x: {} ({} if holding shift)
  target y: {} ({} if holding shift)
  target z: {} ({} if holding shift)
  pitch: {} degrees
  yaw: {} degrees
  distance: {} ({} if holding shift)
""".format(
    CAM_START_TARGET[0],
    CAM_START_TARGET[1],
    CAM_START_TARGET[2],
    CAM_START_PITCH,
    CAM_START_YAW,
    CAM_START_DIST,
    KEY_STEP_X,
    KEY_STEP_X * KEY_FAST_MULT,
    KEY_STEP_Y,
    KEY_STEP_Y * KEY_FAST_MULT,
    KEY_STEP_Z,
    KEY_STEP_Z * KEY_FAST_MULT,
    KEY_STEP_PITCH,
    KEY_STEP_YAW,
    KEY_STEP_DIST,
    KEY_STEP_DIST * KEY_FAST_MULT)
# 0}}}

class Simulation(BulletAppBase):
  """
  Simulate the interactions of various objects, with extensive configuration.
  See Simulation.__init__ for configuration information.
  """
  def __init__(self, numObjects, *args, **kwargs): # {{{0
    """
    Positional arguments:
      numObjects      object count (each scenario interprets this differently)

    Extra keyword arguments: (see BulletAppBase.__init__ for more)
      connectArgs     extra args to pass to pybullet.connect()
      worldSize       world scale (100)
      worldSizeMax    maximum world scale (1000)
      objectRadius    default object radius (0.5)
      objectMass      default object mass (5)
      planeTexture    texture for the world plane ("data/white.png")

    Extra keyword arguments for populating objects (True/False):
      walls           add walls
      bricks (or b1)  add bricks scenario 1
      b2              add bricks scenario 2: "stick tower"
      b3              add bricks scenario 3: "reinforced"
      projectile      add projectile scenario
      pit             add "ball pit" scenario
      bunny           add bunny scenario
      urdfs           add random URDFs scenario
      chain           add chain scenario (not yet implemented)

    Arguments specific to the "walls" scenario:
      wallThickness   bounding-box wall thickness (1.0)
      wallAlpha       bounding-box wall opacity (0.1 or 10%)

    Arguments specific to the "b1", "b2", and "b3" scenarios:
      brickWidth      width half-extent of each brick (5.0)
      brickLength     length half-extent of each brick (1.0)
      brickHeight     height half-extent of each brick (1.5)
      brickDist       distance between neighboring centers of mass (5.0)
      brickScale      brick scale factor (1.0)
      numBrickLayers  number of layers used in the brick tower (20)

    Arguments specific to the "projectile" scenario:
      projShape       1, 2, or 3 for Sphere, Box, or Capsule (1)
      projSize        projectile radius (objectRadius / 2)
      projSpeed       projectile base speed (20)
      projMass        projectile base mass (100)
      projTargetX     projectile target X (0)
      projTargetY     projectile target Y (0)
      projTargetZ     projectile target Z (0)

    Arguments specific to the "bunny" scenario:
      bunnyZ          height of the bunny off the floor (20)
      bunnySize       size multiplier of the bunny (10)
      bunnyMass       mass of the bunny (objectMass * 10)
      margin          collision margin (applies to all soft bodies) (0.5)

    Arguments applying to most scenarios:
      bouncy          set all body resitutions to 1 (False)
      rand            adjust all body velocities by +/- 1% (False)
    """
    super(Simulation, self).__init__(
        createBodyKwargs={"useMaximalCoordinates": True},
        *args, **kwargs)
    if self._debug:
      self.createSphere = pyb3.addLogging(self.createSphere)
      self.createBox = pyb3.addLogging(self.createBox)
      self.createCapsule = pyb3.addLogging(self.createCapsule)
      self.createCylinder = pyb3.addLogging(self.createCylinder)
      self.createPlane = pyb3.addLogging(self.createPlane)
      self.setBodyColor = pyb3.addLogging(self.setBodyColor)
    self._n = numObjects
    self._ws = self.getArg("worldSize", 100, int)
    self._wsmax = self.getArg("worldSizeMax", 1000, int)
    self._or = self.getArg("objectRadius", 0.5, float)
    self._om = self.getArg("objectMass", 5, float)
    self._wt = self.getArg("wallThickness", 1.0, float)
    self._bw = self.getArg("brickWidth", 5.0, float)
    self._bl = self.getArg("brickLength", 1.0, float)
    self._bh = self.getArg("brickHeight", 1.5, float)
    self._bd = self.getArg("brickDist", 5.0, float)
    self._bs = self.getArg("brickScale", 1.0, float)
    if not self.isConnected():
      self.connect()
    # Create parameters available in the GUI
    self.addSlider("Brick Layers", 1, 100, self.getArg("numBrickLayers", 20, int))
    self.addSlider("Wall Opacity", 0, 1, self.getArg("wallAlpha", 0.1, float))
    # Projectile configuration (used by self.fireProjectile)
    self.addSlider("Projectile Size", 1.0, 50.0, 2.5)
    self.addSlider("Projectile Mass", 5, 50, 25)
    self.addSlider("Particle Speed", 100, 1000, 100)
    # Inputs for adding URDFs
    self.addSlider("URDF Size", 10, 1000, 100)
    self.addButtonEvent("Add Random URDF", self.addRandomUrdf)
    self.addButtonEvent("Load URDF", self.loadUrdf)
    # Input for removing most recently added object
    self.addButtonEvent("Remove Last Body", self.removeLastBody)
  # 0}}}

  def numObjects(self): # {{{0
    "Requested number of objects"
    return self._n
  # 0}}}

  def worldSize(self): # {{{0
    "Value of the worldSize configuration option"
    return self._ws
  # 0}}}

  def worldSizeMax(self): # {{{0
    "Value of the worldSizeMax configuration option"
    return self._wsmax
  # 0}}}

  def objectRadius(self): # {{{0
    "Value of the objectRadius configuration option"
    return self._or
  # 0}}}

  def objectMass(self): # {{{0
    "Value of the objectMass configuration option"
    return self._om
  # 0}}}

  def wallThickness(self): # {{{0
    "Value of the wallThickness configuration option"
    return self._wt
  # 0}}}

  def brickWidth(self): # {{{0
    "Value of the brickWidth configuration option"
    return self._bw
  # 0}}}

  def brickLength(self): # {{{0
    "Value of the brickLength configuration option"
    return self._bl
  # 0}}}

  def brickHeight(self): # {{{0
    "Value of the brickHeight configuration option"
    return self._bh
  # 0}}}

  def brickDist(self): # {{{0
    "Value of the brickDist configuration option"
    return self._bd
  # 0}}}

  def brickScale(self): # {{{0
    "Value of the brickScale configuration option"
    return self._bs
  # 0}}}

  def worldFloor(self): # {{{0
    """
    Returns a vector at the center of the world's floor (determined by the
    walls being centered about the origin)
    """
    return V3(0, 0, -self._ws + self._wt)
  # 0}}}

  def worldXMin(self): # {{{0
    "Left-most x value still considered inside the world"
    return self._wt - self._ws
  # 0}}}

  def worldXMax(self): # {{{0
    "Right-most x value still considered inside the world"
    return self._ws - self._wt
  # 0}}}

  def worldYMin(self): # {{{0
    "North-most y value still considered inside the world"
    return self._wt - self._ws
  # 0}}}

  def worldYMax(self): # {{{0
    "South-most y value still considered inside the world"
    return self._ws - self._wt
  # 0}}}

  def worldZMin(self): # {{{0
    "Bottom-most z value still considered inside the world"
    return self.worldFloor()[2]
  # 0}}}

  def worldZMax(self): # {{{0
    "Top-most z value still considered inside the world"
    return -self.worldFloor()[2]
  # 0}}}

  def _addAxes(self): # {{{0
    "Draw axes at the world floor"
    for aname, avec in CAM_AXES.items():
      pt = self.worldFloor() + 2*avec
      self.drawText(aname, pt, color=CAM_AXIS_COLORS[aname])
      self.drawLine(self.worldFloor(), pt, color=CAM_AXIS_COLORS[aname])
  # 0}}}

  def _addGrid(self): # {{{0
    "Draw a grid on the world floor"
    wm = self.worldSizeMax()
    for i in range(int(round(wm))):
      # Draw a line from (-wm, i) to (wm, i)
      p1 = self.worldFloor() + V3(-wm, i, 0)
      p2 = self.worldFloor() + V3(wm, i, 0)
      self.drawLine(p1, p2, color=C_NONE)
      # Draw a line from (-wm, -i) to (wm, -i)
      p1 = self.worldFloor() + V3(-wm, -i, 0)
      p2 = self.worldFloor() + V3(wm, -i, 0)
      self.drawLine(p1, p2, color=C_NONE)
      # Draw a line from (i, -wm) to (i, wm)
      p1 = self.worldFloor() + V3(i, -wm, 0)
      p2 = self.worldFloor() + V3(i, wm, 0)
      self.drawLine(p1, p2, color=C_NONE)
      # Draw a line from (-i, -wm) to (-i, wm)
      p1 = self.worldFloor() + V3(-i, -wm, 0)
      p2 = self.worldFloor() + V3(-i, wm, 0)
      self.drawLine(p1, p2, color=C_NONE)
  # 0}}}

  def _addObjects(self): # {{{0
    "Add all configured objects to the world"
    scs = [EmptyPlaneScenario()]
    wantWalls = self.getArg("walls")
    wantBricks1 = self.getArg("bricks") or self.getArg("b1")
    wantBricks2 = self.getArg("b2")
    wantBricks3 = self.getArg("b3")
    wantProjectile = self.getArg("projectile")
    wantPit = self.getArg("pit")
    wantBunny = self.getArg("bunny")
    wantUrdfs = self.getArg("urdfs")
    wantRand = self.getArg("rand")
    wantChain = self.getArg("chain")
    if wantWalls:
      scs.append(BoxedScenario())
    if wantPit:
      kws = {}
      kws["objShape"] = self.getArg("projShape", ProjectileScenario.SHAPE_SPHERE, int)
      scs.append(BallpitScenario(rand=wantRand, **kws))
    if wantBricks1:
      scs.append(BrickTower1Scenario(rand=wantRand))
    if wantBricks2:
      scs.append(BrickTower2Scenario(rand=wantRand))
    if wantBricks3:
      scs.append(BrickTower3Scenario(rand=wantRand))
    if wantProjectile:
      kws = {}
      kws["projShape"] = self.getArg("projShape", ProjectileScenario.SHAPE_SPHERE, int)
      kws["projSize"] = self.getArg("projSize", self.objectRadius(), float)
      kws["velCoeff"] = self.getArg("projSpeed", 20, float)
      kws["projMass"] = self.getArg("projMass", 100, float)
      kws["projTargetX"] = self.getArg("projTargetX", 0, float)
      kws["projTargetY"] = self.getArg("projTargetY", 0, float)
      kws["projTargetZ"] = self.getArg("projTargetZ", 0, float)
      scs.append(ProjectileScenario(rand=wantRand, **kws))
    if wantBunny:
      kws = {}
      kws["bunnyZ"] = self.getArg("bunnyZ", 20, float)
      kws["bunnySize"] = self.getArg("bunnySize", 10, float)
      kws["bunnyMass"] = self.getArg("bunnyMass", self.objectMass() * 10, float)
      kws["margin"] = self.getArg("margin", 0.5, float)
      scs.append(BunnyScenario(**kws))
    if wantUrdfs:
      scs.append(RandomUrdfScenario())
    if wantChain:
      scs.append(ChainScenario())

    # Add all of the objects from all of the scenarios configured
    for sc in scs:
      if self.getArg("bouncy"):
        sc.set("bouncy", True)
      if self.getArg("frictionless"):
        sc.set("frictionless", True)
      logger.debug("Setting up scenario {!r}".format(sc))
      for b in sc.setup(self):
        self._bodies.add(b)

    # Handle --urdf argument: add URDF at a random position between (-0.5, 0.5)
    for path in self.getArg("urdf", (), tuple):
      pos = randomVec()
      logger.debug("Adding URDF {} at {}".format(path, pos))
      self.loadURDF(path, basePosition=pos)

    # Handle --sb argument: add soft bodies
    for spec in self.getArg("softBodies", (), tuple):
      parts = spec.split(":")
      path = parts[0]
      scale = float(parts[1]) if len(parts) > 1 else 1
      margin = float(parts[2]) if len(parts) > 2 else 1
      pos = V3()
      orn = p.getQuaternionFromEuler((0, math.pi/2, 0))
      logger.debug("Adding soft body {} at {}".format(path, pos))
      self.loadSoftBody(path, pos=pos, scale=scale, mass=100, orn=orn, margin=margin)

    # Handle --rb argument: add rigid bodies
    for spec in self.getArg("rigidBodies", (), tuple):
      parts = spec.split(":")
      path = parts[0]
      scale = float(parts[1]) if len(parts) > 1 else 1
      logger.debug("Creating collision {}, scale={}".format(path, scale))
      coll = self.createCollision(p.GEOM_MESH, fileName=path, meshScale=scale)
      kws = {
        "baseMass": 100,
        "baseCollisionShapeIndex": coll,
        "baseVisualShapeIndex": -1,
        "basePosition": V3()
      }
      logger.debug("Creating multibody {}".format(kws))
      self.createMultiBody(**kws)

    nb, ns = len(self._bodies), len(scs)
    logger.debug("Added {} bodies from {} scenarios".format(nb, ns))

    # Add non-body items
    if self.getArg("axes") or self._debug:
      self._addAxes()
    if self.getArg("grid"):
      self._addGrid()
  # 0}}}

  def dropBody(self): # {{{0
    "Drop an object onto the pile of things"
    m = self.objectMass() * 100
    r = 0.5
    z = self.worldSize() - 4*self.wallThickness() - 2*r
    self._bodies.add(self.createSphere(m, V3(0, 0, z), radius=r))
  # 0}}}

  def fireProjectile(self, **kwargs): # {{{0
    """Fire a projectile in the direction the camera is facing

    size: Projectile size; defaults to self.getSlider("Projectile Size")
    mass: Projectile mass; defaults to self.getSlider("Projectile Mass")
    vel: Projectile velocity (vector)
    pos: Projectile starting position (vector)

    Projectile velocity defaults at self.getSlider("Particle Speed") and in the
    direction the camera is facing.
    """
    ci = self.getCamera()
    size = kwargs.get("size", self.getSlider("Projectile Size"))
    mass = kwargs.get("mass", self.getSlider("Projectile Mass"))
    if "vel" in kwargs:
      vel = kwargs["vel"]
    else:
      vel = ci["camForward"] * self.getSlider("Particle Speed")
    pos = kwargs.get("pos", self.getCameraPos())
    logger.debug("Creating projectile m={}, p={}, r={}".format(mass, pos, size))
    s = self.createSphere(mass, pos, radius=size)
    p.resetBaseVelocity(s, linearVelocity=vel)
    self._bodies.add(s)
    return s
  # 0}}}

  def addRandomUrdf(self): # {{{0
    "Add a random URDF to the simulation"
    f = pyb3.getRandomUrdf()
    if f:
      scale = self.getSlider("URDF Size")
      logger.debug("Adding random URDF: {} (scale {})".format(f, scale))
      self.loadURDF(f, basePosition=randomVec(), globalScaling=scale)
    else:
      logger.error("Failed to add random URDF")
  # 0}}}

  def loadUrdf(self): # {{{0
    "Present an open-file dialog for loading a URDF object"
    objs = openFileDialog(title="Select object file(s)...",
                          globs=("URDF (*.urdf)|*.urdf",),
                          multiple=True)
    logger.debug("Loading objects: {!r}".format(objs))
    scale = self.getSlider("URDF Size")
    for obj in objs:
      pos = randomVec()
      logger.debug("Loading URDF {} at {} with scale {}".format(obj, pos, scale))
      self.loadURDF(obj, basePosition=pos, globalScaling=scale)
  # 0}}}

  def tick(self): # {{{0
    "Check/apply passive/continuous configuration values"
    super(Simulation, self).tick()
  # 0}}}

  def reset(self): # {{{0
    "Reset the entire simulation"
    logger.debug("Resetting simulation")
    super(Simulation, self).reset()
    self.setRealTime(False)
    self.setRendering(False)
    self.updateGravity()
    self._addObjects()
    if self.hasArg("save"):
      p.saveBullet(self.getArg("save"))
    if self.hasArg("load"):
      p.loadBullet(self.getArg("load"))
    self.setRendering(True)
    if not self.getArg("frozen"):
      self.setRealTime(True)
    logger.debug("Reset simulation")
  # 0}}}

### Global functions: input handling ###

def onMovementKey(app, keys): # {{{0
  "Handle pressing one of the movement keys"
  # Rules that define what keys do what
  movementRules = { # {{{1
    CAM_AXIS_PITCH: {
      "enable": anyPressed(keys, "C-Up", "C-Down") \
          and isPressed(keys, p.B3G_CONTROL),
      "stepArgName": "cdp",
      "dir": ("Up", "Down"),
      "enableFast": False
    },
    CAM_AXIS_YAW: {
      "enable": anyPressed(keys, "Left", "Right"),
      "stepArgName": "cdy",
      "dir": ("Left", "Right"),
      "enableFast": False
    },
    CAM_AXIS_DIST: {
      "enable": anyPressed(keys, "Up", "Down") \
          and not isPressed(keys, p.B3G_CONTROL),
      "stepArgName": "cdd",
      "dir": ("Up", "Down"),
      "enableFast": True
    },
    CAM_AXIS_X: {
      "enable": anyPressed(keys, "k4", "k6"),
      "stepArgName": "cdtx",
      "dir": ("k4", "k6"),
      "enableFast": True
    },
    CAM_AXIS_Y: {
      "enable": anyPressed(keys, "k2", "k8"),
      "stepArgName": "cdty",
      "dir": ("k2", "k8"),
      "enableFast": True
    },
    CAM_AXIS_Z: {
      "enable": anyPressed(keys, "k7", "k9"),
      "stepArgName": "cdtz",
      "dir": ("k7", "k9"),
      "enableFast": True
    },
  } # 1}}}

  def getDir(negKey, posKey): # {{{1
    "Determine if we go positive or negative"
    if isPressed(keys, negKey):
      return -1 
    elif isPressed(keys, posKey):
      return 1
    return 0 # 1}}}

  # Handle generic movement keys
  movement = {}
  for axis, rules in movementRules.items():
    if rules["enable"]:
      dv = app.getArg(rules["stepArgName"]) * getDir(*rules["dir"])
      if rules.get("enableFast") and isPressed(keys, p.B3G_SHIFT):
        dv *= KEY_FAST_MULT
      movement[axis] = dv
  app.moveCamera(**movement)
  # Special movement keys
  if isPressed(keys, p.B3G_KP_5):
    if isPressed(keys, p.B3G_SHIFT):
      # S-NUMPAD_5 resets the camera to the starting position and orientation
      app.setTargetPos(CAM_START_TARGET)
      app.setPitch(CAM_START_PITCH)
      app.setYaw(CAM_START_YAW)
      app.setTargetDistance(CAM_START_DIST)
    else:
      # NUMPAD_5 resets the camera target to the world origin (z=floor)
      app.setTargetPos(app.worldFloor())
# 0}}}

### Driver: argument parsing, initialization, keybinds ###

class SimulationDriver(object):
  "Simulation driver class"
  def __init__(self, args, remainder): # {{{0
    self.args = args
    self.remainder = remainder
    self.export = None

    # Camera configuration
    cstart = (CAM_START_PITCH, CAM_START_YAW, CAM_START_DIST, CAM_START_TARGET)
    ci = parseCameraSpec(args.cam, *cstart)

    # Gather configuration and construct the simulation
    app_args = self._gatherSimArgs()
    self.app = Simulation(args.num, **app_args)
    self.app.setCamera(pitch=ci[0], yaw=ci[1], distance=ci[2], target=ci[3])
    self.app.reset()

    # Hide the preview windows as they're not used in pybullet
    self.app.toggleCov(p.COV_ENABLE_RGB_BUFFER_PREVIEW, False)
    self.app.toggleCov(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, False)
    self.app.toggleCov(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, False)

    self.kbm = self._setupKeyBinds()

    if not args.noloop:
      self._loop()
  # 0}}}

  def _gatherBulletArgs(self): # {{{0
    "Gather the arguments to pass to pybullet.connect"
    # Determine background color arguments
    cr = cg = cb = None
    if self.args.bgr is not None:
      cr = self.args.bgr
    if self.args.bgg is not None:
      cg = self.args.bgg
    if self.args.bgb is not None:
      cb = self.args.bgb
    if self.args.bg:
      crgba = parseColor(self.args.bg, cmax=255, scale=255)
      if crgba is not None:
        cr, cg, cb = crgba[0], crgba[1], crgba[2]
      else:
        logger.error("Ignoring invalid --bg value {!r}".format(self.args.bg))

    # Determine the arguments to pass to pybullet.connect
    simArgs = []
    if self.args.width:
      simArgs.append("--width={}".format(self.args.width))
    if self.args.height:
      simArgs.append("--height={}".format(self.args.height))
    if not self.args.gui:
      simArgs.append("--nogui")
    if cr is not None:
      simArgs.append("--background_color_red={}".format(cr / 255.0))
    if cg is not None:
      simArgs.append("--background_color_green={}".format(cg / 255.0))
    if cb is not None:
      simArgs.append("--background_color_blue={}".format(cb / 255.0))
    simArgs.extend(self.remainder)

    return simArgs
  # 0}}}

  def _gatherSimArgs(self): # {{{0
    "Gather the arguments to pass to the simulation"
    extraKwargs = {}
    # Gather arguments from --config
    if self.args.config is not None:
      for val in self.args.config:
        if "=" in val:
          k, v = val.split("=", 1)
          extraKwargs[k] = v
        else:
          extraKwargs[val] = True
    # Gather general simulation arguments
    if self.args.debug:
      extraKwargs["debug"] = True
    if self.args.frozen:
      extraKwargs["frozen"] = True
    if self.args.radius is not None:
      extraKwargs["objectRadius"] = self.args.radius
    if self.args.mass is not None:
      extraKwargs["objectMass"] = self.args.mass
    extraKwargs["cdp"] = self.args.cdp
    extraKwargs["cdy"] = self.args.cdy
    extraKwargs["cdd"] = self.args.cdd
    extraKwargs["cdtx"] = self.args.cdtx
    extraKwargs["cdty"] = self.args.cdty
    extraKwargs["cdtz"] = self.args.cdtz
    extraKwargs["connectArgs"] = self._gatherBulletArgs()
    if self.args.save:
      extraKwargs["save"] = self.args.save
    if self.args.load:
      extraKwargs["load"] = self.args.load
    if self.args.mp4:
      extraKwargs["mp4"] = self.args.mp4
    if self.args.urdf:
      extraKwargs["urdf"] = self.args.urdf
    if self.args.urdfs:
      urdfs = []
      for g in self.args.urdfs:
        urdfs.extend(glob.glob(g))
      if "urdf" not in extraKwargs:
        extraKwargs["urdf"] = []
      extraKwargs["urdf"].extend(urdfs)
    if self.args.sb:
      extraKwargs["softBodies"] = self.args.sb
    if self.args.rb:
      extraKwargs["rigidBodies"] = self.args.rb
    if self.args.plugin:
      extraKwargs["plugins"] = self.args.plugin
    # Gather engine parameters
    eargs = {}
    if self.args.split_impulse:
      eargs["useSplitImpulse"] = 1
    if self.args.time_step:
      eargs["fixedTimeStep"] = self.args.time_step
    if self.args.erp:
      eargs["erp"] = self.args.erp
    if self.args.contact_erp:
      eargs["contactERP"] = self.args.contact_erp
    if self.args.friction_erp:
      eargs["frictionERP"] = self.args.friction_erp
    if eargs:
      extraKwargs["engineParams"] = eargs
    # Print configuration for debugging
    if self.args.debug:
      connArgs = extraKwargs.get("connectArgs", ())
      camArgs = ("cdp", "cdy", "cdd", "cdtx", "cdty", "cdtz")
      print("Engine arguments: {}".format(" ".join(connArgs)))
      print("Simulation arguments:")
      for key in camArgs:
        if key in extraKwargs:
          print("  Camera step {}: {}".format(key[2:], extraKwargs[key]))
      for key in sorted(extraKwargs):
        if key not in camArgs and key not in ("debug", "connectArgs"):
          print("  {}: {!r}".format(key, extraKwargs[key]))
    return extraKwargs
  # 0}}}

  def _getVideoPath(self): # {{{0
    "Format a path to an mp4 file"
    strftime = datetime.datetime.now().strftime
    return os.path.join(os.getcwd(), strftime("pyb3-%Y%m%d%H%M%S.mp4"))
  # 0}}}

  def _setupKeyBinds(self): # {{{0
    "Create the keypress manager and register keypress events"
    kbm = KeyPressManager(self.app, debug=self.args.debug)
    # TODO: Detect these and others using X11
    kbm.map("!", "S-1")
    kbm.map("?", "S-/")
    onMove = lambda keys: onMovementKey(self.app, keys)
    for key in KEY_CLASS_ARROWS + KEY_CLASS_NUMPAD:
      kbm.bind(key, onMove, keys=True, repeat=True)
    kbm.bind(" ", self._onKeyReset)
    kbm.bind("h", self._onKeyHelp)
    kbm.bind("b", self._onKeyDump)
    kbm.bind("?", self._onKeyStatus)
    kbm.bind(".", self._onKeyStep, repeat=True)
    kbm.bind("m", self._onKeyToggleMousePicking)
    kbm.bind("`", self._onKeyToggleKeyEvents)
    kbm.bind("\\", self._onKeyToggleSim)
    kbm.bind("f", self._onKeyFireProjectile, keys=True)
    kbm.bind("F12", self._onKeyVideoLogging)
    kbm.bind("^o", self.app.loadUrdf)

    # Debugging: simulate a Bullet error
    if self.args.debug:
      def injectErrorFunc():
        p.b3Print("Injected error message")
        p.b3Warning("Injected error message")
        p.b3Error("Injected error message")
        p.getAABB(-1) # Results in pybullet.error
      kbm.bind("^e", injectErrorFunc)
    return kbm
  # 0}}}

  def _loop(self): # {{{0
    "Main loop: process key events, mouse events, and changes to GUI settings"
    tickCounter = 0
    startTime = monotonic()
    getTickTime = lambda f: startTime + INPUT_SLEEP * f
    try:
      while self.app.isConnected():
        tickCounter += 1
        # Handle key events
        with self.kbm:
          if self.args.debug:
            self.kbm.debugDumpKeys()
        # Handle mouse events
        if self.args.debug:
          for e in p.getMouseEvents():
            tp, mx, my, bi, bs = e
            tpname = B3.getMouseEventName(tp, short=True)
            bname = B3.getMouseButtonName(bi, short=True)
            bstates = B3.getMouseStateNames(bs, short=True)
            print("{} {}: x={} y={} b={} {!r} s={} {}: {}".format(
              tp, tpname, mx, my, bi, bname, bs, "+".join(bstates), e))
        # Update continuous variables
        self.app.tick()
        # Sleep until the next tick
        nextTickTime = getTickTime(tickCounter + 1)
        dt = nextTickTime - monotonic()
        if dt > 0:
          time.sleep(dt)
    except p.NotConnectedError:
      pass
    except p.error as e:
      logger.exception(e)
  # 0}}}

  # Callback functions:

  def _onKeyReset(self): # {{{0
    "Callback: reset the simulation"
    self.app.reset()
  # 0}}}

  def _onKeyHelp(self): # {{{0
    "Print help information"
    p.b3Print(KEYBIND_HELP_TEXT.rstrip())
  # 0}}}

  def _onKeyDump(self): # {{{0
    "Callback: Dump information on all bodies"
    print("Tracked bodies: {}".format(self.app._bodies))
    for bnr in range(p.getNumBodies()):
      bid = p.getBodyUniqueId(bnr)
      print("Body #{} ID={}:".format(bnr, bid))
      try:
        oi = self.app.getFullObjectInfo(bid, failSoft=True)
        print("  Full info:")
        for k,v in oi.items():
          print("    {!r} = {!r}".format(k, v))
      except p.error as e:
        logger.exception(e)
      try:
        bd = self.app.getBodyDynamics(bid)
        p1, p2 = self.app.getAABB(bid)
        print("  AABB: {} to {}".format(p1, p2))
        print("  Mass: {}".format(bd["mass"]))
        for s in self.app.getBodyShapes(bid):
          tn = B3.getGeometryName(s["geomType"])
          print("  Shape: {}; Color: {}".format(tn, s["rgbaColor"]))
          print("    Size: {}".format(s["size"]))
          if np.linalg.norm(s["localFramePosition"]) > 0:
            print("    Position: {}".format(s["localFramePosition"]))
          if np.linalg.norm(s["localFrameOrientation"][:3]) > 0:
            print("    Orientation: {}".format(s["localFrameOrientation"]))
          if s["meshFileName"]:
            print("    Mesh: {}".format(s["meshFileName"]))
        for c in self.app.getBodyCollision(bid, -1, failSoft=True):
          print("  Collider: {} dim={}".format(c["geomType"], c["dimensions"]))
      except p.error as e:
        logger.exception(e)
        logger.error("Failed getting info: {}".format(e))
  # 0}}}

  def _onKeyStatus(self): # {{{0
    "Callback: print camera status information"
    ci = self.app.getCamera()
    cpos = ci[CAM_AXIS_TARGET] - ci["camForward"] * ci[CAM_AXIS_DIST]
    corn = (ci[CAM_AXIS_PITCH], ci[CAM_AXIS_YAW], ci[CAM_AXIS_DIST])
    kwds = {
      "camera": formatVec(cpos),
      "orientation": formatVec(corn),
      "target": formatVec(ci[CAM_AXIS_TARGET]),
      "theta": GREEK_LOWER_THETA,
      "phi": GREEK_LOWER_PHI,
      "delta": GREEK_LOWER_DELTA
    }
    s = u"""
Canvas size: {width} by {height} pixels
Camera xyz: {camera}
Camera {theta}{phi}{delta}: {orientation}
Target xyz: {target}
Camera up: {camUp}
Camera forward: {camForward}
View Matrix:
{viewMat}
Projection Matrix:
{projMat}
""".format(width=ci["width"],
           height=ci["height"],
           camUp=ci["camUp"],
           camForward=ci["camForward"],
           viewMat=ci["viewMat"].reshape(4, 4),
           projMat=ci["projMat"].reshape(4, 4),
           **kwds).strip()
    sys.stderr.write(s.encode("UTF-8"))
    sys.stderr.write("\n")
  # 0}}}

  def _onKeyStep(self): # {{{0
    "Callback: advance the simulation"
    self.app.stepSim()
  # 0}}}

  def _onKeyToggleMousePicking(self): # {{{0
    "Callback: toggle mouse picking"
    if self.app.toggleCov(p.COV_ENABLE_MOUSE_PICKING):
      p.b3Print("Mouse picking enabled")
    else:
      p.b3Print("Mouse picking disabled")
  # 0}}}

  def _onKeyToggleKeyEvents(self): # {{{0
    "Callback: toggle native key events"
    if self.app.toggleCov(p.COV_ENABLE_KEYBOARD_SHORTCUTS):
      p.b3Print("Native key events enabled")
    else:
      p.b3Print("Native key events disabled")
  # 0}}}

  def _onKeyToggleSim(self): # {{{0
    "Callback: toggle real-time simulation"
    self.app.toggleSim()
  # 0}}}

  def _onKeyFireProjectile(self, keys=()): # {{{0
    "Callback: fire projectile(s) in the direction the camera is facing"
    ci = self.app.getCamera()
    cpos = self.app.getCameraPos()
    posStart = cpos + 8 * ci["camForward"]
    vz = ci["camUp"]
    vy = ci["camForward"]
    vx = np.cross(vy, vz)
    if p.B3G_SHIFT in keys:
      psize = self.app.getSlider("Projectile Size") * 0.5
      dxs = (
        V3(-1,  0, -1), V3( 0,  0, -1), V3( 1,  0, -1),
        V3(-1,  0,  0), V3( 0,  0,  0), V3( 1,  0,  0),
        V3(-1,  0,  1), V3( 0,  0,  1), V3( 1,  0,  1)
      )
      for dx in dxs:
        vec = 2 * psize * (vx * dx[0] + vy * dx[1] + vz * dx[2])
        self.app.setRealTime(False)
        self.app.fireProjectile(size=psize, pos=posStart + vec)
        self.app.setRealTime(True)
    else:
      self.app.fireProjectile(pos=posStart)
  # 0}}}

  def _onKeyVideoLogging(self): # {{{0
    "Start (or stop) dumping simulation to a file"
    if self.export is None:
      path = self._getVideoPath()
      self.export = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, path)
      p.b3Print("Rendering simulation to {}".format(path))
    else:
      p.stopStateLogging(self.export)
      self.export = None
      p.b3Print("Rendering terminated")
  # 0}}}

### Global functions: argument handling and entry point ###

def parseArgs(): # {{{0
  "Return the argument object and a list of the unhandled args"
  ap = argparse.ArgumentParser(
      usage="%(prog)s [options] [pybullet-options]",
      add_help=False,
      epilog="""
Important notes:
* All options after -- will be ignored and passed to pybullet as-is.
* All unhandled arguments are passed to pybullet as-is.
* The -c argument can be used more than once.
* For --cam, "Df" means "default value". Use --help-keys to display what those
  values are.
* The default background color is hard-coded in the ExampleBrowser as RGB
  150,170,170, or #96aaaa.
* Passing --noloop will prevent custom keybinds (pause, reset) from working.
  GUI inputs (sliders, buttons) will also not work. --noloop is implied if the
  interpreter is started in interactive mode using "python -i".
* Bullet plugins can be loaded using --plugin "path:suffix". Custom code is
  needed for calling plugin commands.
* Because the client and server run at different framerates, image/video
  dumping may skip frames.""",
      formatter_class=ArgumentDefaultsHelpFormatter)
  ap.add_argument("-h", "--help", action="store_true",
                  help="show this message and exit")
  ap.add_argument("--help-keys", action="store_true",
                  help="print keybind help information and exit")
  ap.add_argument("--help-commands", action="store_true",
                  help="print command help information and exit")
  ap.add_argument("--help-all", action="store_true",
                  help="print all help information and exit")
  ap.add_argument("--width", metavar="NUM", type=int, default=1024,
                  help="width of the example browser window")
  ap.add_argument("--height", metavar="NUM", type=int, default=768,
                  help="height of the example browser window")
  ap.add_argument("-n", "--num", metavar="N", type=int, default=25,
                  help="number of objects (general magnitude)")
  ap.add_argument("-r", "--radius", metavar="R", type=float, default=0.5,
                  help="object radius/size (general magnitude)")
  ap.add_argument("-m", "--mass", metavar="M", type=float, default=5,
                  help="object mass (general magnitude)")
  ap.add_argument("-g", "--gui", action="store_true",
                  help="start with the GUI visible")
  ap.add_argument("-f", "--frozen", action="store_true",
                  help="start with the simulation paused")
  ap.add_argument("-c", dest="config", action="append", metavar="KEY[=VAL]",
                  help="simulation configuration option(s)")
  ap.add_argument("--bgr", metavar="NUM", type=int,
                  help="red background color component (0 to 255)")
  ap.add_argument("--bgg", metavar="NUM", type=int,
                  help="green background color component (0 to 255)")
  ap.add_argument("--bgb", metavar="NUM", type=int,
                  help="blue background color component (0 to 255)")
  ap.add_argument("--bg", metavar="COLOR",
                  help="background color as either 3-tuple or hex string")
  ap.add_argument("--cam", metavar="P,Y,D,Tx,Ty,Tz", default="Df,Df,Df,Df,Df,Df",
                  help="starting camera position and orientation")
  ap.add_argument("--cdp", metavar="NUM", type=float, default=KEY_STEP_PITCH,
                  help="camera pitch movement step")
  ap.add_argument("--cdy", metavar="NUM", type=float, default=KEY_STEP_YAW,
                  help="camera yaw movement step")
  ap.add_argument("--cdd", metavar="NUM", type=float, default=KEY_STEP_DIST,
                  help="camera distance movement step: distance from camera to target")
  ap.add_argument("--cdtx", metavar="NUM", type=float, default=KEY_STEP_X,
                  help="camera target X axis movement step")
  ap.add_argument("--cdty", metavar="NUM", type=float, default=KEY_STEP_Y,
                  help="camera target Y axis movement step")
  ap.add_argument("--cdtz", metavar="NUM", type=float, default=KEY_STEP_Z,
                  help="camera target Z axis movement step")
  ap.add_argument("--noloop", action="store_true",
                  help="don't enter the main loop (for interactive execution)")
  ap.add_argument("--urdf", metavar="PATH", action="append",
                  help="load URDF object(s)")
  ap.add_argument("--urdfs", metavar="GLOB", action="append",
                  help="load all URDF objects matching GLOB")
  ap.add_argument("--sb", metavar="PATH", action="append",
                  help="load soft body (can be used more than once)")
  ap.add_argument("--rb", metavar="PATH", action="append",
                  help="load rigid body (can be used more than once)")
  ap.add_argument("--save", metavar="PATH",
                  help="save .bullet file before starting the simulation")
  ap.add_argument("--load", metavar="PATH",
                  help="load .bullet file before starting the simulation")
  ap.add_argument("--mp4", metavar="PATH",
                  help="path to mp4 file (press F12 to start/stop rendering)")
  ap.add_argument("--plugin", metavar="PATH", action="append",
                  help="load shared object as a Bullet plugin")
  ap.add_argument("--debug", action="store_true",
                  help="enable diagnostic (debugging) output")
  ap.add_argument("--trace", action="store_true",
                  help="enable tracing of numerous pybullet APIs")

  # Add experimental arguments
  pg = ap.add_argument_group("experimental arguments")
  pg.add_argument("--split-impulse", action="store_true",
                  help="enable split impulse calculation")
  pg.add_argument("--time-step", metavar="SEC", type=float,
                  help="force a fixed timestep")
  pg.add_argument("--erp", metavar="VAL", type=float,
                  help="force a global error reduction parameter")
  pg.add_argument("--contact-erp", metavar="VAL", type=float,
                  help="force a global contact error reduction parameter")
  pg.add_argument("--friction-erp", metavar="VAL", type=float,
                  help="force a global friction error reduction parameter")

  # Make a copy of sys.argv for pre-processing
  argv = sys.argv[1:]

  # For storing arguments not parsed by argparse
  remainder = []

  # Extract arguments after --
  if "--" in argv:
    idx = argv.index("--")
    remainder.extend(argv[idx+1:])
    argv = argv[:idx]

  # Parse sys.argv and gather arguments not handled
  args, remargs = ap.parse_known_args(argv)
  remainder.extend(remargs)

  # Log the unhandled arguments
  for arg in remainder:
    if arg.startswith("-"):
      logger.info("Passing extra argument to pybullet: {!r}".format(arg))

  # Print help message(s) and exit
  center = lambda s: centerString(s, ch="=", padSpace=True)
  if args.help or args.help_all:
    ap.print_help()
    if args.help_keys or args.help_all:
      sys.stderr.write("\n{}\n".format(center("Keybind help text")))
      sys.stderr.write(KEYBIND_HELP_TEXT.strip("\n"))
      sys.stderr.write("\n\n")
      sys.stderr.write(CAMERA_HELP_TEXT.strip("\n"))
    if args.help_commands or args.help_all:
      sys.stderr.write("\n\n{}\n".format(center("Bullet base help text")))
      sys.stderr.write(textwrap.dedent(BulletAppBase.__init__.__doc__).strip("\n"))
      sys.stderr.write("\n\n{}\n".format(center("Simulation help text")))
      sys.stderr.write(textwrap.dedent(Simulation.__init__.__doc__).strip("\n"))
    sys.stderr.write("\n")
    raise SystemExit(0)

  # Enable debugging/verbose mode if requested
  if args.debug:
    logger.setLevel(logging.DEBUG)
    # --debug implies passing --verbose to pybullet
    if "--verbose" not in remainder:
      remainder.append("--verbose")

  return args, remainder
# 0}}}

if __name__ == "__main__": # {{{0
  # If python -i, run PYTHONSTARTUP and add --noloop
  if sys.flags.interactive and "PYTHONSTARTUP" in os.environ:
    execfile(os.environ["PYTHONSTARTUP"])
    if "--noloop" not in sys.argv:
      sys.argv.append("--noloop")

  # Parse sys.argv
  args, remainder = parseArgs()

  # Add debugging to a good number of APIs, if enabled
  if args.trace:
    p.addUserDebugLine = pyb3.addLogging(p.addUserDebugLine)
    p.addUserDebugParameter = pyb3.addLogging(p.addUserDebugParameter)
    p.changeDynamics = pyb3.addLogging(p.changeDynamics)
    p.changeVisualShape = pyb3.addLogging(p.changeVisualShape)
    p.configureDebugVisualizer = pyb3.addLogging(p.configureDebugVisualizer)
    p.connect = pyb3.addLogging(p.connect)
    p.createCollisionShape = pyb3.addLogging(p.createCollisionShape)
    p.createMultiBody = pyb3.addLogging(p.createMultiBody)
    p.createVisualShape = pyb3.addLogging(p.createVisualShape)
    p.getAABB = pyb3.addLogging(p.getAABB)
    p.getBaseVelocity = pyb3.addLogging(p.getBaseVelocity)
    p.getCollisionShapeData = pyb3.addLogging(p.getCollisionShapeData)
    p.getDebugVisualizerCamera = pyb3.addLogging(p.getDebugVisualizerCamera)
    p.getDynamicsInfo = pyb3.addLogging(p.getDynamicsInfo)
    p.getVisualShapeData = pyb3.addLogging(p.getVisualShapeData)
    p.loadPlugin = pyb3.addLogging(p.loadPlugin)
    p.loadSoftBody = pyb3.addLogging(p.loadSoftBody)
    p.loadTexture = pyb3.addLogging(p.loadTexture)
    p.loadURDF = pyb3.addLogging(p.loadURDF)
    p.resetDebugVisualizerCamera = pyb3.addLogging(p.resetDebugVisualizerCamera)
    p.setAdditionalSearchPath = pyb3.addLogging(p.setAdditionalSearchPath)
    p.setInternalSimFlags = pyb3.addLogging(p.setInternalSimFlags)
    p.setPhysicsEngineParameter = pyb3.addLogging(p.setPhysicsEngineParameter)

  # Create and begin the simulation driver
  driver = SimulationDriver(args, remainder)
# 0}}}

# vim: set ts=2 sts=2 sw=2 et:

