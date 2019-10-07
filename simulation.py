#!/usr/bin/env python

"""
PyBullet Simulation: Driver Application

This module defines specific pybullet simulations with the scenarios defined
in scenarios.py. See bulletbase.py, pyb3.py, and the Simulation.__init__
docstring for more information.
"""

from __future__ import print_function

import argparse
import logging
import random
import sys
import time
import numpy as np
from pyb3 import pybullet as p
from bulletbase import B3, BulletAppBase
from utility import *
from keypressmgr import *
from symbols import *
from scenarios import *

# Logging {{{0
LOGGING_FORMAT = "%(filename)s:%(lineno)s:%(levelname)s:%(name)s: %(message)s"
logging.basicConfig(format=LOGGING_FORMAT, level=logging.INFO)
logger = logging.getLogger(__name__)
# 0}}}

# Keybind help text {{{0
# Unused: b e f n q r t u x y z , 1 2 3 4 5 6 7 8 9 0 Num1 Num3
# Broken: c i j k l o
KEYBIND_HELP_TEXT = ur"""Available keybinds:
h       Display keybind help (this list)
b       Print entire body list
?       Print camera/target information
`       Toggle built-in keyboard shortcuts
\       Pause/resume simulation
.       Advance simulation by one timestep
F1      Render scene to a file
Escape  Close simulation

Camera movement:
Up      Increase pitch
Down    Decrease pitch
Left    Decrease yaw: rotate scene to the right
Right   Increase yaw: rotate scene to the left
Alt+Up  Decrease distance to target (shift to move 10x)
Alt+Dn  Increase distance to target (shift to move 10x)
Num 8   Move target position north (towards positive y) (shift to move 10x)
Num 2   Move target position south (towards negative y) (shift to move 10x)
Num 6   Move target position east (towards positive x) (shift to move 10x)
Num 4   Move target position west (towards negative x) (shift to move 10x)
Num 9   Move target position up (towards positive z) (shift to move 10x)
Num 7   Move target position down (towards negative z) (shift to move 10x)
Num 5   Reset target position to the center of the world floor

Built-in keyboard shortcuts (X = may not work):
a       Toggle AABB (during wireframe)
c   X   Toggle drawing of contact points
d       Toggle deactivation
g       Toggle rendering of the grid and GUI
i   X   Pause simulation
j   X   Toggle drawing of framerate
k   X   Toggle drawing of constraints
l   X   Toggle drawing of constraint limits
m       Toggle mouse picking
o   X   Single-step simulation
p       Begin/end logging of timings (to {timings_path})
s       Toggle shadow map (object shading)
v       Toggle rendering visual geometry
w       Toggle wireframe drawing
""".format(timings_path="/tmp/timings")
# 0}}}

# Globals, constants, enumerations {{{0

# Default values for argument parsing (configurable)
DFLT_OBJECTS = 10
DFLT_RADIUS = 0.5
DFLT_MASS = 1

# Constants regarding camera movement
DIR_POS = 1
DIR_NEG = -1

# Starting camera position and orientation
CAM_START_PITCH = 340
CAM_START_YAW = 0
CAM_START_DIST = 25
CAM_START_TARGET = np.zeros(3)

# Default key step amounts (configurable)
KEY_STEP_DIST = 0.25
KEY_STEP_PITCH = 5
KEY_STEP_YAW = 5
KEY_STEP_X = 0.5
KEY_STEP_Y = 0.5
KEY_STEP_Z = 0.5

# Time step (seconds) between subsequent config/input handling events
INPUT_SLEEP = 0.05

# 0}}}

class Simulation(BulletAppBase):
  """
  Simulate the interactions of various objects, with extensive configuration.
  See Simulation.__init__ for configuration information.
  """
  def __init__(self, numObjects, *args, **kwargs): # {{{0
    """
    Positional arguments:
      numObjects      number of objects (sometimes order of magnitude)

    Extra keyword arguments: (see BulletAppBase.__init__ for more)
      connectArgs:    extra args to pass to pybullet.connect()
      worldSize:      world scale (10)
      worldSizeMax:   maximum world scale (100)
      objectRadius:   radius for each object (0.05)
      objectMass:     mass for each object (0.05)
      wallThickness:  thickness of bounding-box walls (0.1)
      numBricks:      override (approximate) number of bricks (numObjects)
      brickWidth:     width of each brick (0.5)
      brickLength:    length of each brick (0.1)
      brickHeight:    height of each brick (0.15)
      brickDist:      distance between bricks (0.5)
      brickScale:     brick scale factor (2)
      numBrickLayers: height of brick tower, in number of bricks (20)
      wallAlpha:      default transparency of the box walls (0.1)

    Extra keyword arguments for populating objects:
      walls:          add walls (False)
      bricks:         add bricks (False, identical to b1)
      b1:             add bricks scenario 1 (False, identical to bricks)
      b2:             add bricks scenario 2 (False)
      b3:             add bricks scenario 3 (False)
      projectile:     add projectile scenario (False)
      pit:            add "ball pit" scenario (False)
      bunny:          add bunny scenario (False)

    Scenario-specific keyword arguments:
      bouncy:         (various) set body resitution to 1 (False)
      rand:           (various) adjust body velocities by +/- 1% (False)
      projShape       ("projectile") 1, 2, 3 for SPHERE, BOX, CAPSULE (1)
      projSize        ("projectile") projectile size (objectRadius/2)
      projSpeed       ("projectile") speed of the projectiles (20)
      bunnyZ          ("bunny") height of the bunny off the floor (5)
      bunnySize       ("bunny") size mult of the bunny (2)
      bunnyMass       ("bunny") mass of the bunny (objectMass * 10)

    If b1, b2, and b3 are False, then "pit" is set to True.

    Note: the following constraints should hold to avoid the simulation locking
    up trying to calculate object motions. "<<" means "much less than".
      0 < numObjects
      0 < numBricks
      0 < objectRadius < worldSize
      0 < brickWidth <= brickDist
      0 < brickLength <= brickDist
      0 < brickHeight <= brickDist
      0 < brickScale
      worldSize << worldSizeMax
    """
    super(Simulation, self).__init__(
        createBodyKwargs={"useMaximalCoordinates": True},
        *args, **kwargs)
    self._n = numObjects
    self._ws = self.getArg("worldSize", 10, int)
    self._wsmax = self.getArg("worldSizeMax", 100, int)
    self._or = self.getArg("objectRadius", 0.05, float)
    self._om = self.getArg("objectMass", 0.05, float)
    self._wt = self.getArg("wallThickness", 0.1, float)
    # Brick size information
    self._bw = self.getArg("brickWidth", 0.5, float)
    self._bl = self.getArg("brickLength", 0.1, float)
    self._bh = self.getArg("brickHeight", 0.15, float)
    self._bd = self.getArg("brickDist", 0.5, float)
    self._bs = self.getArg("brickScale", 2, float)
    # Allow using a pre-existing server
    if not self.isConnected():
      self.connect(kwargs.get("connectArgs", None))
    # Create parameters available in the GUI
    self.addSlider("numBricks", 1, 100,
        self.getArg("numBricks", self._n, int))
    self.addSlider("numBrickLayers", 1, 100,
        self.getArg("numBrickLayers", 20, int))
    self.addSlider("wallAlpha", 0, 1,
        self.getArg("wallAlpha", 0.1, float))
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
    wantBricks2 = self.getArg("bricks2") or self.getArg("b2")
    wantBricks3 = self.getArg("bricks3") or self.getArg("b3")
    wantBricksAny = wantBricks1 or wantBricks2 or wantBricks3
    wantProjectile = self.getArg("projectile")
    wantPit = self.getArg("pit")
    wantBunny = self.getArg("bunny")
    wantRand = self.getArg("rand")
    if wantWalls:
      scs.append(BoxedScenario())
    if wantPit or not wantBricksAny:
      scs.append(BallpitScenario(rand=wantRand))
    if wantBricks1:
      scs.append(BrickTower1Scenario(rand=wantRand))
    if wantBricks2:
      scs.append(BrickTower2Scenario(rand=wantRand))
    if wantBricks3:
      scs.append(BrickTower3Scenario())
    if wantProjectile:
      kws = {}
      kws["projShape"] = self.getArg("projShape", ProjectileScenario.SHAPE_SPHERE, int)
      kws["projSize"] = self.getArg("projSize", self.objectRadius()/2, float)
      kws["velCoeff"] = self.getArg("projSpeed", 20, float)
      kws["projMass"] = self.getArg("projMass", 100, float)
      kws["projTargetX"] = self.getArg("projTargetX", 0, float)
      kws["projTargetY"] = self.getArg("projTargetY", 0, float)
      kws["projTargetZ"] = self.getArg("projTargetZ", 0, float)
      scs.append(ProjectileScenario(rand=wantRand, **kws))
    if wantBunny:
      kws = {}
      kws["bunnyZ"] = self.getArg("bunnyZ", 5, float)
      kws["bunnySize"] = self.getArg("bunnySize", 2, float)
      kws["bunnyMass"] = self.getArg("bunnyMass", self.objectMass() * 10, float)
      scs.append(BunnyScenario(**kws))
    for sc in scs:
      if self.getArg("bouncy"):
        sc.set("bouncy", True)
      if self.getArg("frictionless"):
        sc.set("frictionless", True)
      self.debug("Setting up scenario {!r}".format(sc))
      for b in sc.setup(self):
        self._bodies.add(b)
    logger.debug("Added {} bodies from {} scenarios".format(len(self._bodies), len(scs)))

    if self.getArg("axes") or self._debug:
      self._addAxes()
    if self.getArg("grid"):
      self._addGrid()
  # 0}}}

  def dropBody(self):
    "Drop an object onto the pile of things"
    m = self.objectMass() * 100
    r = 0.5
    z = self.worldSize() - 4*self.wallThickness() - 2*r
    self._bodies.add(self.createSphere(m, V3(0, 0, z), radius=r))

  def fireProjectile(self, size=0.25, mass=1, speed=10):
    ci = self.getCamera()
    vel = ci["camForward"] * speed
    s = self.createSphere(mass, self.getCameraPos(), radius=size)
    p.resetBaseVelocity(s, linearVelocity=vel)
    self._bodies.add(s)

  def reset(self): # {{{0
    "Reset the entire simulation"
    self.debug("Resetting simulation")
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

def onKeyHelp(app): # {{{0
  "Print help information"
  p.b3Print(KEYBIND_HELP_TEXT.rstrip())
# 0}}}

def onKeyDump(app): # {{{0
  "Dump information on all bodies"
  print("Tracked bodies: {}".format(app._bodies))
  for bnr in range(p.getNumBodies()):
    bid = p.getBodyUniqueId(bnr)
    print("Body {} {}:".format(bnr, bid))
    try:
      bd = app.getBodyDynamics(bid)
      p1, p2 = app.getAABB(bid)
      print("  AABB: {} to {}".format(p1, p2))
      print("  Mass: {}".format(bd["mass"]))
      for s in app.getBodyShapes(bid):
        tn = B3.getGeometryName(s["geomType"])
        print("  Shape: {}; Size: {}; Color: {}".format(tn, s["size"], s["rgbaColor"]))
    except p.error as e:
      logger.exception(e)
      logger.error("Failed getting info: {}".format(e))
# 0}}}

def onKeyStatus(app, keys): # {{{0
  "Print status information"
  if keys.get(p.B3G_SHIFT, 0) & (p.KEY_IS_DOWN | p.KEY_WAS_TRIGGERED):
    c = formatVec(app.getCameraPos())
    o = formatVec([app.getPitch(), app.getYaw(), app.getDistanceToTarget()])
    t = formatVec(app.getTargetPos())
    kwds = {
      "camera": c,
      "orientation": o,
      "target": t,
      "theta": GREEK_LOWER_THETA,
      "phi": GREEK_LOWER_PHI,
      "delta": GREEK_LOWER_DELTA
    }
    s = u"""
Camera xyz: {camera}
Camera {theta}{phi}{delta}: {orientation}
Target xyz: {target}""".format(**kwds)
    p.b3Print(s.encode("UTF-8"))
# 0}}}

def onMovementKey(app, keys): # {{{0
  "Handle pressing one of the four arrow keys"
  def getDir(negKey, posKey): # {{{1
    "Determine if we go positive or negative"
    if isPressed(keys, negKey):
      return DIR_NEG
    elif isPressed(keys, posKey):
      return DIR_POS
    return 0 # 1}}}
  # Rules that define what keys do what
  MOVEMENTS = { # {{{1
    CAM_AXIS_PITCH: {
      "enable": anyPressed(keys, p.B3G_UP_ARROW, p.B3G_DOWN_ARROW) \
          and not isPressed(keys, p.B3G_ALT) \
          and isPressed(keys, p.B3G_CONTROL),
      "step": app.getArg("cdp"),
      "dir": getDir(p.B3G_UP_ARROW, p.B3G_DOWN_ARROW),
      "fast": False
    },
    CAM_AXIS_YAW: {
      "enable": anyPressed(keys, p.B3G_LEFT_ARROW, p.B3G_RIGHT_ARROW) \
          and not isPressed(keys, p.B3G_ALT),
      "step": app.getArg("cdy"),
      "dir": getDir(p.B3G_LEFT_ARROW, p.B3G_RIGHT_ARROW),
    },
    CAM_AXIS_DIST: {
      "enable": not isPressed(keys, p.B3G_CONTROL) \
          and anyPressed(keys, p.B3G_UP_ARROW, p.B3G_DOWN_ARROW) \
          and not isPressed(keys, p.B3G_ALT),
      "step": app.getArg("cdd"),
      "dir": getDir(p.B3G_UP_ARROW, p.B3G_DOWN_ARROW),
      "fast": isPressed(keys, p.B3G_SHIFT)
    },
    CAM_AXIS_X: {
      "enable": anyPressed(keys, p.B3G_KP_4, p.B3G_KP_6),
      "step": app.getArg("cdtx"),
      "dir": getDir(p.B3G_KP_4, p.B3G_KP_6),
      "fast": isPressed(keys, p.B3G_SHIFT)
    },
    CAM_AXIS_Y: {
      "enable": anyPressed(keys, p.B3G_KP_2, p.B3G_KP_8),
      "step": app.getArg("cdty"),
      "dir": getDir(p.B3G_KP_2, p.B3G_KP_8),
      "fast": isPressed(keys, p.B3G_SHIFT)
    },
    CAM_AXIS_Z: {
      "enable": anyPressed(keys, p.B3G_KP_7, p.B3G_KP_9),
      "step": app.getArg("cdtz"),
      "dir": getDir(p.B3G_KP_7, p.B3G_KP_9),
      "fast": isPressed(keys, p.B3G_SHIFT)
    },
  } # 1}}}
  # Calculate desired movement
  movement = {}
  for axis, rules in MOVEMENTS.items():
    if rules["enable"]:
      dv = rules["step"] * rules["dir"]
      if rules.get("fast"):
        dv *= 10
      movement[axis] = movement.get(axis, 0) + dv
  app.moveCamera(**movement)
  # NUMPAD_5 resets the camera target to the world floor
  if isPressed(keys, p.B3G_KP_5):
    app.setTargetPos(app.worldFloor())
# 0}}}

def onToggleCov(app, cov, dflt=False): # {{{0
  "Callback: toggle a debug configuration option"
  if isinstance(cov, basestring):
    cov = B3.getCovByName(cov)
  app.toggleCov(cov)
  logger.info("Set COV {} to {}".format(B3.getCovName(cov), app.covActive(cov)))
# 0}}}

### Driver: argument parsing, initialization, keybinds ###

class SimulationDriver(object):
  """
  Simulation driver class
  """
  def __init__(self, args, remainder):
    self.args = args
    self.remainder = remainder

    app_args = self._gatherSimArgs()
    self.app = Simulation(args.num, **app_args)
    self.app.setCamera(**self._parseCameraArg(args.cam))
    self.app.reset()

    self.app.toggleCov(p.COV_ENABLE_RGB_BUFFER_PREVIEW, False)
    self.app.toggleCov(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, False)
    self.app.toggleCov(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, False)

    self.kbm = self._setupKeyBinds()

    if not args.noloop:
      self._loop()

  def _gatherBulletArgs(self):
    "Gather the arguments to pass to pybullet.connect"
    # Determine the arguments to pass to pybullet.connect
    simArgs = []
    if self.args.width:
      simArgs.append("--width={}".format(self.args.width))
    if self.args.height:
      simArgs.append("--height={}".format(self.args.height))
    if not self.args.gui:
      simArgs.append("--nogui")

    # Determine background color arguments
    cr, cg, cb = self.args.bgr, self.args.bgg, self.args.bgb
    if self.args.bg:
      if self.args.bg[0] == '#' and len(self.args.bg) == 7:
        rv, gv, bv = self.args.bg[1:3], self.args.bg[3:5], self.args.bg[5:7]
        cr = int("0x" + rv) * 1.0 / 256
        cg = int("0x" + gv) * 1.0 / 256
        cb = int("0x" + bv) * 1.0 / 256
      elif self.args.bg.count(",") == 2:
        cr, cg, cb = map(float, self.args.bg.split(","))
      else:
        logger.error("Failed parsing --bg {!r}; ignoring".format(self.args.bg))
    if cr is not None:
      simArgs.append("--background_color_red={}".format(cr))
    if cg is not None:
      simArgs.append("--background_color_green={}".format(cg))
    if cb is not None:
      simArgs.append("--background_color_blue={}".format(cb))

    simArgs.extend(self.remainder)
    return simArgs

  def _gatherSimArgs(self):
    "Gather the arguments to pass to the simulation"
    extraKwargs = {}
    if self.args.config is not None:
      for val in self.args.config:
        if "=" in val:
          k, v = val.split("=", 1)
          extraKwargs[k] = v
        else:
          extraKwargs[val] = True
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
    extraKwargs["connectArgs"] = " ".join(gatherBulletArgs(self.args, self.remainder))
    if self.args.save:
      extraKwargs["save"] = self.args.save
    if self.args.load:
      extraKwargs["load"] = self.args.load
    return extraKwargs

  def _parseCameraArg(self, arg):
    "Parse the --cam argument"
    cstart = (CAM_START_PITCH, CAM_START_YAW, CAM_START_DIST, CAM_START_TARGET)
    cinfo = parseCameraSpec(arg, *cstart)
    return {
      "pitch": cinfo[0],
      "yaw": cinfo[1],
      "distance": cinfo[2],
      "target": cinfo[3]
    }

  def _setupKeyBinds(self):
    "Create the keypress manager and register keypress events"
    kbm = KeyPressManager(self.app, debug=self.args.debug)
    onMove = lambda keys: onMovementKey(self.app, keys)
    for key in KEY_CLASS_ARROWS + KEY_CLASS_NUMPAD:
      kbm.bind(key, onMove, keys=True, repeat=True)
    kbm.bind(" ", lambda: self.app.reset())
    kbm.bind("h", lambda: onKeyHelp(self.app))
    kbm.bind("b", lambda: onKeyDump(self.app))
    kbm.bind("/", lambda keys: onKeyStatus(self.app, keys), keys=True)
    kbm.bind(".", lambda: self.app.stepSim(), repeat=True)
    kbm.bind("m", lambda: onToggleCov(self.app, p.COV_ENABLE_MOUSE_PICKING, dflt=True))
    kbm.bind("`", lambda: onToggleCov(self.app, p.COV_ENABLE_KEYBOARD_SHORTCUTS, dflt=True))
    kbm.bind("\\", lambda: self.app.toggleSim())
    kbm.bind("f", lambda: self.app.fireProjectile())
    return kbm

  def _loop(self):
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
        #for e in p.getMouseEvents():
        #  tp, mx, my, bi, bs = e
        #  print(e)
        # Update continuous variables
        self.app.updateGravity()
        # Sleep until the next tick
        nextTickTime = getTickTime(tickCounter + 1)
        dt = nextTickTime - monotonic()
        if dt > 0:
          time.sleep(dt)
    except p.NotConnectedError:
      pass
    except p.error as e:
      logger.exception(e)

def parseArgs(): # {{{0
  # Parse arguments
  ap = argparse.ArgumentParser(
      usage="%(prog)s [options] [pybullet-options]",
      epilog="""
All unhandled arguments are passed to pybullet as-is. The -c argument can be
used more than once. For --cam, "Df" means "default value". Passing --noloop
will prevent some keybinds (pause, reset) from working.""",
      formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  ap.add_argument("--keys-help", action="store_true",
                  help="print keybind help information and exit")
  ap.add_argument("--width", metavar="NUM", type=int, default=1024,
                  help="width of the example browser window")
  ap.add_argument("--height", metavar="NUM", type=int, default=768,
                  help="height of the example browser window")
  ap.add_argument("-n", dest="num", metavar="N", type=int, default=DFLT_OBJECTS,
                  help="number of objects (general magnitude)")
  ap.add_argument("-r", dest="radius", metavar="R", type=float, default=DFLT_RADIUS,
                  help="object radius/size (general magnitude)")
  ap.add_argument("-m", dest="mass", metavar="M", type=float, default=DFLT_MASS,
                  help="object mass (general magnitude)")
  ap.add_argument("-g", "--gui", action="store_true",
                  help="start with the GUI visible")
  ap.add_argument("-f", "--frozen", action="store_true",
                  help="start with the simulation frozen")
  ap.add_argument("-c", dest="config", action="append", metavar="KEY[=VAL]",
                  help="set simulation configuration option(s)")
  ap.add_argument("--bgr", metavar="NUM", type=int,
                  help="background color: red component")
  ap.add_argument("--bgg", metavar="NUM", type=int,
                  help="background color: green component")
  ap.add_argument("--bgb", metavar="NUM", type=int,
                  help="background color: blue component")
  ap.add_argument("--bg", metavar="COLOR",
                  help="background color: 3-tuple or hex string")
  ap.add_argument("--cam", metavar="P,Y,D,Tx,Ty,Tz", default="Df,Df,Df,Df,Df,Df",
                  help="set starting camera position and orientation")
  ap.add_argument("--cdp", metavar="NUM", type=float, default=KEY_STEP_PITCH,
                  help="camera movement step: camera pitch")
  ap.add_argument("--cdy", metavar="NUM", type=float, default=KEY_STEP_YAW,
                  help="camera movement step: camera yaw")
  ap.add_argument("--cdd", metavar="NUM", type=float, default=KEY_STEP_DIST,
                  help="camera movement step: distance from camera to target")
  ap.add_argument("--cdtx", metavar="NUM", type=float, default=KEY_STEP_X,
                  help="camera movement step: target x coordinate")
  ap.add_argument("--cdty", metavar="NUM", type=float, default=KEY_STEP_Y,
                  help="camera movement step: target y coordinate")
  ap.add_argument("--cdtz", metavar="NUM", type=float, default=KEY_STEP_Z,
                  help="camera movement step: target z coordinate")
  ap.add_argument("--noloop", action="store_true",
                  help="don't enter the main loop (for interactive scripting)")
  ap.add_argument("--save", metavar="PATH",
                  help="save .bullet file before starting the simulation")
  ap.add_argument("--load", metavar="PATH",
                  help="load .bullet file before starting the simulation")
  ap.add_argument("--debug", action="store_true",
                  help="output quite a bit of debugging information")
  args, remainder = ap.parse_known_args()
  if args.keys_help:
    ap.print_help()
    sys.stderr.write("\n")
    sys.stderr.write(KEYBIND_HELP_TEXT)
    raise SystemExit(0)
  if args.debug:
    logger.setLevel(logging.DEBUG)
  return args, remainder
# 0}}}

def mainLoop(args, a, kbm): # {{{0
  "Main loop: process key events, mouse events, and changes to GUI settings"
  tickCounter = 0
  startTime = monotonic()
  getTickTime = lambda f: startTime + INPUT_SLEEP * f
  try:
    while a.isConnected():
      tickCounter += 1
      # Handle key events
      with kbm:
        if args.debug:
          kbm.debugDumpKeys()
      #for e in p.getMouseEvents():
      #  tp, mx, my, bi, bs = e
      #  print(e)
      # Update continuous variables
      a.updateGravity()
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

def gatherBulletArgs(args, remainder): # {{{0
  "Gather the arguments to pass to pybullet.connect"
  # Determine the arguments to pass to pybullet.connect
  simArgs = []
  if args.width:
    simArgs.append("--width={}".format(args.width))
  if args.height:
    simArgs.append("--height={}".format(args.height))
  if not args.gui:
    simArgs.append("--nogui")

  # Determine background color arguments
  cr, cg, cb = args.bgr, args.bgg, args.bgb
  if args.bg:
    if args.bg[0] == '#' and len(args.bg) == 7:
      rv, gv, bv = args.bg[1:3], args.bg[3:5], args.bg[5:7]
      cr = int("0x" + rv) * 1.0 / 256
      cg = int("0x" + gv) * 1.0 / 256
      cb = int("0x" + bv) * 1.0 / 256
    elif args.bg.count(",") == 2:
      cr, cg, cb = map(float, args.bg.split(","))
    else:
      logger.error("Failed parsing --bg {!r}; ignoring".format(args.bg))
  if cr is not None:
    simArgs.append("--background_color_red={}".format(cr))
  if cg is not None:
    simArgs.append("--background_color_green={}".format(cg))
  if cb is not None:
    simArgs.append("--background_color_blue={}".format(cb))

  simArgs.extend(remainder)
  return simArgs
# 0}}}

def gatherSimArgs(args, remainder): # {{{0
  "Gather the arguments to pass to the simulation"
  extraKwargs = {}
  if args.config is not None:
    for val in args.config:
      if "=" in val:
        k, v = val.split("=", 1)
        extraKwargs[k] = v
      else:
        extraKwargs[val] = True
  if args.debug:
    extraKwargs["debug"] = True
  if args.frozen:
    extraKwargs["frozen"] = True
  if args.radius is not None:
    extraKwargs["objectRadius"] = args.radius
  if args.mass is not None:
    extraKwargs["objectMass"] = args.mass
  extraKwargs["cdp"] = args.cdp
  extraKwargs["cdy"] = args.cdy
  extraKwargs["cdd"] = args.cdd
  extraKwargs["cdtx"] = args.cdtx
  extraKwargs["cdty"] = args.cdty
  extraKwargs["cdtz"] = args.cdtz
  extraKwargs["connectArgs"] = " ".join(gatherBulletArgs(args, remainder))
  if args.save:
    extraKwargs["save"] = args.save
  if args.load:
    extraKwargs["load"] = args.load
  return extraKwargs
# 0}}}

def parseCameraArg(arg): # {{{0
  cstart = (CAM_START_PITCH, CAM_START_YAW, CAM_START_DIST, CAM_START_TARGET)
  cinfo = parseCameraSpec(arg, *cstart)
  return {
    "pitch": cinfo[0],
    "yaw": cinfo[1],
    "distance": cinfo[2],
    "target": cinfo[3]
  }
# 0}}}

def setupKeyBinds(a): # {{{0
  "Create the keypress manager and register keypress events"
  kbm = KeyPressManager(a, debug=args.debug)
  onMove = lambda keys: onMovementKey(a, keys)
  for key in KEY_CLASS_ARROWS + KEY_CLASS_NUMPAD:
    kbm.bind(key, onMove, keys=True, repeat=True)
  kbm.bind(" ", lambda: a.reset())
  kbm.bind("h", lambda: onKeyHelp(a))
  kbm.bind("b", lambda: onKeyDump(a))
  kbm.bind("/", lambda keys: onKeyStatus(a, keys), keys=True)
  kbm.bind(".", lambda: a.stepSim(), repeat=True)
  kbm.bind("m", lambda: onToggleCov(a, p.COV_ENABLE_MOUSE_PICKING, dflt=True))
  kbm.bind("`", lambda: onToggleCov(a, p.COV_ENABLE_KEYBOARD_SHORTCUTS, dflt=True))
  kbm.bind("\\", lambda: a.toggleSim())
  kbm.bind("f", lambda: a.fireProjectile())
  return kbm
# 0}}}

if __name__ == "__main__": # {{{0
  # If python -i, run PYTHONSTARTUP
  if sys.flags.interactive and "PYTHONSTARTUP" in os.environ:
    execfile(os.environ["PYTHONSTARTUP"])

  # Parse sys.argv
  args, remainder = parseArgs()

  driver = SimulationDriver(args, remainder)
  raise SystemExit(0)

  # Construct the simulation
  app = Simulation(args.num, **gatherSimArgs(args, remainder))

  # Parse and apply the --cam argument
  app.setCamera(**parseCameraArg(args.cam))

  # Start the simulation
  app.reset()

  # Hide the preview windows as they're completely unused
  app.toggleCov(p.COV_ENABLE_RGB_BUFFER_PREVIEW, False)
  app.toggleCov(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, False)
  app.toggleCov(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, False)

  # Bind various keys (camera movement, toggles, help, etc)
  kbm = setupKeyBinds(app)

  # Process events until the server closes
  if not args.noloop:
    mainLoop(args, app, kbm)
# 0}}}

# vim: set ts=2 sts=2 sw=2 et:

