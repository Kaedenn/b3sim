from __future__ import print_function

import argparse
import random
import sys
import time
from pyb3 import pybullet as p
from bulletbase import B3, BulletAppBase
from utility import *
from keypressmgr import *
from greek import *
from scenarios import *

import numpy as np

# Globals, constants, enumerations {{{0

# Information on the available keybinds
# Unused: b e f n q r t u x y z
KEYBIND_HELP_TEXT = r"""Available keybinds:
h       Display keybind help (this list)
?       Print camera/target information
`       Toggle built-in keyboard shortcuts
\       Pause/resume simulation
F1      Render scene to a file
Escape  Close simulation

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
p       Begin/end logging of timings
s       Toggle shadow map
v       Toggle rendering visual geometry
w       Toggle wireframe drawing
"""

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
      sphereZOffset:  starting minimum height for the spheres (10)
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
      fuzzy:          apply random jitter to certain scenarios (0.0)

    Scenario-specific keyword arguments:
      bouncy:         ("walls") make things bounce off the walls (False)
      rand:           (various) adjust body velocities by +/- 1% (False)
      projSpeed       ("projectile") speed of the projectiles (20)
      bunnyZ          ("bunny") height of the bunny off the floor (5)
      bunnySize       ("bunny") size mult of the bunny (2)
      bunnyMass       ("bunny") mass of the bunny (objectMass * 10)

    If b1, b2, and b3 are False, then "pit" is set to True.

    The following debug inputs are applied only on resets (the backslash key):
      numBricks
      numBrickLayers
      wallAlpha
      sphereZOffset

    Note: the following constraints should hold to avoid the simulation locking
    up trying to calculate object motions. "<<" means "much less than" (as in,
    by at least about an order of magnitude).
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
    self.addSlider("sphereZOffset", 0, 20,
        self.getArg("sphereZOffset", self._ws, int))
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
    return np.array([0, 0, -self._ws + self._wt])
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
      p1 = self.worldFloor() + np.array([-wm, i, 0])
      p2 = self.worldFloor() + np.array([wm, i, 0])
      self.drawLine(p1, p2, color=C_NONE)
      # Draw a line from (-wm, -i) to (wm, -i)
      p1 = self.worldFloor() + np.array([-wm, -i, 0])
      p2 = self.worldFloor() + np.array([wm, -i, 0])
      self.drawLine(p1, p2, color=C_NONE)
      # Draw a line from (i, -wm) to (i, wm)
      p1 = self.worldFloor() + np.array([i, -wm, 0])
      p2 = self.worldFloor() + np.array([i, wm, 0])
      self.drawLine(p1, p2, color=C_NONE)
      # Draw a line from (-i, -wm) to (-i, wm)
      p1 = self.worldFloor() + np.array([-i, -wm, 0])
      p2 = self.worldFloor() + np.array([-i, wm, 0])
      self.drawLine(p1, p2, color=C_NONE)
  # 0}}}

  def _addObjects(self): # {{{0
    "Add all configured objects to the world"
    scs = [EmptyPlaneScenario()]
    wantWalls = self.getArg("walls")
    wantBricks1 = self.getArg("bricks") or self.getArg("b1")
    wantBricks2 = self.getArg("b2")
    wantBricks3 = self.getArg("b3")
    wantBricksAny = wantBricks1 or wantBricks2 or wantBricks3
    wantProjectile = self.getArg("projectile")
    wantPit = self.getArg("pit")
    wantBunny = self.getArg("bunny")
    wantRandom = self.getArg("fuzzy")
    if wantWalls:
      scs.append(BoxedScenario())
    if wantPit or not wantBricksAny:
      scs.append(BallpitScenario())
    if wantBricks1:
      scs.append(BrickTower1Scenario())
    if wantBricks2:
      scs.append(BrickTower2Scenario())
    if wantBricks3:
      scs.append(BrickTower3Scenario())
    if wantProjectile:
      scs.append(ProjectileScenario(
        rand=wantRandom,
        projShape=self.getArg("projShape", ProjectileScenario.SHAPE_SPHERE, int),
        projSize=self.getArg("projSize", self.objectRadius()/2, float),
        velCoeff=self.getArg("projSpeed", 20, float)))
    if wantBunny:
      scs.append(BunnyScenario(
        rand=wantRandom,
        bunnyZ=self.getArg("bunnyZ", 5, float),
        bunnySize=self.getArg("bunnySize", 2, float),
        bunnyMass=self.getArg("bunnyMass", self.objectMass() * 10, float)))
    for sc in scs:
      sc.setup(self)

    if self.getArg("axes") or self._debug:
      self._addAxes()
    if self.getArg("grid"):
      self._addGrid()
  # 0}}}

  def reset(self): # {{{0
    "Reset the entire simulation"
    self.debug("Resetting simulation")
    super(Simulation, self).reset()
    self.setRealTime(False)
    self.setRendering(False)
    self.updateGravity()
    self._addObjects()
    self.setRendering(True)
    if not self.getArg("frozen"):
      self.setRealTime(True)
  # 0}}}

  def tick(self): # {{{0
    "Apply continuous information from the user debug inputs"
    self.updateGravity()
  # 0}}}

  def stepSim(self): # {{{0
    "Advance the simulation by one step"
    p.stepSimulation()
  # 0}}}

def onHelpKey(app): # {{{0
  "Print help information"
  p.b3Print(KEYBIND_HELP_TEXT.rstrip())
# 0}}}

def onStatusKey(app, keys): # {{{0
  "Print status information"
  if keys.get(p.B3G_SHIFT, 0) & (p.KEY_IS_DOWN | p.KEY_WAS_TRIGGERED):
    c = formatVec(app.getCameraPos())
    o = formatVec([app.getPitch(), app.getYaw(), app.getDistanceToTarget()])
    t = formatVec(app.getTargetPos())
    s = u"""Camera position, camera orientation, target position:
cxyz: {camera}
c{theta}{phi}{delta}: {orientation}
txyz: {target}
""".format(camera=c,
           orientation=o,
           target=t,
           theta=GREEK_LOWER_THETA,
           phi=GREEK_LOWER_PHI,
           delta=GREEK_LOWER_DELTA)
    p.b3Print(s.encode("UTF-8").rstrip())
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
  MOVEMENTS = {
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
  }
  movement = {}
  # Determine what we're updating
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

def onToggleCov(app, covname, dflt=False): # {{{0
  "Callback: toggle a debug configuration option"
  cov = B3.getCovByName(covname)
  app.toggleCov(cov)
# 0}}}

def mainLoop(args, a, kbm): # {{{0
  "Main loop: process key events, mouse events, and changes to GUI settings"
  tickCounter = 0
  startTime = monotonic()
  getTickTime = lambda f: startTime + INPUT_SLEEP * f
  try:
    while a.isConnected():
      tickCounter += 1
      #for e in p.getMouseEvents():
      #  tp, mx, my, bi, bs = e
      #  print(e)
      # Handle key events
      with kbm:
        if args.debug:
          kbm.debugDumpKeys()
      # Update continuous variables, like gravity
      a.tick()
      # Sleep until the next tick
      nextTickTime = getTickTime(tickCounter + 1)
      dt = nextTickTime - monotonic()
      if dt > 0:
        time.sleep(dt)
  except p.NotConnectedError:
    pass
# 0}}}

if __name__ == "__main__": # {{{0
  # If python -i, run PYTHONSTARTUP
  if sys.flags.interactive and "PYTHONSTARTUP" in os.environ:
    execfile(os.environ["PYTHONSTARTUP"])

  # Parse arguments
  parser = argparse.ArgumentParser(
      usage="%(prog)s [options] [pybullet-options]",
      epilog="""
All unhandled arguments are passed to pybullet as-is. The -c argument can be
used more than once. For --cam, "Df" means "default value". Passing --noloop
will prevent some keybinds (pause, reset) from working.
""",
      formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument("--keys-help", action="store_true",
                      help="print keybind help information and exit")
  parser.add_argument("-n", dest="num", metavar="N", type=int, default=DFLT_OBJECTS,
                      help="number of objects (general magnitude)")
  parser.add_argument("-r", dest="radius", metavar="R", type=float, default=DFLT_RADIUS,
                      help="object radius/size (general magnitude)")
  parser.add_argument("-m", dest="mass", metavar="M", type=float, default=DFLT_MASS,
                      help="object mass (general magnitude)")
  parser.add_argument("-g", "--gui", action="store_true",
                      help="start with the GUI visible")
  parser.add_argument("-f", "--frozen", action="store_true",
                      help="start with the simulation frozen")
  parser.add_argument("-c", dest="config", action="append", metavar="KEY[=VAL]",
                      help="set simulation configuration option(s)")
  parser.add_argument("--cam", metavar="P,Y,D,Tx,Ty,Tz", default="Df,Df,Df,Df,Df,Df",
                      help="set starting camera position and orientation")
  parser.add_argument("--cdp", metavar="NUM", type=float, default=KEY_STEP_PITCH,
                      help="camera movement step: camera pitch")
  parser.add_argument("--cdy", metavar="NUM", type=float, default=KEY_STEP_YAW,
                      help="camera movement step: camera yaw")
  parser.add_argument("--cdd", metavar="NUM", type=float, default=KEY_STEP_DIST,
                      help="camera movement step: distance from camera to target")
  parser.add_argument("--cdtx", metavar="NUM", type=float, default=KEY_STEP_X,
                      help="camera movement step: target x coordinate")
  parser.add_argument("--cdty", metavar="NUM", type=float, default=KEY_STEP_Y,
                      help="camera movement step: target y coordinate")
  parser.add_argument("--cdtz", metavar="NUM", type=float, default=KEY_STEP_Z,
                      help="camera movement step: target z coordinate")
  parser.add_argument("--noloop", action="store_true",
                      help="don't enter the main loop (for interactive scripting)")
  parser.add_argument("--debug", action="store_true",
                      help="output quite a bit of debugging information")
  args, remainder = parser.parse_known_args()
  if args.keys_help:
    parser.print_help()
    sys.stderr.write("\n")
    sys.stderr.write(KEYBIND_HELP_TEXT)
    raise SystemExit(0)

  # Determine the arguments to pass to pybullet.connect
  simArgs = []
  if not args.gui:
    simArgs.append("--nogui")
  simArgs.extend(remainder)

  # Determine the arguments to pass to the simulation class
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
  extraKwargs["connectArgs"] = " ".join(simArgs)

  # Construct the simulation
  app = a = Simulation(numObjects=args.num, **extraKwargs)

  # Parse and apply the --cam argument
  cinfo = parseCameraSpec(args.cam,
                          CAM_START_PITCH,
                          CAM_START_YAW,
                          CAM_START_DIST,
                          CAM_START_TARGET)
  camPitch, camYaw, camDist, camTarget = cinfo
  a.setCamera(pitch=camPitch, yaw=camYaw, distance=camDist, target=camTarget)
  # Start the simulation
  a.reset()

  # Hide the preview windows as they're completely unused
  app.toggleCov(p.COV_ENABLE_RGB_BUFFER_PREVIEW, False)
  app.toggleCov(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, False)
  app.toggleCov(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, False)

  # Bind various keys (camera movement, toggles, help, etc)
  kbm = KeyPressManager(a, debug=args.debug)
  kbm.bind(p.B3G_SPACE, a.reset)
  kbm.bindAll(KEY_CLASS_ARROWS + KEY_CLASS_NUMPAD,
              lambda keys: onMovementKey(a, keys),
              wantKeyInfo=True,
              on=p.KEY_IS_DOWN | p.KEY_WAS_TRIGGERED)
  kbm.bind("h", onHelpKey, funcArgs=(a,))
  kbm.bind("/", lambda keys: onStatusKey(a, keys), wantKeyInfo=True)
  kbm.bind(".", a.stepSim, on=p.KEY_IS_DOWN | p.KEY_WAS_TRIGGERED)
  kbm.bind("m", onToggleCov,
           funcArgs=(a, "ENABLE_MOUSE_PICKING"),
           funcKwargs={"dflt": True})
  kbm.bind("`", onToggleCov,
           funcArgs=(a, "ENABLE_KEYBOARD_SHORTCUTS"),
           funcKwargs={"dflt": True})
  kbm.bind("\\", a.toggleSim)

  # Process events until the server closes
  if not args.noloop:
    mainLoop(args, a, kbm)
# 0}}}

# vim: set ts=2 sts=2 sw=2 et:

