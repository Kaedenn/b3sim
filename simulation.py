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

import numpy as np

# BPL:
# eye = target - direction * distance
# forward = (target-eye).normalize()*dt
# right=forward.cross(up).normalize()*dt
# up=cam.up*dt
# and there you go, with right/up/forward and some dx/dy/dz you've got already the offset vectors to adjust your view matrix or eye/target values

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

Built-in keyboard shortcuts:
a       Toggle AABB
c       Toggle drawing of contact points
d       Toggle deactivation
g       Toggle rendering of the grid and GUI
i       Pause simulation
j       Toggle drawing of framerate
k       Toggle drawing of constraints
l       Toggle drawing of constraint limits
m       Toggle mouse picking
o       Single-step simulation
p       Begin/end logging of timings
s       Toggle shadow map
v       Toggle rendering visual geometry
w       Toggle wireframe drawing
"""

# Default values for argument parsing
DFLT_OBJECTS = 10
DFLT_RADIUS = 0.5
DFLT_MASS = 1

# Constants regarding camera movement
DIR_POS = 1
DIR_NEG = -1
KEY_STEP_DIST = 0.25
KEY_STEP_PITCH = 5
KEY_STEP_YAW = 5
KEY_STEP_X = 0.5
KEY_STEP_Y = 0.5
KEY_STEP_Z = 0.5

# Time step (seconds) between subsequent config/input handling events
INPUT_SLEEP = 0.05

# Grace period to debounce keyboard events (intervals of INPUT_SLEEP)
DEFAULT_GRACE_PERIOD = 20

# Box size information
BW = 0.5 # box width
BL = 0.10 # box length
BH = 0.15 # box height
BD = 0.501 # box distance

# Keyboard keys that result in camera movement
MOVEMENT_KEYS = KEY_CLASS_ARROWS + KEY_CLASS_NUMPAD_ARROWS + (p.B3G_KP_7, p.B3G_KP_9)
# 0}}}

class Simulation(BulletAppBase):
  "Simulate interaction of objects in a box"
  def __init__(self, n, *args, **kwargs): # {{{0
    """
    Extra keyword arguments: (see BulletAppBase.__init__ for more)
      argv:           extra args to pass to pybullet.connect()
      worldSize:      world scale (10)
      objectRadius:   radius for each object (0.05)
      objectMass:     mass for each object (0.05)
      walls:          add walls (False)
      bricks:         add bricks (False, unless walls and spheres are False)
      spheres:        add spheres (False)
      sphereZOffset:  starting minimum height for the spheres (10)
      wallThickness:  thickness of bounding-box walls (0.1)
      numBricks:      override (approximate) number of bricks (numObjects)
      brickWidth:     width of each brick
      brickLength:    length of each brick
      brickHeight:    height of each brick
      brickDist:      distance between bricks
      brickScale:     brick scale factor (1)
      brickLayers:    height of brick tower, in number of bricks (20)
    """
    super(Simulation, self).__init__(*args, **kwargs)
    self._spheres = set()
    self._n = n
    self._ws = getRemove(kwargs, "worldSize", 10, int)
    self._sr = getRemove(kwargs, "objectRadius", 0.05, float)
    self._sm = getRemove(kwargs, "objectMass", 0.05, float)
    self._wt = getRemove(kwargs, "wallThickness", 0.1, float)
    # Brick size information
    self._bw = getRemove(kwargs, "brickWidth", 0.5, float)
    self._bl = getRemove(kwargs, "brickLength", 0.10, float)
    self._bh = getRemove(kwargs, "brickHeight", 0.15, float)
    self._bd = getRemove(kwargs, "brickDist", 0.5, float)
    self._bs = getRemove(kwargs, "brickScale", 1, float)
    # Allow using a pre-existing server
    if not self.isConnected():
      self.connect(getRemove(kwargs, "argv", None))
    # Create parameters available in the GUI
    self.addUserDebug("numObjects", 1, 100,
        getRemove(kwargs, "numObjects", n, int))
    self.addUserDebug("numBricks", 1, 100,
        getRemove(kwargs, "numBricks", n, int))
    self.addUserDebug("numBrickLayers", 1, 100,
        getRemove(kwargs, "brickLayers", 20, int))
    self.addUserDebug("wallAlpha", 0, 1, 0.1)
    self.addUserDebug("worldSize", 2, 100, 10)
    self.addUserDebug("sphereZOffset", 0, 20,
        getRemove(kwargs, "sphereZOffset", 10, int))
  # 0}}}

  def worldFloor(self): # {{{0
    "Returns a vector at the center of the world's floor"
    return np.array([0, 0, -self._ws + self._wt])
  # 0}}}

  def _addWorldPlane(self): # {{{0
    self._plane = self.createBox([100, 100, 0], self.worldFloor(), [0, 0, 0, 0])
  # 0}}}

  def _addWorldTray(self): # {{{0
    self._tray = self.addURDF("data/tray/traybox.urdf")
  # 0}}}

  def _addWorldWalls(self): # {{{0
    """
    Create walls around the ball pit

    Walls enclose a cube with side length self._ws centered around 0, 0, with
    the bottom face co-planar to the world plane.

    Available interior area is exactly
      -(self._ws - self._wt) < {x,y,z} < self._ws - self._wt
    """
    alpha = self.getUserDebug("wallAlpha")
    wt = np.float(self._wt)
    ws = np.float(self._ws)
    wallcx = np.array([wt, ws+wt, ws+wt]) # wall x collision along y axis
    wallcy = np.array([ws+wt, wt, ws+wt]) # wall y collision along x axis
    wallcz = np.array([ws+wt, ws+wt, wt]) # wall z collision
    self.createBox(wallcx, np.array([ws, 0, 0]), rgba=withAlpha(C3_RED, alpha))
    self.createBox(wallcx, np.array([-ws, 0, 0]), rgba=withAlpha(C3_BLU, alpha))
    self.createBox(wallcy, np.array([0, ws, 0]), rgba=withAlpha(C3_GRN, alpha))
    self.createBox(wallcy, np.array([0, -ws, 0]), rgba=withAlpha(C3_MAG, alpha))
    self.createBox(wallcz, np.array([0, 0, ws]), rgba=withAlpha(C3_GRN, alpha))
    self.createBox(wallcz, np.array([0, 0, -ws]), rgba=withAlpha(C3_BLU, alpha))

    # Ground plane
    gp = [10*ws+wt, 10*ws+wt, wt]
    self.createBox(gp, np.array([0, 0, -ws]), rgba=[0, 0, 0, 0])
  # 0}}}

  def _addBricksVersion1(self, layers, num): # {{{0
    "Add bricks in an alternating tower"
    n = int(round(num**0.5))
    bw = self._bw * self._bs
    bl = self._bl * self._bs
    bh = self._bh * self._bs
    bd = self._bd * self._bs
    box1 = np.array([bw/2, bl/2, bh/2])
    box2 = np.array([bl/2, bw/2, bh/2])
    bx = lambda num: bd * (num + (1-n)/2)
    by = lambda num: bd * (num + (1-n)/2)
    bz = lambda num: num * bh
    boxes = []
    for i in range(n):
      for j in range(n):
        for k in range(layers):
          boxes.append((box1, [bx(i), by(j), bz(2*k)]))
          boxes.append((box2, [bx(i), by(j), bz(2*k+1)]))
    return boxes
  # 0}}}

  def _addBricksVersion2(self, layers, num): # {{{0
    "Add bricks like version 1, but with alternating platforms"
    n = int(round(num**0.5))
    bw = self._bw * self._bs
    bl = self._bl * self._bs
    bh = self._bh * self._bs
    bd = self._bd * self._bs
    box1 = np.array([bw/2, bl/2, bh/2])
    box2 = np.array([bl/2, bw/2, bh/2])
    box3 = np.array([n*bd/2, n*bd/2, bh/2])
    bx = lambda num: bd * (num + (1-n)/2)
    by = lambda num: bd * (num + (1-n)/2)
    bz = lambda num: num * bh
    boxes = []
    for k in range(layers/2):
      for i in range(n):
        for j in range(n):
          boxes.append((box1, [bx(i), by(j), bz(3*k)]))
          boxes.append((box2, [bx(i), by(j), bz(3*k+1)]))
      boxes.append((box3, [0, 0, bz(3*k+2)]))
    return boxes
  # 0}}}

  def _addBricksVersion3(self, layers, num): # {{{0
    "Add pillars"
    n = int(num)
    bw = self._bw * self._bs / 2
    bl = self._bl * self._bs / 2
    bh = self._bh * self._bs * 2
    bd = self._bd * self._bs
    box1 = np.array([bw/2, bl/2, bh/2])
    box2 = np.array([bl/2, bw/2, bh/2])
    bx = lambda num: bd * (num + (1-n)/2)
    by = lambda num: bd * (num + (1-n)/2)
    bz = lambda num: num * bh + bh/2
    boxes = []
    c = 0
    cs = [box1, box2]
    for k in range(layers):
      for i in range(n):
        for j in range(n):
          boxes.append((cs[c], [bx(i), by(j), bz(k)]))
          c = (c + 1) % 2
    return boxes
  # 0}}}

  def _addBricks(self): # {{{0
    "Add bricks to the simulation"
    layers = int(round(self.getUserDebug("numBrickLayers")))
    num = int(round(self.getUserDebug("numBricks")))
    # Create a temp list because self.createBox is expensive and (somehow)
    # increases the cost of everything in the loop with it
    if self._hasArg("v2"):
      boxes = self._addBricksVersion2(layers=layers, num=num)
    elif self._hasArg("v3"):
      boxes = self._addBricksVersion3(layers=layers, num=num)
    else:
      boxes = self._addBricksVersion1(layers=layers, num=num)
    boxBasePos = self.worldFloor()
    boxMass = self._sm/2
    for bc, bp in boxes:
      self.createBox(bc, boxBasePos + bp, mass=boxMass)
  # 0}}}

  def _addSpheres(self): # {{{0
    self._spColl = self.createCollision(p.GEOM_SPHERE, self._sr)
    offset = np.array([0, 0, self.getUserDebug("sphereZOffset")])
    for i in range(self._n):
      sxyz = np.array([
        random.uniform(-0.01, 0.01),
        random.uniform(-0.01, 0.01),
        2 * self._sr * i + random.random() * self._sr
      ]) + self.worldFloor() + offset
      s = self.createSphere(self._sm, sxyz, self._sr)
      p.resetBaseVelocity(s, linearVelocity=[0, 0, -10])
      self._spheres.add(s)
  # 0}}}

  def _addAxes(self): # {{{0
    "Draw axes at the world floor"
    for aname, avec in CAM_AXES.items():
      pt = self.worldFloor() + 2*avec
      self.drawText(aname, pt, color=CAM_AXIS_COLORS[aname])
      self.drawLine(self.worldFloor(), pt, color=CAM_AXIS_COLORS[aname])
  # 0}}}

  def _addObjects(self): # {{{0
    "Add all configured objects to the world"
    self._addWorldPlane()
    walls = self._getArg("walls")
    bricks = self._getArg("bricks")
    spheres = self._getArg("spheres")
    if any((walls, bricks, spheres)):
      if self._getArg("walls"):
        self._addWorldWalls()
      if self._getArg("bricks"):
        self._addBricks()
      if self._getArg("spheres"):
        self._addSpheres()
    else:
      self._addBricks()
    if self._getArg("axes") or self._debug:
      self._addAxes()
  # 0}}}

  def reset(self): # {{{0
    """
    Reset the entire simulation
    """
    self.debug("Resetting simulation")
    super(Simulation, self).reset()
    self._n = self.getUserDebug("numObjects", int)
    self._ws = self.getUserDebug("worldSize")
    self.setRealTime(False)
    self.setRendering(False)
    self.updateGravity()
    self._addObjects()
    self.setRendering(True)
    self.setRealTime(True)
  # 0}}}

  def tick(self): # {{{0
    self.updateGravity()
  # 0}}}

def onHelpKey(app): # {{{0
  "Print help information"
  sys.stderr.write(KEYBIND_HELP_TEXT)
# 0}}}

def onStatusKey(app, keys): # {{{0
  "Print status information"
  if keys.get(p.B3G_SHIFT, 0) & (p.KEY_IS_DOWN | p.KEY_WAS_TRIGGERED):
    c = formatVec(app.getCameraPos())
    o = formatVec([app.getPitch(), app.getYaw(), app.getDistanceToTarget()])
    t = formatVec(app.getTargetPos())
    sys.stderr.write(u"""Camera position, camera orientation, target position:
cxyz: {camera}
c{theta}{phi}{delta}: {orientation}
txyz: {target}
""".format(camera=c,
           orientation=o,
           target=t,
           theta=GREEK_LOWER_THETA,
           phi=GREEK_LOWER_PHI,
           delta=GREEK_LOWER_DELTA))
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
      "step": KEY_STEP_PITCH,
      "dir": getDir(p.B3G_UP_ARROW, p.B3G_DOWN_ARROW),
      "fast": False
    },
    CAM_AXIS_YAW: {
      "enable": anyPressed(keys, p.B3G_LEFT_ARROW, p.B3G_RIGHT_ARROW) \
          and not isPressed(keys, p.B3G_ALT),
      "step": KEY_STEP_YAW,
      "dir": getDir(p.B3G_LEFT_ARROW, p.B3G_RIGHT_ARROW),
    },
    CAM_AXIS_DIST: {
      "enable": not isPressed(keys, p.B3G_CONTROL) \
          and anyPressed(keys, p.B3G_UP_ARROW, p.B3G_DOWN_ARROW) \
          and not isPressed(keys, p.B3G_ALT),
      "step": KEY_STEP_DIST,
      "dir": getDir(p.B3G_UP_ARROW, p.B3G_DOWN_ARROW),
      "fast": isPressed(keys, p.B3G_SHIFT)
    },
    CAM_AXIS_X: {
      "enable": anyPressed(keys, p.B3G_KP_4, p.B3G_KP_6),
      "step": KEY_STEP_X,
      "dir": getDir(p.B3G_KP_4, p.B3G_KP_6),
      "fast": isPressed(keys, p.B3G_SHIFT)
    },
    CAM_AXIS_Y: {
      "enable": anyPressed(keys, p.B3G_KP_2, p.B3G_KP_8),
      "step": KEY_STEP_Y,
      "dir": getDir(p.B3G_KP_2, p.B3G_KP_8),
      "fast": isPressed(keys, p.B3G_SHIFT)
    },
    CAM_AXIS_Z: {
      "enable": anyPressed(keys, p.B3G_KP_7, p.B3G_KP_9),
      "step": KEY_STEP_Z,
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
  app.moveCameraBy(**movement)
  # NUMPAD_5 resets the camera target to the world floor
  if isPressed(keys, p.B3G_KP_5):
    app.setTargetPos(app.worldFloor())
# 0}}}

def onToggleCov(app, covname, dflt=False): # {{{0
  "Callback: toggle a debug configuration option"
  cov = B3.getCovByName(covname)
  v = not app.get(cov, dflt)
  sys.stderr.write("Setting {} to {}\n".format(covname, v))
  p.configureDebugVisualizer(cov, 1 if v else 0)
  app.set(cov, v)
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

  # Parse arguments {{{1
  parser = argparse.ArgumentParser(
      usage="%(prog)s [options] [pybullet-options]",
      epilog="All unhandled arguments are passed to pybullet as-is",
      formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument("-n", dest="num", metavar="N", type=int, default=DFLT_OBJECTS,
                      help="number of objects (general magnitude)")
  parser.add_argument("-r", dest="radius", metavar="R", type=float, default=DFLT_RADIUS,
                      help="object radius/size (general magnitude)")
  parser.add_argument("-m", dest="mass", metavar="M", type=float, default=DFLT_MASS,
                      help="object mass (general magnitude)")
  parser.add_argument("--gui", action="store_true", help="start with the GUI visible")
  parser.add_argument("--help-keys", action="store_true", help="print keybinds")
  parser.add_argument("-c", dest="config", action="append", metavar="KEY[=VAL]",
                      help="set simulation configuration option(s); can be " +
                           "used more than once")
  parser.add_argument("--debug", action="store_true", help="print debug information")
  parser.add_argument("--noloop", action="store_true", help="don't enter the main loop")
  args, remainder = parser.parse_known_args()
  if args.help_keys:
    parser.print_help()
    sys.stderr.write("\n")
    sys.stderr.write(KEYBIND_HELP_TEXT)
    raise SystemExit(0)
  if not args.gui:
    remainder.append("--nogui")
  # 1}}}

  # Determine the arguments to pass to the simulation class {{{1
  extraArgs = " ".join(remainder)
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
  if args.radius is not None:
    extraKwargs["objectRadius"] = args.radius
  if args.mass is not None:
    extraKwargs["objectMass"] = args.mass
  # 1}}}

  # Construct the simulation
  app = a = Simulation(n=args.num, argv=extraArgs, **extraKwargs)
  # Set the starting camera location to look at the world floor
  a.setCamera(pitch=305, yaw=0, distance=10, target=app.worldFloor())
  # Start the simulation
  a.reset()

  # Hide the preview windows as they're completely unused
  p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
  p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
  p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)

  # Bind various keys (camera movement, toggles, help, etc) {{{1
  kbm = KeyPressManager(a, debug=args.debug)
  kbm.bind(p.B3G_SPACE, a.reset)
  kbm.bindAll(MOVEMENT_KEYS + (p.B3G_KP_5,),
              lambda keys: onMovementKey(a, keys),
              wantKeyInfo=True,
              on=p.KEY_IS_DOWN | p.KEY_WAS_TRIGGERED)
  kbm.bind("h", onHelpKey, funcArgs=(a,))
  kbm.bind("/", lambda keys: onStatusKey(a, keys), wantKeyInfo=True)
  kbm.bind("m", onToggleCov,
           funcArgs=(a, "ENABLE_MOUSE_PICKING"),
           funcKwargs={"dflt": True})
  kbm.bind("`", onToggleCov,
           funcArgs=(a, "ENABLE_KEYBOARD_SHORTCUTS"),
           funcKwargs={"dflt": True})
  kbm.bind("\\", a.toggleSim, timeoutPeriod=0)
  # 1}}}

  # Process events until the server closes
  if not args.noloop:
    mainLoop(args, a, kbm)
# 0}}}

# vim: set ts=2 sts=2 sw=2 et:

