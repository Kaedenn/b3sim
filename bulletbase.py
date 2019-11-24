#!/usr/bin/env python

"""
PyBullet Simulation: Base Class API

This module defines a base class and a utility class for pybullet simulations.
Simulation modules can inherit from BulletAppBase to simplify the (rather
complex) pybullet API.

Note that all references to pybullet must be done through the pyb3 module in
order to provide compatibility across different versions of the pybullet API.
"""

# TODO:
# Value caching for functions called multiple times per loop
#   e.g. getCamera(), body info, etc.

import pyb3
from pyb3 import pybullet as p
from utility import *

logger = pyb3.getLogger()

# Only export the classes, not the modules
__all__ = ["B3", "BulletAppBase"]

class B3(object): # {{{0
  "Functions for mapping constants to strings and strings to constants"

  @staticmethod
  def getMouseEventName(e, short=False): # {{{1
    "Get the pybullet name for the given mouse event constant"
    if e == p.MOUSE_MOVE_EVENT:
      return "MOVE" if short else "MOUSE_MOVE_EVENT"
    if e == p.MOUSE_BUTTON_EVENT:
      return "BUTTON" if short else "MOUSE_BUTTON_EVENT"
    return "UNKNOWN"
  # 1}}}

  @staticmethod
  def getMouseButtonName(b, short=False): # {{{1
    "Get the pybullet name for the given mouse button constant"
    if b == p.MOUSE_BUTTON_NONE:
      return "NONE" if short else "MOUSE_BUTTON_NONE"
    if b == p.MOUSE_BUTTON_LEFT:
      return "LMB" if short else "MOUSE_BUTTON_LEFT"
    if b == p.MOUSE_BUTTON_RIGHT:
      return "RMB" if short else "MOUSE_BUTTON_RIGHT"
    if b == p.MOUSE_WHEEL:
      return "WHEEL" if short else "MOUSE_WHEEL"
    return "UNKNOWN"
  # 1}}}

  @staticmethod
  def getMouseStateNames(s, short=False): # {{{1
    "Return a list of states corresponding to the given value"
    names = []
    if s == 0:
      names.append("NONE")
    if s & p.MOUSE_PRESS == p.MOUSE_PRESS:
      names.append("PRESS" if short else "MOUSE_PRESS")
    if s & p.MOUSE_RELEASE == p.MOUSE_RELEASE:
      names.append("RELEASE" if short else "MOUSE_RELEASE")
    return names
  # 1}}}

  @staticmethod
  def getKeyName(k): # {{{1
    "Get the name for the given keyboard key"
    try:
      import xkeys
      for name in dir(xkeys.lib):
        if name.startswith("Key_") and getattr(xkeys.lib, name) == k:
          return name[4:]
    except ImportError:
      pass
    for name in dir(p):
      if name.startswith("B3G_") and getattr(p, name) == k:
        return name[4:]
    return "Unknown"
  # 1}}}

  @staticmethod
  def getConnectionName(c): # {{{1
    "Get the pybullet constant for the given connect server type"
    if c == p.SHARED_MEMORY:
      return "SHARED_MEMORY"
    if c == p.DIRECT:
      return "DIRECT"
    if c == p.GUI:
      return "GUI"
    if c == p.UDP:
      return "UDP"
    if c == p.TCP:
      return "TCP"
    if c == p.GUI_SERVER:
      return "GUI_SERVER"
    if c == p.GUI_MAIN_THREAD:
      return "GUI_MAIN_THREAD"
    if c == p.SHARED_MEMORY_SERVER:
      return "SHARED_MEMORY_SERVER"
    if c == p.SHARED_MEMORY_GUI:
      return "SHARED_MEMORY_GUI"
    return "UNKNOWN"
  # 1}}}

  @staticmethod
  def getGeometryName(geom): # {{{1
    "Get the pybullet constant for the given geometry"
    if geom == p.GEOM_SPHERE:
      return "GEOM_SPHERE"
    if geom == p.GEOM_BOX:
      return "GEOM_BOX"
    if geom == p.GEOM_CYLINDER:
      return "GEOM_CYLINDER"
    if geom == p.GEOM_MESH:
      return "GEOM_MESH"
    if geom == p.GEOM_PLANE:
      return "GEOM_PLANE"
    if geom == p.GEOM_CAPSULE:
      return "GEOM_CAPSULE"
    if geom == p.GEOM_HEIGHTFIELD:
      return "GEOM_HEIGHTFIELD"
    if geom == p.GEOM_FORCE_CONCAVE_TRIMESH:
      return "GEOM_FORCE_CONCAVE_TRIMESH"
    if geom == p.GEOM_CONCAVE_INTERNAL_EDGE:
      return "GEOM_CONCAVE_INTERNAL_EDGE"
    return "GEOM_UNKNOWN"
  # 1}}}

  @staticmethod
  def getGeomByName(geom): # {{{1
    "Return the value of the given geometry by name"
    all_geom = [v for v in dir(p) if v.startswith("GEOM_")]
    if geom in all_geom:
      return getattr(p, geom)
    elif "GEOM_" + geom in all_geom:
      return getattr(p, "GEOM_" + geom)
    return "GEOM_UNKNOWN"
  # 1}}}

  @staticmethod
  def getCovName(cov): # {{{1
    "Get the pybullet constant for the given debug configuration option"
    if cov == p.COV_ENABLE_GUI:
      return "COV_ENABLE_GUI"
    if cov == p.COV_ENABLE_SHADOWS:
      return "COV_ENABLE_SHADOWS"
    if cov == p.COV_ENABLE_WIREFRAME:
      return "COV_ENABLE_WIREFRAME"
    if cov == p.COV_ENABLE_VR_TELEPORTING:
      return "COV_ENABLE_VR_TELEPORTING"
    if cov == p.COV_ENABLE_VR_PICKING:
      return "COV_ENABLE_VR_PICKING"
    if cov == p.COV_ENABLE_VR_RENDER_CONTROLLERS:
      return "COV_ENABLE_VR_RENDER_CONTROLLERS"
    if cov == p.COV_ENABLE_RENDERING:
      return "COV_ENABLE_RENDERING"
    if cov == p.COV_ENABLE_KEYBOARD_SHORTCUTS:
      return "COV_ENABLE_KEYBOARD_SHORTCUTS"
    if cov == p.COV_ENABLE_MOUSE_PICKING:
      return "COV_ENABLE_MOUSE_PICKING"
    if cov == p.COV_ENABLE_Y_AXIS_UP:
      return "COV_ENABLE_Y_AXIS_UP"
    if cov == p.COV_ENABLE_TINY_RENDERER:
      return "COV_ENABLE_TINY_RENDERER"
    if cov == p.COV_ENABLE_RGB_BUFFER_PREVIEW:
      return "COV_ENABLE_RGB_BUFFER_PREVIEW"
    if cov == p.COV_ENABLE_DEPTH_BUFFER_PREVIEW:
      return "COV_ENABLE_DEPTH_BUFFER_PREVIEW"
    if cov == p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW:
      return "COV_ENABLE_SEGMENTATION_MARK_PREVIEW"
    if cov == p.COV_ENABLE_PLANAR_REFLECTION:
      return "COV_ENABLE_PLANAR_REFLECTION"
    if cov == p.COV_ENABLE_SINGLE_STEP_RENDERING:
      return "COV_ENABLE_SINGLE_STEP_RENDERING"
    return "COV_UNKNOWN"
  # 1}}}

  @staticmethod
  def getCovByName(cov): # {{{1
    "Return the value of the given debug configuration option by name"
    all_cov = [v for v in dir(p) if v.startswith("COV_")]
    if cov in all_cov:
      return getattr(p, cov)
    elif "COV_" + cov in all_cov:
      return getattr(p, "COV_" + cov)
    return "COV_UNKNOWN"
  # 1}}}
# 0}}}

def _merge(v1, v2): # {{{0
  "Merge two collections into one"
  v = None
  if type(v1) == type(v2):
    if isinstance(v1, set):
      v = v1.union(v2)
    elif isinstance(v1, dict):
      v = {}
      v.update(v1)
      v.update(v2)
    else:
      v = v1 + v2
  elif v1 is None:
    v = v2
  elif v2 is None:
    v = v1
  return v
# 0}}}

def _or(v, d): # {{{0
  "Either v or d, whichever is not None first"
  if v is not None:
    return v
  return d
# 0}}}

class BulletAppBase(object):
  "Base class for pybullet simulations"
  def __init__(self, connect=False, # {{{1
               engineParams=None,
               connectArgs=None,
               createBodyArgs=None,
               createBodyKwargs=None,
               createCollisionArgs=None,
               createCollisionKwargs=None,
               createVisualShapeArgs=None,
               createVisualShapeKwargs=None,
               **kwargs):
    """
    Extra keyword arguments extracted from **kwargs:
      debug         enable some debug output (False)
      plugins       tuple of plugin paths or paths:suffixes to load
      gravityX      default gravity for x (0) (between -20 and 20)
      gravityY      default gravity for y (0) (between -20 and 20)
      gravityZ      default gravity for z (-9.8) (between -20 and 20)
    """
    self._args = kwargs
    self._debug = getRemove(kwargs, "debug", False)
    self._plugins = getRemove(kwargs, "plugins", ())
    self._loadedPlugins = {}
    self._defaultGravity = {
      "x": getRemove(kwargs, "gravityX", 0, float),
      "y": getRemove(kwargs, "gravityY", 0, float),
      "z": getRemove(kwargs, "gravityZ", -9.8, float)
    }
    self._server = None
    self._stateLogger = None
    self._buttons = {}
    self._bodies = set()
    self._colliders = set()
    self._visualShapes = set()
    self._textures = {}
    self._covValues = {}
    self._debugConfigs = {}
    self._engineParams = _or(engineParams, {})
    self._connectArgs = _or(connectArgs, "")
    self._createBodyArgs = (
        _or(createBodyArgs, ()),
        _or(createBodyKwargs, {}))
    self._createCollisionArgs = (
        _or(createCollisionArgs, ()),
        _or(createCollisionKwargs, {}))
    self._createVisualShapeArgs = (
        _or(createVisualShapeArgs, ()),
        _or(createVisualShapeKwargs, {}))
    if connect:
      self.connect()
  # 1}}}

  def hasArg(self, arg): # {{{1
    "Return True if arg has a value specified"
    return arg in self._args
  # 1}}}

  def getArg(self, arg, dflt=None, typecls=None): # {{{1
    "Obtain the requested engine argument"
    val = self._args.get(arg, dflt)
    if typecls is not None:
      return typecls(val)
    return val
  # 1}}}

  def connect(self, args=None): # {{{1
    "Start the shared memory server and GUI client"
    # Collect arguments to pass to connect()
    def parseArgObj(argobj):
      if isinstance(argobj, basestring):
        return argobj.split()
      elif argobj:
        return argobj
      return []
    cargs = []
    cargs.extend(parseArgObj(self._connectArgs))
    cargs.extend(parseArgObj(args))
    cargs = " ".join(cargs)

    # Start and connect to server
    logger.debug("Starting GUI server: {}".format(cargs))
    self._server = p.connect(p.GUI_SERVER, options=cargs)
    p.setInternalSimFlags(0)
    if pyb3.HAS_PYB3_DATA:
      p.setAdditionalSearchPath(pyb3.pybullet_data.getDataPath())

    if self._plugins:
      for plugin in self._plugins:
        if ":" in plugin:
          path, prefix = plugin.split(":", 1)
        else:
          path, prefix = plugin, ""
        self._loadedPlugins[path] = p.loadPlugin(path, prefix)

    # Add standard debug config sliders
    self.addSlider("Gravity X", -20, 20, self._defaultGravity["x"])
    self.addSlider("Gravity Y", -20, 20, self._defaultGravity["y"])
    self.addSlider("Gravity Z", -20, 20, self._defaultGravity["z"])
  # 1}}}

  def isConnected(self): # {{{1
    "Return whether or not we're connected"
    return p.isConnected()
  # 1}}}

  # Debug interface functions:

  def toggleCov(self, cov, forceValue=None): # {{{1
    "Toggle a debug configuration option"
    if forceValue is not None:
      self._covValues[cov] = forceValue
    elif cov in self._covValues:
      self._covValues[cov] = not self._covValues[cov]
    else:
      self._covValues[cov] = False
    cv = self._covValues[cov]
    logger.debug("Setting COV {} to {}".format(B3.getCovName(cov), cv))
    p.configureDebugVisualizer(cov, 1 if cv else 0)
    return cv
  # 1}}}

  def covActive(self, cov): # {{{1
    "Return whether or not the debug configuration option is active"
    return self._covValues.get(cov, False)
  # 1}}}

  def addSlider(self, name, min, max, dflt=None): # {{{1
    "Add user debug parameter"
    start = min if dflt is None else dflt
    self._debugConfigs[name] = p.addUserDebugParameter(name, min, max, start)
  # 1}}}

  def getSlider(self, name, tp=float): # {{{1
    "Read user debug parameter"
    return tp(p.readUserDebugParameter(self._debugConfigs[name]))
  # 1}}}

  def addButton(self, name): # {{{1
    "Add a button parameter"
    try:
      self._debugConfigs[name] = p.addUserDebugButton(name)
    except NotImplementedError as e:
      logger.error(e)
  # 1}}}

  def getButton(self, name): # {{{1
    "If the button has been triggered since last reset"
    try:
      return p.readUserDebugButton(self._debugConfigs[name])
    except NotImplementedError as e:
      logger.error(e)
    except p.error as e:
      logger.error(e)
  # 1}}}

  def resetButton(self, name): # {{{1
    "Reset a button to un-triggered"
    try:
      p.resetUserDebugButton(self._debugConfigs[name])
    except NotImplementedError as e:
      logger.error(e)
    except p.error as e:
      logger.error(e)
  # 1}}}

  def addButtonEvent(self, name, func): # {{{0
    """Add a button which calls func() when pressed. Requires self.tick() be
    called in order to work."""
    self._buttons[name] = {
      "name": name,
      "id": self.addButton(name),
      "func": func
    }
  # 0}}}

  def drawLine(self, p1, p2, thickness=1, color=C3_BLK): # {{{1
    "Draw a line from p1 to p2"
    p.addUserDebugLine(p1, p2, color, thickness)
  # 1}}}

  def drawText(self, t, pt, size=None, color=C3_BLK): # {{{1
    "Draw text at the given point"
    kwargs = {}
    kwargs["textColorRGB"] = color
    if size is not None:
      kwargs["textSize"] = size
    p.addUserDebugText(t, pt, **kwargs)
  # 1}}}

  # Main loop functions:

  def updateGravity(self): # {{{1
    "Apply gravity debug configuration values"
    gx = self.getSlider("Gravity X")
    gy = self.getSlider("Gravity Y")
    gz = self.getSlider("Gravity Z")
    p.setGravity(gx, gy, gz)
  # 1}}}

  def reset(self): # {{{1
    "Reset simulation"
    p.resetSimulation()
    iterations = self.getArg("numSolverIterations", 10)
    p.setPhysicsEngineParameter(numSolverIterations=iterations)
    contactBreaking = self.getArg("contactBreakingThreshold", 0.001)
    p.setPhysicsEngineParameter(contactBreakingThreshold=contactBreaking)
    if self._engineParams:
      logger.debug("Applying engine parameters: {}".format(self._engineParams))
      p.setPhysicsEngineParameter(**self._engineParams)
    try:
      if self._debug or "--verbose" in self._connectArgs:
        p.setPhysicsEngineParameter(verboseMode=1)
    except (TypeError, p.error) as e:
      # Unimplemented; ignore it
      pass
    self._bodies = set()
    self._colliders = set()
    self._visualShapes = set()
    if self._stateLogger is not None:
      p.stopStateLogging(self._stateLogger)
      p._stateLogger = None
    if self.hasArg("mp4"):
      f = self.getArg("mp4")
      self._stateLogger = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, f)
  # 1}}}

  def setRendering(self, enable=True): # {{{1
    "Enable or disable rendering"
    if enable:
      p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    else:
      p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
  # 1}}}

  def setRealTime(self, enable=True): # {{{1
    "Enable or disable real-time simulation"
    if enable:
      p.setRealTimeSimulation(1)
    else:
      p.setRealTimeSimulation(0)
  # 1}}}

  def toggleSim(self): # {{{1
    "Toggle real-time simulation"
    s = p.getPhysicsEngineParameters()
    if s["useRealTimeSimulation"]:
      self.setRealTime(False)
    else:
      self.setRealTime(True)
  # 1}}}

  def stepSim(self): # {{{1
    "Advance the simulation by one step"
    p.stepSimulation()
  # 1}}}

  def tick(self): # {{{0
    "Process main-loop events: gravity, button functions, etc"
    self.updateGravity()
    for bname in self._buttons:
      if self.getButton(bname):
        self._buttons[bname]["func"]()
        self.resetButton(bname)
  # 0}}}

  # Asset functions:

  def saveBullet(self, fname): # {{{1
    "Save a snapshot of the world"
    app.saveBullet(fname)
  # 1}}}

  def loadBullet(self, fname): # {{{1
    "Load a snapshot of the world"
    app.loadBullet(fname)
  # 1}}}

  def getTexture(self, path): # {{{1
    "Load and return a unique ID for the given texture file"
    if path not in self._textures:
      self._textures[path] = p.loadTexture(path)
    return self._textures[path]
  # 1}}}

  # Camera functions:

  def getCamera(self, val=None): # {{{1
    """
    Return information on the debug camera

    For movement by dt:
    cam = target - direction * distance
    forward = (target - cam).normalize() * dt
    right = np.cross(forward, up).normalize() * dt
    up = cam.up * dt
    """
    camera = p.getDebugVisualizerCamera()
    ci = {
      "width": np.int(camera[0]),
      "height": np.int(camera[1]),
      "viewMat": np.mat(camera[2]),
      "projMat": np.mat(camera[3]),
      "camUp": np.array(camera[4]),
      "camForward": np.array(camera[5]),
      "horizVec": np.array(camera[6]),
      "vertVec": np.array(camera[7]),
      CAM_AXIS_YAW: np.float(camera[8]),
      CAM_AXIS_PITCH: np.float(camera[9]),
      CAM_AXIS_DIST: np.float(camera[10]),
      CAM_AXIS_TARGET: np.array(camera[11]),
    }
    if val is not None:
      return ci[val]
    return ci
  # 1}}}

  def setCamera(self, **kwargs): # {{{1
    "Set the camera axes directly to the values given"
    ci = self.getCamera()
    vals = {
      CAM_AXIS_PITCH: kwargs.get(CAM_AXIS_PITCH, ci[CAM_AXIS_PITCH]),
      CAM_AXIS_YAW: kwargs.get(CAM_AXIS_YAW, ci[CAM_AXIS_YAW]),
      CAM_AXIS_DIST: kwargs.get(CAM_AXIS_DIST, ci[CAM_AXIS_DIST]),
      CAM_AXIS_TARGET: kwargs.get(CAM_AXIS_TARGET, ci[CAM_AXIS_TARGET])
    }
    p.resetDebugVisualizerCamera(
      cameraDistance=vals[CAM_AXIS_DIST],
      cameraYaw=vals[CAM_AXIS_YAW],
      cameraPitch=vals[CAM_AXIS_PITCH],
      cameraTargetPosition=vals[CAM_AXIS_TARGET])
  # 1}}}

  def moveCamera(self, **kwargs): # {{{1
    "Adjust the camera by the values given"
    ci = self.getCamera()
    self.setCamera(**applyAxisMovements(ci, **kwargs))
  # 1}}}

  def getPitch(self): # {{{1
    "Get value of CAM_AXIS_PITCH"
    return self.getCamera(CAM_AXIS_PITCH)
  # 1}}}

  def setPitch(self, val): # {{{1
    "Set value of CAM_AXIS_PITCH"
    self.setCamera(**{CAM_AXIS_PITCH: val})
  # 1}}}

  def getYaw(self): # {{{1
    "Get value of CAM_AXIS_YAW"
    return self.getCamera(CAM_AXIS_YAW)
  # 1}}}

  def setYaw(self, val): # {{{1
    "Set value of CAM_AXIS_YAW"
    self.setCamera(**{CAM_AXIS_YAW: val})
  # 1}}}

  def getTargetDistance(self): # {{{1
    "Get value of CAM_AXIS_DIST"
    return self.getCamera(CAM_AXIS_DIST)
  # 1}}}

  def setTargetDistance(self, val): # {{{1
    "Set value of CAM_AXIS_DIST"
    self.setCamera(**{CAM_AXIS_DIST: val})
  # 1}}}

  def getTargetPos(self): # {{{1
    "Get value of CAM_AXIS_TARGET"
    return self.getCamera(CAM_AXIS_TARGET)
  # 1}}}

  def setTargetPos(self, val): # {{{1
    "Set value of CAM_AXIS_TARGET"
    self.setCamera(**{CAM_AXIS_TARGET: val})
  # 1}}}

  def getCameraPos(self): # {{{1
    "Get the poisition of the camera in world coordinates"
    ci = self.getCamera()
    return ci[CAM_AXIS_TARGET] - ci["camForward"] * ci[CAM_AXIS_DIST]
  # 1}}}

  # Body information:

  def getBodyDynamics(self, oid, link=-1): # {{{1
    "Return dynamics information about the object"
    oi = p.getDynamicsInfo(oid, -1)
    return {
      "mass": np.float(oi[0]),
      "lateralFrictionCoeff": np.float(oi[1]),
      "inertiaDiagonal": np.array(oi[2]),
      "inertiaPosition": np.array(oi[3]),
      "inertiaOrientation": np.array(oi[4]),
      "restitution": np.float(oi[5]),
      "rollingFriction": np.float(oi[6]),
      "spinningFriction": np.float(oi[7]),
      "contactDamping": np.float(oi[8]),
      "contactStiffness": np.float(oi[9])
    }
  # 1}}}

  def getAABB(self, oid): # {{{1
    "Return the object's AABB as two points"
    aabb = p.getAABB(oid)
    return np.array(aabb[0]), np.array(aabb[1])
  # 1}}}

  def getBodyVelocity(self, oid): # {{{1
    "Return the object's linear and angular velocity"
    vel = p.getBaseVelocity(oid)
    return np.array(vel[0]), np.array(vel[1])
  # 1}}}

  def getBodyCollision(self, oid, link=-1, failSoft=False): # {{{1
    "Return information on the body's collider"
    colliders = []
    try:
      for entry in p.getCollisionShapeData(oid, link):
        colliders.append({
          "geomType": entry[2],
          "dimensions": np.array(entry[3]),
          "meshFileName": entry[4],
          "localFramePosition": np.array(entry[5]),
          "localFrameOrientation": np.array(entry[6]),
        })
    except p.error as e:
      if not failSoft:
        raise
    return colliders
  # 1}}}

  def getBodyShapes(self, oid): # {{{1
    "Return a list of the body's shape data entries"
    shapes = []
    for entry in p.getVisualShapeData(oid):
      shapes.append({
        "geomType": entry[2],
        "size": np.array(entry[3]),
        "meshFileName": entry[4],
        "localFramePosition": np.array(entry[5]),
        "localFrameOrientation": np.array(entry[6]),
        "rgbaColor": entry[7]
      })
    return shapes
  # 1}}}

  def getFullObjectInfo(self, oid, link=-1, failSoft=False): # {{{1
    "Return everything known about the object"
    bi = {}
    bi.update(self.getBodyDynamics(oid, link))
    bi["AABB"] = self.getAABB(oid)
    bi["linearVelocity"], bi["angularVelocity"] = self.getBodyVelocity(oid)
    bi["collisionData"] = self.getBodyCollision(oid, link, failSoft)
    bi["visualShapeData"] = self.getBodyShapes(oid)
    return bi
  # 1}}}

  def getBodies(self): # {{{1
    "Iterate over every body (returns a body's index)"
    for bid in range(p.getNumBodies()):
      yield bid
  # 1}}}

  # Object creation:

  def createCollision(self, *args, **kwargs): # {{{1
    """Create a collision shape. All arguments are passed to pybullet
    createCollisionShape."""
    cargs = _merge(self._createCollisionArgs[0], args)
    ckwargs = _merge(self._createCollisionArgs[1], kwargs)
    ret = p.createCollisionShape(*cargs, **ckwargs)
    assertSuccess(ret, "createCollision")
    self._colliders.add(ret)
    return ret
  # 1}}}

  def createMultiBody(self, *args, **kwargs): # {{{1
    """Create a MultiBody.
    Keyword arguments:
      restitution   object restitution (0)
      color         object color
      rgba          object color (if "color" is not present)
      texture       object texture filename
    All other arguments are passed to pybullet createMultiBody.
    """
    restitution = getRemove(kwargs, "restitution", 0, float)
    color = getRemove(kwargs, "color", None)
    if color is None:
      color = getRemove(kwargs, "rgba", None)
    texture = getRemove(kwargs, "texture", None)
    bargs = _merge(self._createBodyArgs[0], args)
    bkwargs = _merge(self._createBodyArgs[1], kwargs)
    ret = p.createMultiBody(*bargs, **bkwargs)
    assertSuccess(ret, "createMultiBody")
    if restitution != 0:
      p.changeDynamics(ret, -1, restitution=restitution)
    if color is not None:
      self.setBodyColor(ret, color)
    if texture is not None:
      self.setBodyTexture(ret, texture)
    self._bodies.add(ret)
    return ret
  # 1}}}

  def createVisualShape(self, *args, **kwargs): # {{{1
    """Create a visual shape. All arguments are passed to pybullet
    createVisualShae.
    """
    vargs = _merge(self._createVisualShapeArgs[0], args)
    vkwargs = _merge(self._createVisualShapeArgs[1], kwargs)
    ret = p.createVisualShape(*vargs, **vkwargs)
    assertSuccess(ret, "createVisualShape")
    self._visualShapes.add(ret)
    return ret
  # 1}}}

  def createSphere(self, mass, pos, # {{{1
                   radius=None,
                   collider=None,
                   visualShape=None,
                   **kwargs):
    "Insert a new sphere into the world"
    if not (collider is None) ^ (radius is None):
      raise ValueError("Must pass either radius or collider")
    elif collider is not None:
      cid = collider
    elif radius is not None:
      cid = self.createCollision(p.GEOM_SPHERE, radius)
    vid = _or(visualShape, -1)
    return self.createMultiBody(mass, cid, vid, pos, **kwargs)
  # 1}}}

  def createBox(self, mass, pos, exts, **kwargs): # {{{1
    "Insert a new box into the world"
    coll = self.createCollision(p.GEOM_BOX, halfExtents=exts)
    obj = self.createMultiBody(mass, coll, basePosition=pos, **kwargs)
    return obj
  # 1}}}

  def createCapsule(self, mass, pos, # {{{1
                    radii=None,
                    collider=None,
                    visualShape=None,
                    **kwargs):
    "Insert a new capsule into the world"
    if not (collider is None) ^ (radii is None):
      raise ValueError("Must pass either radii or collider")
    elif collider is not None:
      cid = collider
    elif radii is not None:
      cid = self.createCollision(p.GEOM_CAPSULE, radius=radii[0], height=radii[1])
    vid = _or(visualShape, -1)
    return self.createMultiBody(mass, cid, vid, pos, **kwargs)
  # 1}}}

  def createCylinder(self, mass, pos, # {{{1
                     radii=None,
                     collider=None,
                     visualShape=None,
                     **kwargs):
    "Insert a new cylinder into the world"
    if not (collider is None) ^ (radii is None):
      raise ValueError("Must pass either radii or collider")
    elif collider is not None:
      cid = collider
    elif radii is not None:
      cid = self.createCollision(p.GEOM_CYLINDER, radius=radii[0], height=radii[1])
    vid = _or(visualShape, -1)
    return self.createMultiBody(mass, cid, vid, pos, **kwargs)
  # 1}}}

  def createPlane(self, pos, **kwargs): # {{{1
    "Create a plane at the given position"
    cid = self.createCollision(p.GEOM_PLANE)
    return self.createMultiBody(0, cid, basePosition=pos, **kwargs)
  # 1}}}

  def loadSoftBody(self, path, # {{{1
                   pos=None,
                   orn=None,
                   scale=None,
                   mass=None,
                   margin=None):
    "Load a soft body into the world"
    if not hasattr(p, "loadSoftBody"):
      logger.error("pybullet error: loadSoftBody is not available\n")
      return None
    args = {}
    if pos is not None:
      args["basePosition"] = pos
    if orn is not None:
      args["baseOrientation"] = orn
    if scale is not None:
      args["scale"] = scale
    if mass is not None:
      args["mass"] = mass
    if margin is not None:
      args["collisionMargin"] = margin
    ret = assertSuccess(p.loadSoftBody(path, **args), "loadSoftBody")
    self._bodies.add(ret)
    return ret
  # 1}}}

  def loadURDF(self, path, *args, **kwargs): # {{{1
    "Load object defined by the URDF file"
    uargs = tuple(args)
    ukwargs = dict(kwargs)
    if self._createBodyArgs[1].get("useMaximalCoordinates", False):
      ukwargs["useMaximalCoordinates"] = True
    idx = p.loadURDF(path, *uargs, **ukwargs)
    assertSuccess(idx, "loadURDF")
    self._bodies.add(idx)
    return idx
  # 1}}}

  # Object management:

  def removeBody(self, bodyId=None, bodyIndex=None): # {{{0
    "Remove a body by either ID or index"
    bid = None
    if bodyId is not None:
      bid = bodyId
    elif bodyIndex is not None:
      bid = p.getBodyUniqueId(bodyIndex)
    else:
      raise ValueError("Must specify a body to remove")
    if bid in self._bodies:
      self._bodies.remove(bid)
    p.removeBody(bid)
  # 0}}}

  def removeLastBody(self): # {{{0
    "Remove the most recently added body. Will not remove the first body."
    nr = p.getNumBodies()
    if nr == 0:
      logger.warning("No body left to remove")
    else:
      bid = p.getBodyUniqueId(nr-1)
      if nr == 1:
        logger.warning("Refusing to remove last object {}".format(bid))
      else:
        self.removeBody(bodyId=bid)
  # 0}}}

  def setBodyColor(self, idx, rgba, link=-1): # {{{1
    "Change a body's color to the given 3-tuple or 4-tuple"
    if isinstance(rgba, basestring):
      color = parseColor(rgba)
    else:
      color = list(rgba)
    if len(color) == 3:
      color.append(1)
    p.changeVisualShape(idx, link, rgbaColor=color)
  # 1}}}

  def setBodyTexture(self, idx, path, link=-1): # {{{1
    "Change a body's texture to use the given texture file"
    p.changeVisualShape(idx, link, textureUniqueId=self.getTexture(path))
  # 1}}}

  def setBodyRestitution(self, idx, restitution, link=-1): # {{{1
    p.changeDynamics(idx, link, restitution=restitution)
  # 1}}}

# vim: set ts=2 sts=2 sw=2 et:

