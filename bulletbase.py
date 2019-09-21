#!/usr/bin/env python

"""
Base and utility classes for a pybullet simulation
"""

# TODO:
# readUserDebugButton
# clearUserDebugButton (or some way to reset it)

from pyb3 import pybullet as p
from utility import *

# Only export the classes, not the modules
__all__ = ["B3", "BulletAppBase"]

class B3(object): # {{{0
  "Bullet constants and functions for those constants"

  @staticmethod
  def getGeometryName(gom): # {{{1
    "Get the pybullet constant for the given geometry"
    if geom == p.GEOM_FORCE_CONCAVE_TRIMESH:
      return "GEOM_FORCE_CONCAVE_TRIMESH"
    if geom == p.GEOM_CONCAVE_INTERNAL_EDGE:
      return "GEOM_CONCAVE_INTERNAL_EDGE"
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
  return v
# 0}}}

def _or(v, d): # {{{0
  "Either v or d, whichever is not None first"
  if v is not None:
    return v
  return d
# 0}}}

class BulletAppBase(object): # {{{0
  "Base class for pybullet simulations"
  def __init__(self, connect=False, # {{{1
               connectArgs=None,
               connectKwargs=None,
               createBodyArgs=None,
               createBodyKwargs=None,
               createCollisionArgs=None,
               createCollisionKwargs=None,
               createVisualShapeArgs=None,
               createVisualShapeKwargs=None,
               **kwargs):
    """
    Extra keyword arguments:
      debug:        enable some debug output (False)
      gravityX:     default gravity for x (0)
      gravityY:     default gravity for y (0)
      gravityZ:     default gravity for z (-8)
    """
    self._debug = getRemove(kwargs, "debug", False)
    self._defaultGravity = {
      "x": getRemove(kwargs, "gravityX", 0, float),
      "y": getRemove(kwargs, "gravityY", 0, float),
      "z": getRemove(kwargs, "gravityZ", -9.8, float)
    }
    self._server = None
    self._bodies = set()
    self._colliders = set()
    self._visualShapes = set()
    self._covValues = {}
    self._debugConfigs = {}
    self._args = kwargs
    self._connectArgs = (
        _or(connectArgs, ()),
        _or(connectKwargs, {}))
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

  def debug(self, m): # {{{1
    "Print a debug message, if debugging is enabled"
    if self._debug:
      sys.stderr.write("BulletAppBase: DEBUG: ")
      sys.stderr.write(m.strip())
      sys.stderr.write("\n")
  # 1}}}

  def toggleCov(self, cov, forceValue=None): # {{{1
    "Toggle a debug configuration option"
    if forceValue is not None:
      self._covValues[cov] = forceValue
    elif cov in self._covValues:
      self._covValues[cov] = not self._covValues[cov]
    else:
      self._covValues[cov] = False
    self.debug("Setting COV {} to {}".format(B3.getCovName(cov), self._covValues[cov]))
    p.configureDebugVisualizer(cov, 1 if self._covValues[cov] else 0)
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

  def _do_connect(self, method, args=None): # {{{1
    "Private: connect to the Bullet server"
    if args is None:
      result = p.connect(method)
      self.debug("connect({})".format(method, result))
    else:
      result = p.connect(method, options=args)
      self.debug("connect({}, options={!r}) = {}".format(method, args, result))
    return result
  # 1}}}

  def isConnected(self): # {{{1
    "Return whether or not we're connected"
    return p.isConnected()
  # 1}}}

  def connect(self, *args, **kwargs): # {{{1
    "Connect to the Bullet server, first trying shared memory, then GUI"
    self.debug("request connect(*{}, **{})".format(args, kwargs))
    cargs = _merge(self._connectArgs[0], args)
    ckwargs = _merge(self._connectArgs[1], kwargs)
    self._server = self._do_connect(p.SHARED_MEMORY, *args, **kwargs)
    if self._server < 0:
      self._server = self._do_connect(p.GUI, *args, **kwargs)
    p.setInternalSimFlags(0)
    # Add standard debug config sliders
    self.addSlider("gravityX", -10, 10, self._defaultGravity["x"])
    self.addSlider("gravityY", -10, 10, self._defaultGravity["y"])
    self.addSlider("gravityZ", -10, 10, self._defaultGravity["z"])
  # 1}}}

  def saveBullet(self, fname): # {{{1
    "Save a snapshot of the world"
    app.saveBullet(fname)
  # 1}}}

  def loadBullet(self, fname): # {{{1
    "Load a snapshot of the world"
    app.loadBullet(fname)
  # 1}}}

  def addSlider(self, name, min, max, dflt=None): # {{{1
    "Add user debug parameter"
    ival = min
    if dflt is not None:
      ival = dflt
    self._debugConfigs[name] = p.addUserDebugParameter(name, min, max, ival)
    self.debug("addDebug({!r}, min={}, max={}, ival={})".format(name, min, max, ival))
  # 1}}}

  def getSlider(self, name, tp=float): # {{{1
    "Read user debug parameter"
    return tp(p.readUserDebugParameter(self._debugConfigs[name]))
  # 1}}}

  def addButton(self, name, state=False): # {{{1
    "Add a button parameter"
    self._debugConfigs[name] = p.addUserDebugButton(name, isTrigger=state)
    self.debug("addButton({!r})".format(name))
  # 1}}}

  def getButton(self, name): # {{{1
    "Whether or not a button is currently pressed"
    # TODO
    return None
    # return p.readUserDebugButton(self._debugConfigs[name])
  # 1}}}

  def updateGravity(self): # {{{1
    "Apply gravity debug configuration values"
    gx = self.getSlider("gravityX")
    gy = self.getSlider("gravityY")
    gz = self.getSlider("gravityZ")
    p.setGravity(gx, gy, gz)
  # 1}}}

  def loadURDF(self, path, *args, **kwargs): # {{{1
    "Load object defined by the URDF file"
    idx = p.loadURDF(path, useMaximalCoordinates=True, *args, **kwargs)
    assertSuccess(idx, "createVisualShape")
    self._bodies.add(idx)
    return idx
  # 1}}}

  def reset(self): # {{{1
    "Reset simulation"
    p.resetSimulation()
    iterations = self.getArg("numSolverIterations", 10)
    p.setPhysicsEngineParameter(numSolverIterations = iterations)
    contactBreaking = self.getArg("contactBreakingThreshold", 0.001)
    p.setPhysicsEngineParameter(contactBreakingThreshold = contactBreaking)
    self._bodies = set()
    self._colliders = set()
    self._visualShapes = set()
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
    vals = applyAxisMovements(ci, **kwargs)
    p.resetDebugVisualizerCamera(
        cameraDistance=vals[CAM_AXIS_DIST],
        cameraYaw=vals[CAM_AXIS_YAW],
        cameraPitch=vals[CAM_AXIS_PITCH],
        cameraTargetPosition=vals[CAM_AXIS_TARGET])
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

  def getDistanceToTarget(self): # {{{1
    "Get value of CAM_AXIS_DIST"
    return self.getCamera(CAM_AXIS_DIST)
  # 1}}}

  def setDistanceToTarget(self, val): # {{{1
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

  def getBodyCollision(self, oid, link=-1): # {{{1
    "Return information on the body's collider"
    colliders = []
    for entry in p.getCollisionShapeData(oid, -1):
      colliders.append({
        "geomType": entry[2],
        "dimensions": np.array(entry[3]),
        "meshFileName": entry[4],
        "localFramePosition": np.array(entry[5]),
        "localFrameOrientation": np.array(entry[6]),
      })
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

  def getFullObjectInfo(self, oid): # {{{1
    "Return everything known about the object"
    bi = {}
    bi.update(self.getBodyDynamics(oid))
    bi["AABB"] = self.getAABB(oid)
    bi["linearVelocity"], bi["angularVelocity"] = self.getBodyVelocity(oid)
    bi["collisionData"] = self.getBodyCollision(oid)
    bi["visualShapeData"] = self.getBodyShapes(oid)
    return bi
  # 1}}}

  def getBodies(self): # {{{1
    "Iterate over every body (returns a body's unique ID)"
    for bid in range(p.getNumBodies()):
      yield bid
  # 1}}}

  # FIXME: Doesn't work somehow?
  def getWorldExtrema(self): # {{{1
    "Return the smallest AABB containing all objects"
    worldMin = [None, None, None]
    worldMax = [None, None, None]
    points = []
    for a, b in [self.getAABB(bidx) for bidx in self.getBodies()]:
      points.append(a)
      points.append(b)
    worldMin[0] = min(p[0] for p in points)
    worldMax[0] = max(p[0] for p in points)
    worldMin[1] = min(p[1] for p in points)
    worldMax[1] = max(p[1] for p in points)
    worldMin[2] = min(p[2] for p in points)
    worldMax[2] = max(p[2] for p in points)
    return (np.array(worldMin), np.array(worldMax))
  # 1}}}

  def createCollision(self, *args, **kwargs): # {{{1
    "Create a collision shape, asserting success"
    cargs = _merge(self._createCollisionArgs[0], args)
    ckwargs = _merge(self._createCollisionArgs[1], kwargs)
    ret = p.createCollisionShape(*cargs, **ckwargs)
    assertSuccess(ret, "createCollision")
    self._colliders.add(ret)
    return ret
  # 1}}}

  def createMultiBody(self, *args, **kwargs): # {{{1
    "Create a MultiBody, asserting success"
    r = getRemove(kwargs, "restitution", 0, float)
    bargs = _merge(self._createBodyArgs[0], args)
    bkwargs = _merge(self._createBodyArgs[1], kwargs)
    b = p.createMultiBody(*bargs, **bkwargs)
    assertSuccess(b, "createMultiBody")
    if r != 0:
      p.changeDynamics(b, -1, restitution=r)
    self._bodies.add(b)
    return b
  # 1}}}

  def createVisualShape(self, *args, **kwargs): # {{{1
    "Create a visual shape, asserting success"
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
    elif collider is None:
      cid = self.createCollision(p.GEOM_SPHERE, radius)
    elif radius is None:
      cid = collider
    vid = -1 if visualShape is None else visualShape
    return self.createMultiBody(mass, cid, vid, pos, **kwargs)
  # 1}}}

  def createBox(self, mass, pos, exts, rgba=None, **kwargs): # {{{1
    """
    Create a box with the given half extents, at the given position, and
    with the given color
    """
    coll = self.createCollision(p.GEOM_BOX, halfExtents=exts)
    obj = self.createMultiBody(mass, coll, basePosition=pos, **kwargs)
    if rgba is not None:
      self.setBodyColor(obj, rgba)
    return obj
  # 1}}}

  def createCapsule(self, mass, pos, radii=None, collider=None, # {{{1
                    visualShape=None, **kwargs):
    "Insert a new capsule into the world"
    if not (collider is None) ^ (radii is None or len(radii) != 2):
      raise ValueError("Must pass either radii or collider")
    elif collider is not None:
      cid = collider
    elif radii is not None:
      cid = self.createCollision(p.GEOM_CAPSULE, radius=radii[0], height=radii[1])
    vid = -1 if visualShape is None else visualShape
    return self.createMultiBody(mass, cid, vid, pos, **kwargs)
  # 1}}}

  def loadSoftBody(self, path, pos=None, orn=None, scale=None, mass=None): # {{{1
    "Load a soft body into the world"
    if not hasattr(p, "loadSoftBody"):
      sys.stderr.write("pybullet error: loadSoftBody is not available\n")
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
    ret = p.loadSoftBody(path, collisionMargin=0.5, **args)
    assertSuccess(ret, "loadSoftBody")
    self._bodies.add(ret)
    return ret
  # 1}}}

  def setBodyColor(self, idx, rgba): # {{{1
    "Change a body's color to the given 4-tuple"
    p.changeVisualShape(idx, -1, rgbaColor=list(rgba))
  # 1}}}

  def drawLine(self, p1, p2, thickness=1, color=None): # {{{1
    "Draw a line from p1 to p2"
    c = color
    if color is None:
      c = [0, 0, 0]
    p.addUserDebugLine(p1, p2, c, thickness)
  # 1}}}

  def drawText(self, t, pt, size=None, color=None): # {{{1
    "Draw text at the given point"
    kwargs = {}
    kwargs["textColorRGB"] = C3_BLK if color is None else color
    if size is not None:
      kwargs["textSize"] = size
    p.addUserDebugText(t, pt, **kwargs)
  # 1}}}
# 0}}}

# vim: set ts=2 sts=2 sw=2 et:

