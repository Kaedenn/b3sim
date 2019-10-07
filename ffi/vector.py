#!/usr/bin/env python

"""
Vector type: Python wrapper module

This module wraps the native_vector.so ffi module, which provides only the
underlying vec3_t and vec4_t types.

The vector is a union of one 128-bit floating-point value and four 32-bit
floating-point values, named "m128", "x", "y", "z", and "w", respectively.
This is to allow vectors to be interpreted as scalars, which is convenient for
working with the Bullet3 library.

This class abstracts vectors in 3-space and 4-space using the same interface.
Special care is made to ensure vectors in 3-space are treated appropriately
and differently from vectors in 4-space. Vectors in 3-space have a `w` value
of None. However, vectors in 3-space can be converted to vectors in 4-space by
setting w.

v = Vec() # Vector in 3-space: len(v) == 3
v.w = 0   # Vector is now in 4-space: len(v) == 4

Note that it is an error to call numpy functions on two vectors of differing
dimensions, due to the vectors having different shapes.

Note that vec.m128's value has no correlation with the four floating-point 
values. Due to the internal representation of floating-point numbers, vec.m128
will be some nonsense value.
"""

import cffi
import ctypes
import numpy as np
import operator
import os
import sys

__all__ = ["LIB_NAME", "vec_ffi", "vec_lib", "Vec", "VEC_EPSILON"]

# Determine where native_vector.so is and import it
LIB_NAME = "native_vector.so"
if os.path.exists(LIB_NAME):
  # Module is where we expect it to be
  pass
elif os.path.exists(os.path.join(os.path.realpath(".."), LIB_NAME)):
  # Module is in a parent directory
  sys.path.append(os.path.join(os.path.realpath(".."), LIB_NAME))

from native_vector import ffi as vec_ffi, lib as vec_lib

# Epsilon: the smallest value that can be represented
VEC_EPSILON = vec_lib.VEC_EPSILON

class Vec(object): # {{{0
  """Abstraction layer wrapping the ffi native_vector.so module"""

  # Native data type
  dtype = np.float32

  def __init__(self, *args, **kwargs): # {{{1
    """
    3-vectors (vectors of three spatial coordinates):
      Vec()
      Vec(m128)
      Vec(x, y, z)
      Vec(m128=val)
      Vec(x=val, y=val, z=val)
      Vec(np.array([x, y, z]))
    4-vectors (vectors of four coordinates; quaternions):
      Vec(x, y, z, w)
      Vec(x=val, y=val, z=val, w=val)
      Vec(np.array([x, y, z, w]))
    """
    self._val = vec_ffi.new("vec4_t*")
    self._has_w = False
    self._readonly = False        # So that self._update works
    self._update(*args, **kwargs)
    self._readonly = kwargs.get("readonly", False)
  # 1}}}

  def _update(self, *args, **kwargs): # {{{1
    "Assign a value to the vector"
    if len(args) == 0 and len(kwargs) == 0:
      self.x = 0
      self.y = 0
      self.z = 0
    elif len(args) == 1:
      if isinstance(args[0], np.ndarray):
        self._update(*args[0])
      elif isinstance(args[0], Vec):
        self._update(*tuple(args[0]))
      elif hasattr(args[0], "__len__") and len(args[0]) in (3, 4):
        self._update(*args[0])
      else:
        self.m128 = args[0]
    elif len(args) in (3, 4):
      self.x = args[0]
      self.y = args[1]
      self.z = args[2]
      if len(args) == 4:
        self.w = args[3]
    elif "m128" in kwargs:
      self.m128 = kwargs["m128"]
    elif all(k in kwargs for k in ("x", "y", "z")):
      self.x = kwargs["x"]
      self.y = kwargs["y"]
      self.z = kwargs["z"]
      if "w" in kwargs:
        self.w = kwargs["w"]
    else:
      raise TypeError("Invalid arguments (*{}, **{})".format(args, kwargs))
  # 1}}}

  @property
  def shape(self): # {{{1
    "Shape of the vector (len, 1)"
    if self._has_w:
      return (4, 1)
    return (3, 1)
  # 1}}}

  def as_array(self, dtype=np.float32): # {{{1
    "Create an np.array from the vector"
    if self._has_w:
      t = (self.x, self.y, self.z, self.w)
    else:
      t = (self.x, self.y, self.z)
    # Store to prevent gc crashes
    self._arr = np.array(t, dtype=dtype)
    return self._arr
  # 1}}}

  __array__ = as_array

  def __getattr__(self, k): # {{{1
    "Obtain some other attribute of the underlying value"
    return getattr(self.value, k)
  # 1}}}

  def pointer(self): # {{{1
    "Return the underlying value pointer"
    return self._val
  # 1}}}

  def buffer(self): # {{{1
    "Return a buffer representing the underlying value's data"
    return tuple(vec_ffi.buffer(self._val))
  # 1}}}

  def norm(self): # {{{1
    "Return the scalar length of this vector"
    return np.linalg.norm(self)
  # 1}}}

  def unit(self): # {{{1
    "Return a unit vector in the same direction"
    return self / self.norm()
  # 1}}}

  def safe_unit(self): # {{{1
    """Return a unit vector in the same direction, or None if a vector is
    directionless (as in, norm() == 0)"""
    n = self.norm()
    if n >= VEC_EPSILON:
      return self / n
  # 1}}}

  def dot(self, other): # {{{1
    "Return the dot product of self and other"
    return np.dot(self, other)
  # 1}}}

  def cross(self, other): # {{{1
    "Return the vector cross product of self and other"
    return Vec(np.cross(self, other))
  # 1}}}

  def distance(self, other): # {{{1
    "Distance between two vectors (treating other like a point)"
    return (self - other).norm()
  # 1}}}

  def transform(self, mat): # {{{1
    "Apply a matrix transformation"
    v = Vec(self)
    v.itransform(mat)
    return v
  # 1}}}

  def itransform(self, mat): # TODO # {{{1
    "Apply a matrix transformation in-place"
    pass
  # 1}}}

  def get_cyl(self): # {{{1
    "Return the cylindrical coordinates for the vector's position"
    r = abs(self)
    if self.x >= VEC_EPSILON:
      theta = np.math.atan(self.y / self.x)
    else:
      theta = np.math.atan(float("infinity"))
    z = self.z
    return r, theta, z
  # 1}}}

  def get_sph(self): # {{{1
    "Return the spherical coordinates for the vector's position"
    r = abs(self)
    if self.x >= VEC_EPSILON:
      theta = np.math.atan(self.y / self.x)
    else:
      theta = np.math.atan(float("infinity"))
    if r >= VEC_EPSILON:
      phi = np.math.acos(self.z / r)
    else:
      phi = np.math.acos(float("infinity"))
    return r, theta, phi
  # 1}}}

  def rotate(self, axis, angle): # {{{1
    "Rotate the vector by a given angle along a given axis"
    v = Vec(self)
    v.irotate(axis, angle)
    return v
  # 1}}}

  def irotate(self, axis, angle): # TODO # {{{1
    "Rotate the vector in-place by a given angle along a given axis"
    assert not self._readonly
  # 1}}}

  def setAngle(self, axis, angle): # TODO # {{{1
    "Set quaternion angle using the axis-angle method"
    pass
  # 1}}}

  def setEuler(self, pitch, yaw, roll): # TODO # {{{1
    "Set quaternion angle using Euler angles"
    pass
  # 1}}}

  def angle(self, other): # {{{1
    "Return the quaternion angle between self and other"
    if self._has_w:
      return 2.0 * np.math.acos(self.w)
    return 0
  # 1}}}

  def getAxis(self): # {{{1
    "Get the quaternion's axis of rotation"
    if self._has_w:
      s_sq = 1.0 - self.w ** 2
      if s_sq < 10.0 * VEC_EPSILON:
        return Vec(1, 0, 0)
      s = 1.0 / np.math.sqrt(s_sq)
      return Vec(self.x * s, self.y * s, self.z * s)
    return Vec(0, 0, 0)
  # 1}}}

  def inverse(self): # TODO # {{{1
    "Calculate the inverse of the vector or quaternion"
    if self._has_w:
      return Vec(-self.x, -self.y, -self.z, self.w)
    else:
      # TODO
      pass
  # 1}}}

  def absolute(self): # {{{1
    "Return a vector with all components positive"
    v = Vec()
    v.x = abs(self.x)
    v.y = abs(self.y)
    v.z = abs(self.z)
    if self._has_w:
      v.w = abs(self.w)
    return v
  # 1}}}

  def triple(self, v1, v2): # TODO # {{{1
    "Return the triple product of self and the two vectors given"
    pass
  # 1}}}

  def minAxis(self): # {{{1
    "Return which axis is the smallest: 0, 1, or 2 (or 3 if scale=4)"
    return min(range(len(self)), key=lambda i: self[i])
  # 1}}}

  def maxAxis(self): # {{{1
    "Return which axis is the largest: 0, 1, or 2 (or 3 if scale=4)"
    return max(range(len(self)), key=lambda i: self[i])
  # 1}}}

  def lerp(self, v, t): # TODO # {{{1
    "Return the linear interpolation between this and another vector"
    pass
  # 1}}}

  def _apply_op(self, op, other, swap=False): # {{{1
    "Apply an operator to self and other"
    if isinstance(other, Vec):
      p1 = self.as_array()
      p2 = other.as_array()
    else:
      p1 = self.as_array()
      p2 = other
    if swap:
      p1, p2 = p2, p1
    return Vec(op(p1, p2))
  # 1}}}

  # Operators {{{1
  def __add__(self, other):
    return self._apply_op(operator.add, other)
  def __sub__(self, other):
    return self._apply_op(operator.sub, other)
  def __mul__(self, other):
    return self._apply_op(operator.mul, other)
  def __div__(self, other):
    return self._apply_op(operator.div, other)
  def __radd__(self, other):
    return self._apply_op(operator.add, other, swap=True)
  def __rsub__(self, other):
    return self._apply_op(operator.sub, other, swap=True)
  def __rmul__(self, other):
    return self._apply_op(operator.mul, other, swap=True)
  def __rdiv__(self, other):
    return self._apply_op(operator.div, other, swap=True)
  # 1}}}

  def __abs__(self): # {{{1
    "Return the length of the vector"
    return self.norm()
  # 1}}}
  
  def __neg__(self): # {{{1
    "Negate the vector"
    return -1 * self
  # 1}}}

  def __eq__(self, other): # {{{1
    "Compare two vectors for equality"
    return tuple(self) == tuple(other)
  # 1}}}

  @property
  def value(self): # {{{1
    return self._val[0]
  @value.setter
  def value(self, v):
    assert not self._readonly
    self._val[0] = v
  # 1}}}

  @property
  def m128(self): # {{{1
    return self.value.m128
  @m128.setter
  def m128(self, v):
    assert not self._readonly
    self.value.m128 = v
  # 1}}}

  @property
  def x(self): # {{{1
    return self.value.x
  @x.setter
  def x(self, v):
    assert not self._readonly
    self.value.x = v
  # 1}}}

  @property
  def y(self): # {{{1
    return self.value.y
  @y.setter
  def y(self, v):
    assert not self._readonly
    self.value.y = v
  # 1}}}

  @property
  def z(self): # {{{1
    return self.value.z
  @z.setter
  def z(self, v):
    assert not self._readonly
    self.value.z = v
  # 1}}}

  @property
  def w(self): # {{{1
    if self._has_w:
      return self.value.w
    return None
  @w.setter
  def w(self, v):
    assert not self._readonly
    self._has_w = True
    self.value.w = v
  # 1}}}

  def __getitem__(self, i): # {{{1
    "Obtain component by index"
    if i == 0:
      return self.x
    if i == 1:
      return self.y
    if i == 2:
      return self.z
    if i == 3 and self._has_w:
      return w
    raise IndexError(i)
  # 1}}}

  def __iter__(self): # {{{1
    "Iterate over the three (or four) vector components"
    yield self.x
    yield self.y
    yield self.z
    if self._has_w:
      yield self.w
  # 1}}}

  def __len__(self): # {{{1
    "Return the number of vector components (3 or 4)"
    if self._has_w:
      return 4
    return 3
  # 1}}}

  def __str__(self): # {{{1
    "Return a sensible string representation of the vector"
    if self._has_w:
      return "Vec({}, {}, {}, {})".format(self.x, self.y, self.z, self.w)
    else:
      return "Vec({}, {}, {})".format(self.x, self.y, self.z)
  # 1}}}

  def __repr__(self): # {{{1
    "Return a string containing the raw vector information"
    typename = vec_ffi.typeof(self._val).cname
    buf = tuple("{:02x}".format(ord(c)) for c in self.buffer())
    nums = ["".join(buf[i:i+4]) for i in range(0, len(buf), 4)]
    return "({}){{{}}}".format(typename, ", ".join("0x" + n for n in nums))
  # 1}}}
# 0}}}

# Specific common vectors
Vec.V000 = Vec(0, 0, 0, readonly=True)
Vec.V100 = Vec(1, 0, 0, readonly=True)
Vec.V010 = Vec(0, 1, 0, readonly=True)
Vec.V001 = Vec(0, 0, 1, readonly=True)
Vec.V111 = Vec(1, 1, 1, readonly=True)
Vec.V0000 = Vec(0, 0, 0, 0, readonly=True)
Vec.V1000 = Vec(1, 0, 0, 0, readonly=True)
Vec.V0100 = Vec(0, 1, 0, 0, readonly=True)
Vec.V0010 = Vec(0, 0, 1, 0, readonly=True)
Vec.V0001 = Vec(0, 0, 0, 1, readonly=True)
Vec.V1111 = Vec(1, 1, 1, 1, readonly=True)
Vec.VHalf = Vec(0.5, 0.5, 0.5, readonly=True)
Vec.VHalf4 = Vec(0.5, 0.5, 0.5, 0.5, readonly=True)
