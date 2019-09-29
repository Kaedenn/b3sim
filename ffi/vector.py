#!/usr/bin/env python

"""
Vector type: Python wrapper module
"""

import os
import sys
import cffi
import numpy as np

LIB_NAME = "native_vector.so"
if os.path.exists(LIB_NAME):
  # Module is where we expect it to be
  pass
elif os.path.exists(os.path.join(os.path.realpath(".."), LIB_NAME)):
  # Module is in a parent directory
  sys.path.append(os.path.join(os.path.realpath(".."), LIB_NAME))
else:
  sys.stderr.write("Unable to find {}\n".format(LIB_NAME))

from native_vector import ffi as vec_ffi, lib as vec_lib

class Vec(object): # {{{0
  """Abstraction layer wrapping native_vector
  
  Supports the following functionality:
    Vec()
    Vec(m128)
    Vec(x, y, z)
    Vec(x, y, z, w)
    Vec(m128=val)
    Vec(x=val, y=val, z=val)
    Vec(x=val, y=val, z=val, w=val)
  """
  def __init__(self, *args, **kwargs):
    self._val = vec_ffi.new("vec4_t*")
    self._has_w = False
    if len(args) == 1:
      self.m128 = args[0]
    if len(args) == 3:
      self.x, self.y, self.z = args
    elif len(args) == 4:
      self.x, self.y, self.z, self.w = args
    elif "m128" in kwargs:
      self.m128 = kwargs["m128"]
    else:
      if "x" in kwargs:
        self.x = kwargs["x"]
      if "y" in kwargs:
        self.y = kwargs["y"]
      if "z" in kwargs:
        self.z = kwargs["z"]
      if "w" in kwargs:
        self.w = kwargs["w"]

  def __getattr__(self, k):
    "Obtain some other attribute of the underlying value"
    return getattr(self.value, k)

  def pointer(self):
    "Return the underlying value pointer"
    return self._val

  def buffer(self):
    "Return a buffer representing the underlying value's data"
    return tuple(vec_ffi.buffer(self._val))

  @property
  def value(self):
    return self._val[0]
  @value.setter
  def value(self, v):
    self._val[0] = v

  @property
  def m128(self):
    return self.value.m128
  @m128.setter
  def m128(self, v):
    self.value.m128 = v

  @property
  def x(self):
    return self.value.x
  @x.setter
  def x(self, v):
    self.value.x = v

  @property
  def y(self):
    return self.value.y
  @y.setter
  def y(self, v):
    self.value.y = v

  @property
  def z(self):
    return self.value.z
  @z.setter
  def z(self, v):
    self.value.z = v

  @property
  def w(self):
    return self.value.w
  @w.setter
  def w(self, v):
    self._has_w = True
    self.value.w = v

  def __eq__(self, other):
    return self.buffer() == other.buffer()

  def __iter__(self):
    yield self.x
    yield self.y
    yield self.z
    if self._has_w:
      yield self.w

  def __len__(self):
    if self._has_w:
      return 4
    return 3

  def __str__(self):
    return "Vec(m128={}, x={}, y={}, z={}, w={})".format(
      self.m128, self.x, self.y, self.z, self.w)

  def __repr__(self):
    typename = vec_ffi.typeof(self._val).cname
    buf = tuple("{:02x}".format(ord(c)) for c in self.buffer())
    nums = ["".join(buf[i:i+4]) for i in range(0, len(buf), 4)]
    return "({}){{{}}}".format(typename, ", ".join("0x" + n for n in nums))
# 0}}}

