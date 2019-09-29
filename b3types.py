#!/usr/bin/env python

"""
"""

import numpy as np
import os
import sys

b3Scalar = float

b3vMzeroMask = (-0.0, -0.0, -0.0, -0.0)
b3v1110 = (1.0, 1.0, 1.0, 0.0)
b3vHalf = (0.5, 0.5, 0.5, 0.5)

def b3MakeVector3(x, y, z, w=0):
  return np.array((x, y, z, w), dtype=np.float32)

def b3MakeVector4(x, y, z, w):
  return np.array((x, y, z, w), dtype=np.float32)

class Vector3(object): # {{{0
  "Vector3 type"

  def __init__(self, *args):
    super(Vector3, self).__setattr__("_value", np.zeros(3))
    if len(args) == 3:
      self._value[:] = args
    elif len(args) == 1:
      self._value[:] = self._interpret(args[0])

  def _interpret(self, val):
    "Attempt to interpret val as a Vector3"
    if isinstance(val, np.ndarray):
      if len(val) == 3:
        return val
      raise ValueError("Failed to interpret {!r}: invalid size".format(val))
    try:
      v = tuple(val)
      if len(v) == 3:
        return v
      raise ValueError("Failed to interpret {!r}: invalid size".format(val))
    except TypeError as e:
      raise ValueError("Failed to interpret {!r}: {}".format(val, e))

  def _is_attr(self, attr):
    "Return whether or not attr is a valid vec3 attribute"
    return re.match("^[xyz]+$", attr) or re.match("^[rgb]+$", attr)

  def _attr_to_index(self, c):
    "Convert an attriute character to an index in self._value"
    if c in "xyz":
      return "xyz".index(c)
    if c in "rgb":
      return "rgb".index(c)
    raise AttributeError(c)

  def get(self):
    "Obtain the underlying value"
    return self._value

  def __getitem__(self, idx):
    "Obtain a single value"
    if idx in (0, 1, 2):
      return self._value[idx]
    return self._value[self._attr_to_index(idx)]

  def __getattr__(self, attr):
    "Get either a single or multi value"
    if attr == "_value":
      return object.__getattr__(self, attr)
    if self._is_attr(attr):
      if len(attr) == 1:
        return self._value[self._attr_to_index(attr)]
      else:
        return tuple(self._value[self._attr_to_index(i)] for i in attr)
    raise AttributeError(attr)

  def __setattr__(self, attr, value):
    "Set either a single or multi value"
    if attr == "_value":
      object.__setattr__(self, attr, value)
    if self._is_attr(attr):
      if len(attr) == 1:
        pass
      elif len(attr) == len(value):
        pass
      else:
        raise TypeError("attr, value differ in size: {!r} {!r}".format(attr, value))
    raise AttributeError(attr)

  def __iter__(self):
    return iter(self._value)

  def __str__(self):
    return str(self._value)

  def __repr__(self):
    return "Vector3({!r})".format(self._value)
# 0}}}

# vim: set ts=2 sts=2 sw=2:
