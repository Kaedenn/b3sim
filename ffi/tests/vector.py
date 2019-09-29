#!/usr/bin/env python

"""
Vector tests
"""

import os
import sys
import cffi
import numpy as np

# Append the path to the ffi module to sys.path
MODULE_PATH = os.path.dirname(sys.argv[0])
FFI_PATH = os.path.realpath(os.path.join(MODULE_PATH, ".."))
HOST_PATH = os.path.realpath(os.path.join(MODULE_PATH, "../.."))
sys.path.append(FFI_PATH)
sys.path.append(HOST_PATH)

from native_vector import ffi, lib
from ffi.vector import Vec

if __name__ == "__main__": # {{{0
  import unittest
  def test_size():
    zero = Vec()
    def _test_type(tname):
      print("sizeof({}) = {}".format(tname, ffi.sizeof(tname)))
    _test_type("vec3_t")
    _test_type("vec4_t")
    _test_type("m128_t")
    _test_type("m32_t")

  def test_vec_basic():
    zero = Vec()
    zero2 = Vec(0, 0, 0, 0)
    ones = Vec(x=1, y=1, z=1)

    assert zero == zero2
    assert zero != ones
    assert len(zero) <= len(zero2)
    assert len(zero) == len(ones)
    print(str(zero))
    print(repr(ones))
    print(list(zero))
    print(list(ones))
    ones.w = 1;
    print(list(ones))

  print("m128: {} bits".format(ffi.sizeof("m128_t") * 8))
  print("m32: {} bits".format(ffi.sizeof("m32_t") * 8))
  test_size()
  test_vec_basic()
# 0}}}
