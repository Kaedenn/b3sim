#!/usr/bin/env python

import functools
import os
import string
import sys
import time

from pyb3 import pybullet as p

try:
  from monotonic import monotonic
except ImportError:
  from time import monotonic

# Key states
KEY_ANY_STATE = p.KEY_IS_DOWN | p.KEY_WAS_TRIGGERED | p.KEY_WAS_RELEASED
KEY_STATES = {
  "KEY_DOWN": p.KEY_IS_DOWN,
  "KEY_TRIGGERED": p.KEY_WAS_TRIGGERED,
  "KEY_RELEASED": p.KEY_WAS_RELEASED
}

# Modifier keys
KEY_CLASS_MODIFIERS = (p.B3G_CONTROL, p.B3G_SHIFT, p.B3G_ALT)

# Arrow keys
KEY_CLASS_ARROWS = (
  p.B3G_DOWN_ARROW, p.B3G_LEFT_ARROW, p.B3G_RIGHT_ARROW, p.B3G_UP_ARROW)

# Numpad number keys
KEY_CLASS_NUMPAD = (
  p.B3G_KP_0, p.B3G_KP_1, p.B3G_KP_2, p.B3G_KP_3, p.B3G_KP_4,
  p.B3G_KP_5, p.B3G_KP_6, p.B3G_KP_7, p.B3G_KP_8, p.B3G_KP_9)

# Numpad arrow keys
KEY_CLASS_NUMPAD_ARROWS = (
  p.B3G_KP_DOWN, p.B3G_KP_LEFT, p.B3G_KP_RIGHT, p.B3G_KP_UP)

# Number keys
KEY_CLASS_NUMBERS = tuple("0123456789")

# Letters, lower-case
KEY_CLASS_LETTERS_LC = tuple(string.ascii_lowercase)

# Letters, upper-case
KEY_CLASS_LETTERS_UC = tuple(string.ascii_uppercase)

# Letters, lower-case and upper-case
KEY_CLASS_LETTERS = tuple(string.ascii_letters)

# Function keys
KEY_CLASS_FN = (
  p.B3G_F1, p.B3G_F2, p.B3G_F3, p.B3G_F4, p.B3G_F5, p.B3G_F6,
  p.B3G_F7, p.B3G_F8, p.B3G_F9, p.B3G_F10, p.B3G_F11, p.B3G_F12)

def getKeyStateString(state): # {{{0
  "Obtain a string describing the state code"
  s = []
  for sn, sv in KEY_STATES.items():
    if state & sv == sv:
      s.append(sn)
  return "+".join(s)
# 0}}}

def getKeyName(key): # {{{0
  "Obtain the name of the key code"
  if 0 <= key < 256:
    return chr(key)
  for label in dir(p):
    if label.startswith("B3G_") and getattr(p, label) == key:
      return label
  return "unknown key {:x}".format(int(key))
# 0}}}

def isPressed(keys, key, states=KEY_ANY_STATE): # {{{0
  "Return True if an event exists for the given key"
  if isinstance(key, basestring) and len(key) == 1:
    code = ord(key)
  else:
    code = key
  return keys.get(code, 0) & states != 0
# 0}}}

def anyPressed(keys, *keysToCheck): # {{{0
  "Return True if any of the keys have events"
  for k in keysToCheck:
    if isPressed(keys, k):
      return True
  return False
# 0}}}

def _verifyKeysAttr(func): # {{{0
  "Decorator: verify self._keys is not None"
  @functools.wraps(func)
  def wrappedFunc(self, *args, **kwargs):
    if self._keys is not None:
      return func(self, *args, **kwargs)
  return wrappedFunc
# 0}}}

def _stateGetter(*states): # {{{0
  "Decorator: key state getter: return whether or not key is in states"
  def decoratorFunc(func):
    @functools.wraps(func)
    def getterFunc(self, key):
      if self._keys is not None:
        return self._keys.get(key, None) in states
    return getterFunc
  return decoratorFunc
# 0}}}

class KeyPressManager(object): # {{{0
  """
  Manages handling key presses for pybullet applications.

  This class obtains key information on __enter__. The is<state> functions must
  be called between __enter__ and __exit__. Example usage:

  kpm = KeyPressManager()
  while mainLoop:
    with kpm:
      # Request key information from kpm
      if kpm.isPressed(key):
        # Do stuff
  """
  def __init__(self, app, **kwargs): # {{{1
    self._app = app
    self._keys = None
    self._binds = []
    self._debug = kwargs.get("debug", False)
  # 1}}}

  def _parseKey(self, key): # {{{1
    "Covert a key to a value understood by Bullet"
    if key[0] == 'F' and key[1:].isdigit() and 1 <= int(key[1:]) <= 12:
      return p.B3G_F1 + int(key[1:]) - 1
    return key
  # 1}}}

  def bind(self, key, func, # {{{1
           timeout=0,
           keys=None,
           on=p.KEY_WAS_TRIGGERED,
           repeat=False,
           args=None, kwargs=None):
    """Bind a key to a function
    key
      The key code to listen for
    func
      The function to call
    timeout
      Time in seconds to ignore the key
    keys
      If True, pass the key-press info as the first argument to func
    on
      Bit mask of the desired key states; only matching states will call func
    repeat
      Handle a key being held down continuously
    args
      Extra positional args to pass to func
    kwargs
      Extra keyword arguments to pass to func
    """
    fArgs = []
    fKwargs = {}
    if args is not None:
      fArgs.extend(args)
    if kwargs is not None:
      fKwargs.update(kwargs)
    kb = {
      "key": key,
      "func": func,
      "states": on | (p.KEY_IS_DOWN if repeat else 0),
      "timeout": timeout,
      "lastPress": 0,
      "wantKeyInfo": keys,
      "extraArgs": fArgs,
      "extraKwargs": fKwargs
    }
    self._binds.append(kb)
  # 1}}}

  def bindAll(self, keys, func, *args, **kwargs): # {{{1
    "Bind a list/tuple of keys to a function (see self.bind)"
    for k in keys:
      self.bind(k, func, *args, **kwargs)
  # 1}}}

  def __enter__(self): # {{{1
    "Start current round of key press handling"
    self._keys = p.getKeyboardEvents()
    # Handle keybinds
    for kb in self._binds:
      if isPressed(self._keys, kb["key"], kb["states"]):
        if monotonic() - kb["lastPress"] >= kb["timeout"]:
          self._callBoundFunc(kb)
        kb["lastPress"] = monotonic()
  # 1}}}

  def __exit__(self, exc_type, exc_val, exc_tb): # {{{1
    "End current round of key press handling"
    self._keys = None
  # 1}}}

  @_verifyKeysAttr
  def _callBoundFunc(self, kb): # {{{1
    "Private: call the function defined in the keybind object"
    wantKeys = kb["wantKeyInfo"] if kb["wantKeyInfo"] in (True, False) else False
    func = kb["func"]
    funcArgs = [self._keys] if wantKeys else []
    funcKwargs = {}
    funcArgs.extend(kb["extraArgs"])
    funcKwargs.update(kb["extraKwargs"])
    if self._debug:
      print("Calling {}(*{}, **{})".format(func, funcArgs, funcKwargs))
    return func(*funcArgs, **funcKwargs)
  # 1}}}

  @_verifyKeysAttr
  def debugDumpKeys(self, target=sys.stderr): # {{{1
    "Print current keypress information"
    for key, state in self._keys.items():
      if key in (p.B3G_SHIFT, p.B3G_CONTROL, p.B3G_ALT):
        continue
      kn = getKeyName(key)
      ks = getKeyStateString(state)
      s = "Key: {} {:x} ({}) {} ({})".format(key, key, kn, ks, state)
      if p.B3G_SHIFT in self._keys:
        s += "; Shift"
      if p.B3G_CONTROL in self._keys:
        s += "; Control"
      if p.B3G_ALT in self._keys:
        s += "; Alt"
      target.write(s)
      target.write("\n")
  # 1}}}

  @_verifyKeysAttr
  def getKeys(self): # {{{1
    "Return the keys present in the current key state"
    return self._keys.keys()
  # 1}}}

  @_verifyKeysAttr
  def getKeyStates(self): # {{{1
    "Return the current key state information"
    return self._keys
  # 1}}}

  @_stateGetter(p.KEY_IS_DOWN | p.KEY_WAS_TRIGGERED | p.KEY_WAS_RELEASED)
  def isAny(self, key): # {{{1
    "Return True if key has any event present"
    pass
  # 1}}}

  @_stateGetter(p.KEY_IS_DOWN)
  def isDown(self, key): # {{{1
    "Return True if key presently has the DOWN state"
    pass
  # 1}}}

  @_stateGetter(p.KEY_WAS_TRIGGERED)
  def isTriggered(self, key): # {{{1
    "Return True if key presently has the TRIGGERED state"
    pass
  # 1}}}

  @_stateGetter(p.KEY_WAS_RELEASED)
  def isReleased(self, key): # {{{1
    "Return True if key presently has the RELEASED state"
    pass
  # 1}}}

  @_stateGetter(p.KEY_IS_DOWN | p.KEY_WAS_TRIGGERED)
  def isPressed(self, key): # {{{1
    "Return True if the key was pressed and not released"
    pass
  # 1}}}

  @_stateGetter(p.KEY_WAS_TRIGGERED | p.KEY_WAS_RELEASED)
  def isTapped(self, key): # {{{1
    "Return True if key was pressed and released in the same physics step"
    pass
  # 1}}}
# 0}}}

# vim: set ts=2 sts=2 sw=2 et:

