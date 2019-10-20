#!/usr/bin/env python

"""
Keypress Management

The KeyPressManager class tracks keypresses and invokes the bound functions as
per the requested state information.

Example:

kpm = KeyPressManager()
kpm.bind(key, function, *args, **kwargs)
kpm.bindAll(keys, function, *args, **kwargs)

while True:
  with kpm:
    # Bound functions called automatically
    # Process extra keys if desired:
    if isPressed(key):
      keyIsPressed()

Key compositions (using Control, Alt, and Shift) are possible using the
notation documented in _parseKey, where x is any key:
  x       x
  X       Shift+x (where X is the uppercase x key)
  S-x     Shift+x
  C-x     Control+x
  S-C-x   Shift+Control+x

Note that symbol compositions are *not* supported. Functions bound to "!", for
example, will never be called, as the class has no way of knowing that Shift+1
composes to "!". To get around this, bind to "S-1" instead of "!".

See TODO below.

Key states are a bit mask of the following constants:
  pybullet.KEY_IS_DOWN        = 1
  pybullet.KEY_WAS_TRIGGERED  = 2
  pybullet.KEY_WAS_RELEASED   = 4
The states are used via the following logic, where 1 means "pressed" and 0
means "not pressed" (TODO: VERIFY):
  0 -> 1:         KEY_IS_DOWN | KEY_WAS_TRIGGERED
  1 -> 0:         KEY_WAS_RELEASED
  1 -> 1:         KEY_IS_DOWN
  0 -> 1 -> 0:    (in one frame) KEY_WAS_TRIGGERED | KEY_WAS_RELEASED

The following special keys are understood:
  F1, F2, ...   Function keys F1 through F12
  Left          Left arrow key
  Up            Up arrow key
  Right         Right arrow key
  Down          Down arrow key
  k0, k1, ...   Keypad keys 0 through 9
"""

# TODO: Advanced compositions
# "!" -> "S-1"
# "@" -> "S-2"
# etc...

import functools
import os
import string
import sys
import time

import pyb3
from pyb3 import pybullet as p

try:
  from monotonic import monotonic
except ImportError:
  from time import monotonic

logger = pyb3.getLogger()

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

# Letters
KEY_CLASS_LETTERS = tuple(string.ascii_lowercase)

# Mapping from key codes to key classes. Values can be pairs (key code,
# modifiers) or numbers (key code). Names are loosely derived from vim's key
# names.
KEY_MAP = {
  "Left": p.B3G_LEFT_ARROW,
  "Up": p.B3G_UP_ARROW,
  "Right": p.B3G_RIGHT_ARROW,
  "Down": p.B3G_DOWN_ARROW,
  "F1": p.B3G_F1,
  "F2": p.B3G_F2,
  "F3": p.B3G_F3,
  "F4": p.B3G_F4,
  "F5": p.B3G_F5,
  "F6": p.B3G_F6,
  "F7": p.B3G_F7,
  "F8": p.B3G_F8,
  "F9": p.B3G_F9,
  "F10": p.B3G_F10,
  "F11": p.B3G_F11,
  "F12": p.B3G_F12,
  "k0": p.B3G_KP_0,
  "k1": p.B3G_KP_1,
  "k2": p.B3G_KP_2,
  "k3": p.B3G_KP_3,
  "k4": p.B3G_KP_4,
  "k5": p.B3G_KP_5,
  "k6": p.B3G_KP_6,
  "k7": p.B3G_KP_7,
  "k8": p.B3G_KP_8,
  "k9": p.B3G_KP_9
}

KEY_MAP.update(dict((v,ord(v)) for v in KEY_CLASS_NUMBERS))
KEY_MAP.update(dict((v,ord(v)) for v in KEY_CLASS_LETTERS))

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
  c = ord(key) if isinstance(key, basestring) else key
  if 0 <= c < 256:
    return repr(chr(c))
  for label in dir(p):
    if label.startswith("B3G_") and getattr(p, label) == key:
      return label
  return "unknown key {}".format(int(key))
# 0}}}

def isPressed(keys, key, states=KEY_ANY_STATE, modifiers=()): # {{{0
  "Return True if an event exists for the given key"
  code, mods = _parseKey(key)
  pressed = (keys.get(code, 0) & states != 0)
  # Ensure all modifiers are present, regardless of value
  for m in set(modifiers).union(set(mods)):
    if m not in keys:
      return False
  return pressed
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

def _parseKey(key): # {{{0
  """Covert a key to a code (with modifiers) understood by Bullet. Returns a
  pair (key_code, (modifiers...)).

  If key is a string, the following modifiers are understood:
    "S-"    Require Shift to be pressed
    "M-"    Require Alt to be pressed
    "C-"    Require Control (or Ctrl) to be pressed
    "^"     Require Control (or Ctrl) to be pressed
  For example:
    "S-a"   (p.B3G_A, (p.B3G_SHIFT,))
    "^S-a"  (p.B3G_A, (p.B3G_SHIFT, p.B3G_CONTROL))
    "C-S-a" (p.B3G_A, (p.B3G_SHIFT, p.B3G_CONTROL))
  """
  # Direct lookup
  if key in KEY_MAP:
    val = KEY_MAP[key]
    if type(val) == tuple and len(val) == 2:
      return val[0], val[1]
    else:
      return val, ()
  # Parsing (handling strings)
  mods = []
  if isinstance(key, basestring):
    if len(key) == 0:
      raise ValueError("Unable to parse the empty string as a key code")
    elif len(key) == 1:
      key = ord(key)
    else:
      if "^" in key:
        mods.append(p.B3G_CONTROL)
        key = key.replace("^", "")
      if "C-" in key:
        mods.append(p.B3G_CONTROL)
        key = key.replace("C-", "")
      if "S-" in key:
        mods.append(p.B3G_SHIFT)
        key = key.replace("S-", "")
      if "M-" in key:
        mods.append(p.B3G_ALT)
        key = key.replace("M-", "")
      if key in KEY_MAP:
        key = KEY_MAP[key]
      elif len(key) == 1:
        key = ord(key)
      else:
        raise VaueError("Failed to parse key: {!r} unknown".format(key))
  if key >= ord('A') and key <= ord('Z'):
    return ord(chr(key).lower()), tuple(mods + [p.B3G_SHIFT])
  return key, tuple(mods)
# 0}}}

class KeyPressManager(object): # {{{0
  """
  Manages handling key presses for pybullet applications.

  This class obtains key information on __enter__. The is<state> functions must
  be called between __enter__ and __exit__. Example usage:

  kpm = KeyPressManager()
  while mainLoop:
    with kpm:
      # Registered key binds are handled automatically
      # Process custom key binds if desired:
      if kpm.isPressed(key):
        # Do stuff
  """
  def __init__(self, app, **kwargs): # {{{1
    self._app = app
    self._keys = None
    self._binds = []
    self._debug = kwargs.get("debug", False)
  # 1}}}

  def bind(self, key, func, # {{{1
           timeout=0,
           keys=None,
           on=p.KEY_WAS_TRIGGERED,
           repeat=False,
           modifiers=(),
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
    modifiers
      Modifier keys (B3G_SHIFT, B3G_CONTROL, etc) required
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
    keycode, keymods = _parseKey(key)
    kb = {
      "key": keycode,
      "func": func,
      "states": on | (p.KEY_IS_DOWN if repeat else 0),
      "timeout": timeout,
      "lastPress": 0,
      "wantKeyInfo": keys,
      "modifiers": keymods + modifiers,
      "extraArgs": fArgs,
      "extraKwargs": fKwargs
    }
    if self._debug:
      s = "Bound {} ".format(keycode)
      if kb["modifiers"]:
        s += "+".join(getKeyName(m) for m in kb["modifiers"]) + "+"
      s += getKeyName(kb["key"])
      s += " on {}".format(getKeyStateString(kb["states"]))
      if kb["timeout"]:
        s += " timeout {}".format(kb["timeout"])
      try:
        fs = str(kb["func"].__name__)
      except AttributeError:
        fs = str(kb["func"])
      s += " to {}".format(fs)
      ainfo = []
      if kb["wantKeyInfo"]:
        ainfo.append("keys")
      if kb["extraArgs"]:
        ainfo.extend(repr(arg) for arg in kb["extraArgs"])
      if kb["extraKwargs"]:
        ainfo.extend("{!r}={!r}".format(k, v) for k, v in kb["extraKwargs"].items())
      s += "({})".format(", ".join(ainfo))
      logger.debug(s)
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
      if isPressed(self._keys, kb["key"], kb["states"], kb["modifiers"]):
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
      logger.debug("Calling {}(*{}, **{})".format(func, funcArgs, funcKwargs))
    return func(*funcArgs, **funcKwargs)
  # 1}}}

  @_verifyKeysAttr
  def debugDumpKeys(self): # {{{1
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
      logger.debug(s)
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

