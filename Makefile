
BASEDIR ?= .
PYFILES := $(shell find $(BASEDIR) -name '*.py')

FFI_MODULES = xkeys $(wildcard $(BASEDIR)/ffi/*.module)
FFI_BUILD = $(BASEDIR)/ffi/build.py

.PHONY: all ffi clean distclean

all: ffi

ffi:
	python $(FFI_BUILD) $(FFI_MODULES)

clean:
	-rm $(patsubst %.py,%.pyc,$(PYFILES)) 2>/dev/null
	-rm __pycache__/*.pyc
	-rmdir __pycache__
	python $(FFI_BUILD) -a clean $(FFI_MODULES)

distclean:
	-rm $(patsubst %.py,%.pyc,$(PYFILES)) 2>/dev/null
	-rm __pycache__/*.pyc
	-rmdir __pycache__
	python $(FFI_BUILD) -a distclean $(FFI_MODULES)

