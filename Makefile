
SRCFILES = $(wildcard *.py)

FFI_MODULES = $(wildcard ffi/*.module) xkeys

.PHONY: all clean distclean setup ffi

all: ffi

clean:
	-rm *.pyc
	python ffi/build.py -a clean $(FFI_MODULES)

distclean:
	-rm *.pyc
	python ffi/build.py -a distclean $(FFI_MODULES)

setup: setup.sh
	$(SHELL) setup.sh

ffi:
	python ffi/build.py $(FFI_MODULES)
