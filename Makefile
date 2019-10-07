
SRCFILES = $(wildcard *.py) $(wildcard ffi/*.py)

FFI_MODULES = xkeys $(wildcard ffi/*.module)

.PHONY: all ffi clean distclean

all: ffi

ffi:
	python ffi/build.py $(FFI_MODULES)

clean:
	-rm *.pyc
	-rm ffi/*.pyc
	python ffi/build.py -a clean $(FFI_MODULES)

distclean:
	-rm *.pyc
	python ffi/build.py -a distclean $(FFI_MODULES)

