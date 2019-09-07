
SRCFILES = $(wildcard *.py)

.PHONY: all clean setup

all:

clean:
	-rm *.pyc

setup: setup.sh
	$(SHELL) setup.sh

