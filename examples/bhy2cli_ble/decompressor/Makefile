ifeq ($(OS),Windows_NT)
HOME=C:
endif

COINES_INSTALL_PATH ?= ../../../submodules/coines

EXAMPLE_FILE ?= decompressor.c

INCLUDEPATHS += ../

# Override other target definitions
TARGET = PC

include $(COINES_INSTALL_PATH)/coines.mk
