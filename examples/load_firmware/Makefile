COINES_INSTALL_PATH ?= ../../../..

EXAMPLE_FILE ?= load_firmware.c

BHY2_INTF ?= 

COINES_BACKEND ?= COINES_BRIDGE

C_SRCS += ../common/common.c
C_SRCS += ../../bhy2.c
C_SRCS += ../../bhy2_hif.c
C_SRCS += ../../bhy2_parse.c

INCLUDEPATHS += .
INCLUDEPATHS += ../common
INCLUDEPATHS += ../..
INCLUDEPATHS += ../../firmware

# By default, the sensor is connected over SPI. Define this to change to I2C
ifeq ($(BHY2_INTF), I2C)
CFLAGS += -DBHY2_USE_I2C
endif

include $(COINES_INSTALL_PATH)/coines.mk