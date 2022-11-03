#CCE_Board_Definitions:BHI260, BHA260
COINES_INSTALL_PATH ?= ../../../../..

EXAMPLE_FILE ?= swim_data_injection.c

BHY2_INTF ?= 

COINES_BACKEND ?= COINES_BRIDGE

C_SRCS += ../../common/common.c
C_SRCS += ../../../bhy2.c
C_SRCS += ../../../bhy2_hif.c
C_SRCS += ../../../bhy2_parse.c
C_SRCS += ../../../bhy2_swim.c

INCLUDEPATHS += ../../common
INCLUDEPATHS += ../../..

#SHUTTLE_BOARD ?= BHI260

#TARGET = MCU_APP30

#LOCATION = FLASH

# By default, the sensor is connected over SPI. Define this to change to I2C
ifeq ($(BHY2_INTF), I2C)
CFLAGS += -DBHY2_USE_I2C
endif

include $(COINES_INSTALL_PATH)/coines.mk