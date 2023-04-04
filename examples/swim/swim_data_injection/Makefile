#CCE_Board_Definitions:BHI260, BHA260
COINES_INSTALL_PATH ?= ../../../../..

EXAMPLE_FILE ?= swim_data_injection.c

BHY2_INTF ?= 

COINES_BACKEND ?= COINES_BRIDGE

API_LOCATION ?= ../../..

FW_LOCATION ?= ../../..

COMMON_LOCATION ?= ../..

C_SRCS += \
$(COMMON_LOCATION)/common/common.c \
$(API_LOCATION)/bhy2.c \
$(API_LOCATION)/bhy2_hif.c \
$(API_LOCATION)/bhy2_parse.c \
$(API_LOCATION)/bhy2_swim.c 

INCLUDEPATHS += . \
$(COMMON_LOCATION)/common \
$(API_LOCATION) \
$(FW_LOCATION)/firmware

#SHUTTLE_BOARD ?= BHI260

#TARGET = MCU_APP30

#LOCATION = FLASH

# By default, the sensor is connected over SPI. Define this to change to I2C
ifeq ($(BHY2_INTF), I2C)
CFLAGS += -DBHY2_USE_I2C
endif

include $(COINES_INSTALL_PATH)/coines.mk