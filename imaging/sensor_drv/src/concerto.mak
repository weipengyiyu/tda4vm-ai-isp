ifeq ($(TARGET_CPU), $(filter $(TARGET_CPU), R5F))

include $(PRELUDE)
TARGET      := ti_imaging_sensordrv
TARGETTYPE  := library
CSOURCES    := $(call all-c-files)

CSOURCES    += ovx3c/iss_sensor_ovx3c.c
# CSOURCES    += sc2210/iss_sensor_sc2210.c
CSOURCES    += sc2210/iss_sensor_sc2210_wdr.c
CSOURCES    += imx623/iss_sensor_imx623.c
CSOURCES    += imx728/iss_sensor_imx728.c

IDIRS       += $(HOST_ROOT)/sensor_drv/include
IDIRS       += $(APP_UTILS_PATH)/utils/remote_service/include
IDIRS       += $(APP_UTILS_PATH)/utils/ipc/include
IDIRS       += $(PDK_PATH)/packages

ifeq ($(SOC), am62a)
SKIPBUILD=1
endif

include $(FINALE)

endif
