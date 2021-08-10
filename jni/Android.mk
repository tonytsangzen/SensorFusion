LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)
LOCAL_C_INCLUDES:= $(LOCAL_PATH)/../
LOCAL_C_INCLUDES += $(LIBUSB_ROOT_ABS)
LOCAL_CFLAGS:=-O2 -Wall -Wextra 
LOCAL_SRC_FILES:= ../libfusion.cpp ../sensor_fusion.cpp ../fusion.cpp
LOCAL_PROPRIETARY_MODULE := true
LOCAL_MODULE:= fusion

include $(BUILD_SHARED_LIBRARY)
