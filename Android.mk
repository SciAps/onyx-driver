LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := lib1000.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_PATH := $(TARGET_OUT)/lib/modules
LOCAL_SRC_FILES := lib1000.ko
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
include $(BUILD_PREBUILT)

KERNEL_DIR := vendor/dm3730-kernel
EXPORTS_DEFS := ARCH=arm CROSS_COMPILE=arm-eabi- KERNEL_DIR=$(realpath $(KERNEL_DIR))

SOURCE_FILES := \
	$(LOCAL_PATH)/lib1000.c \
	$(LOCAL_PATH)/lib1000.h

define onyx-driver-rule
$(LOCAL_PATH)/lib1000.ko: $(KERNEL_DIR)/arch/arm/boot/uImage $(SOURCE_FILES)
	export $(EXPORTS_DEFS) && \
	cd $(LOCAL_PATH) && \
	make
endef

$(eval $(call onyx-driver-rule))




