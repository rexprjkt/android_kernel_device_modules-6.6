# SPDX-License-Identifier: GPL-2.0
#
# Copyright (C) 2024 XiaoMi, Inc.
#

obj-$(CONFIG_MITEE_SUPPORT) += mitee.o

mitee-y := core.o
mitee-y += call.o
mitee-y += rpc.o
mitee-y += supp.o
mitee-y += ffa_abi.o
mitee-y += device.o
mitee-y += bench.o
mitee-y += mitee_memlog.o
mitee-y += mitee_task.o
mitee-y += mitee_smc_notify.o
mitee-y += dynamic_mem.o
mitee-y += tee/tee_core.o
mitee-y += tee/tee_shm.o
mitee-y += tee/tee_shm_pool.o

ccflags-y += -I$(DEVICE_MODULES_PATH)/drivers/tee/mitee/tee
ccflags-y += -I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/trusted_mem/ffa_v11

ifeq ($(TARGET_BUILD_VARIANT),userdebug)
ccflags-y += -DENABLE_CMDLINE_SUPPORT
endif
ifeq ($(TARGET_BUILD_VARIANT),eng)
ccflags-y += -DENABLE_CMDLINE_SUPPORT
endif
