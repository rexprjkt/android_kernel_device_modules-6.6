/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _XIAOMI_TOUCH_SCP_H
#define _XIAOMI_TOUCH_SCP_H

#include <linux/device.h>
#include <linux/pinctrl/consumer.h>

struct xiaomi_touch_scp_param {
	u32 state;
	u32 vendor;
	u32 gesture_address;
	u32 acknowledge_address;
	u32 gesture_size;
	u32 reserved;
	u32 fod_address;
	u32 fod_size;
	u32 frame_address;
	u32 frame_size;
	u32 event;
	u32 event_size;
	u8 data[64];
};

struct xiaomi_touch_scp_ops {
	void (*gesture)(void *context);
	void (*irq_owner)(void *context, bool scp);
};

int xiaomi_touch_scp_register(struct device *dev, struct pinctrl *pinctrl,
	const struct xiaomi_touch_scp_param *param,
	const struct xiaomi_touch_scp_ops *ops, void *context);
void xiaomi_touch_scp_unregister(void);
int xiaomi_touch_scp_switch(bool scp, u32 gestures);
int xiaomi_touch_scp_update_gesture(u32 gestures, u32 nonui);
int xiaomi_touch_scp_log_level(u32 level);
bool xiaomi_touch_scp_owned(void);
int xiaomi_touch_scp_read(u32 address, void *data, size_t size);

#endif
