// SPDX-License-Identifier: GPL-2.0
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include "../../../misc/mediatek/scp/include/scp.h"
#include "xiaomi_touch_scp.h"

extern struct mtk_ipi_device scp_ipidev;

static struct {
	struct device *dev;
	struct pinctrl *pinctrl;
	struct pinctrl_state *ap_mode;
	struct pinctrl_state *scp_mode;
	struct xiaomi_touch_scp_param param;
	struct xiaomi_touch_scp_ops ops;
	void *context;
	void *shared;
	phys_addr_t physical;
	size_t size;
	bool active;
	bool registered;
	u32 receive[3];
	unsigned int retries;
	struct completion ack;
	struct work_struct gesture_work;
	struct delayed_work param_work;
	struct notifier_block notifier;
} touch_scp;

static DEFINE_MUTEX(touch_scp_lock);
static DEFINE_SPINLOCK(touch_scp_rx_lock);

static int touch_scp_send(u32 command, u32 value, u32 argument, u32 reserved)
{
	u32 message[] = { command, value, argument, reserved };

	return mtk_ipi_send(&scp_ipidev, 54, 0, message, ARRAY_SIZE(message), 80);
}

static void touch_scp_gesture_work(struct work_struct *work)
{
	if (READ_ONCE(touch_scp.active))
		touch_scp.ops.gesture(touch_scp.context);
}

static void touch_scp_param_work(struct work_struct *work)
{
	unsigned long flags;

	mutex_lock(&touch_scp_lock);
	if (!touch_scp.active || !touch_scp.shared ||
	    READ_ONCE(touch_scp.param.state) != 1)
		goto out;
	spin_lock_irqsave(&touch_scp_rx_lock, flags);
	memcpy(touch_scp.shared, &touch_scp.param, sizeof(touch_scp.param));
	spin_unlock_irqrestore(&touch_scp_rx_lock, flags);
	wmb();
	touch_scp_send(1, lower_32_bits(touch_scp.physical), touch_scp.size, 0);
	if (touch_scp.retries++ < 5)
		schedule_delayed_work(&touch_scp.param_work, msecs_to_jiffies(1000));
out:
	mutex_unlock(&touch_scp_lock);
}

static int touch_scp_receive(unsigned int id, void *private, void *data,
			     unsigned int length)
{
	u32 command = *(u32 *)data;
	struct xiaomi_touch_scp_param *shared = touch_scp.shared;
	unsigned long flags;
	u32 size;

	spin_lock_irqsave(&touch_scp_rx_lock, flags);
	if (!touch_scp.active)
		goto out;
	switch (command) {
	case 0x11:
	case 0x12:
	case 0x14:
		if (!shared)
			break;
		rmb();
		size = READ_ONCE(shared->event_size);
		if (size > sizeof(touch_scp.param.data))
			break;
		touch_scp.param.event = READ_ONCE(shared->event);
		touch_scp.param.event_size = size;
		memcpy(touch_scp.param.data, shared->data, size);
		schedule_work(&touch_scp.gesture_work);
		break;
	case 0x13:
		WRITE_ONCE(touch_scp.param.state, 2);
		break;
	case 0x15:
		complete(&touch_scp.ack);
		break;
	case 0x16:
		if (touch_scp.param.state != 1 && touch_scp.param.state != 2) {
			WRITE_ONCE(touch_scp.param.state, 1);
			touch_scp.retries = 0;
			schedule_delayed_work(&touch_scp.param_work, 0);
		}
		break;
	}
out:
	spin_unlock_irqrestore(&touch_scp_rx_lock, flags);
	return 0;
}

static int touch_scp_ready(struct notifier_block *nb, unsigned long event, void *data)
{
	mutex_lock(&touch_scp_lock);
	if (!touch_scp.active)
		goto out;
	if (event == SCP_EVENT_READY) {
		WRITE_ONCE(touch_scp.param.state, 1);
		touch_scp.retries = 0;
		schedule_delayed_work(&touch_scp.param_work, 0);
	} else if (event == SCP_EVENT_STOP) {
		cancel_delayed_work(&touch_scp.param_work);
		if (touch_scp.param.state == 3) {
			pinctrl_select_state(touch_scp.pinctrl, touch_scp.ap_mode);
			touch_scp.ops.irq_owner(touch_scp.context, false);
		}
		WRITE_ONCE(touch_scp.param.state, 0);
	}
out:
	mutex_unlock(&touch_scp_lock);
	return NOTIFY_OK;
}

int xiaomi_touch_scp_register(struct device *dev, struct pinctrl *pinctrl,
	const struct xiaomi_touch_scp_param *param,
	const struct xiaomi_touch_scp_ops *ops, void *context)
{
	int ret;

	BUILD_BUG_ON(sizeof(*param) != 0x70);
	if (!dev || IS_ERR_OR_NULL(pinctrl) || !ops || !ops->gesture || !ops->irq_owner)
		return -EINVAL;
	mutex_lock(&touch_scp_lock);
	if (touch_scp.registered) {
		ret = -EBUSY;
		goto out;
	}
	touch_scp.ap_mode = pinctrl_lookup_state(pinctrl, "touch_mode_ap");
	touch_scp.scp_mode = pinctrl_lookup_state(pinctrl, "touch_mode_scp");
	if (IS_ERR(touch_scp.ap_mode) || IS_ERR(touch_scp.scp_mode)) {
		ret = -ENOENT;
		goto out;
	}
	touch_scp.physical = scp_get_reserve_mem_phys(SCP_THP_MEM_ID);
	touch_scp.shared = (void *)(unsigned long)scp_get_reserve_mem_virt(SCP_THP_MEM_ID);
	touch_scp.size = scp_get_reserve_mem_size(SCP_THP_MEM_ID);
	if (!touch_scp.shared || !touch_scp.physical || touch_scp.size < sizeof(*param)) {
		ret = -ENOMEM;
		goto out;
	}
	touch_scp.dev = dev;
	touch_scp.pinctrl = pinctrl;
	touch_scp.param = *param;
	touch_scp.ops = *ops;
	touch_scp.context = context;
	touch_scp.retries = 0;
	init_completion(&touch_scp.ack);
	INIT_WORK(&touch_scp.gesture_work, touch_scp_gesture_work);
	INIT_DELAYED_WORK(&touch_scp.param_work, touch_scp_param_work);
	touch_scp.notifier.notifier_call = touch_scp_ready;
	WRITE_ONCE(touch_scp.active, true);
	ret = mtk_ipi_register(&scp_ipidev, 55, touch_scp_receive, NULL, touch_scp.receive);
	if (ret)
		WRITE_ONCE(touch_scp.active, false);
	else
		touch_scp.registered = true;
out:
	mutex_unlock(&touch_scp_lock);
	if (!ret)
		scp_A_register_notify(&touch_scp.notifier);
	return ret;
}
EXPORT_SYMBOL_GPL(xiaomi_touch_scp_register);

void xiaomi_touch_scp_unregister(void)
{
	unsigned long flags;

	mutex_lock(&touch_scp_lock);
	if (!touch_scp.active) {
		mutex_unlock(&touch_scp_lock);
		return;
	}
	spin_lock_irqsave(&touch_scp_rx_lock, flags);
	WRITE_ONCE(touch_scp.active, false);
	spin_unlock_irqrestore(&touch_scp_rx_lock, flags);
	mutex_unlock(&touch_scp_lock);
	scp_A_unregister_notify(&touch_scp.notifier);
	mtk_ipi_unregister(&scp_ipidev, 55);
	cancel_delayed_work_sync(&touch_scp.param_work);
	cancel_work_sync(&touch_scp.gesture_work);
	mutex_lock(&touch_scp_lock);
	if (touch_scp.param.state == 3) {
		pinctrl_select_state(touch_scp.pinctrl, touch_scp.ap_mode);
		touch_scp.ops.irq_owner(touch_scp.context, false);
	}
	touch_scp.param.state = 0;
	touch_scp.registered = false;
	mutex_unlock(&touch_scp_lock);
}
EXPORT_SYMBOL_GPL(xiaomi_touch_scp_unregister);

bool xiaomi_touch_scp_owned(void)
{
	return READ_ONCE(touch_scp.active) && READ_ONCE(touch_scp.param.state) == 3;
}
EXPORT_SYMBOL_GPL(xiaomi_touch_scp_owned);

int xiaomi_touch_scp_switch(bool scp, u32 gestures)
{
	int ret;

	mutex_lock(&touch_scp_lock);
	if (!touch_scp.active || touch_scp.param.state < 2) {
		ret = -EAGAIN;
		goto out;
	}
	if (scp) {
		touch_scp.ops.irq_owner(touch_scp.context, true);
		ret = pinctrl_select_state(touch_scp.pinctrl, touch_scp.scp_mode);
		if (ret)
			goto restore;
		WRITE_ONCE(touch_scp.param.state, 3);
		ret = touch_scp_send(2, 1, gestures, 0);
		if (!ret)
			goto out;
	} else {
		reinit_completion(&touch_scp.ack);
		ret = touch_scp_send(2, 0, gestures, 0);
		if (!ret && !wait_for_completion_timeout(&touch_scp.ack, msecs_to_jiffies(50)))
			ret = -ETIMEDOUT;
	}
restore:
	pinctrl_select_state(touch_scp.pinctrl, touch_scp.ap_mode);
	touch_scp.ops.irq_owner(touch_scp.context, false);
	WRITE_ONCE(touch_scp.param.state, 4);
out:
	mutex_unlock(&touch_scp_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(xiaomi_touch_scp_switch);

int xiaomi_touch_scp_update_gesture(u32 gestures, u32 nonui)
{
	int ret = -EAGAIN;

	mutex_lock(&touch_scp_lock);
	if (xiaomi_touch_scp_owned())
		ret = touch_scp_send(5, gestures, nonui, 0);
	mutex_unlock(&touch_scp_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(xiaomi_touch_scp_update_gesture);

int xiaomi_touch_scp_log_level(u32 level)
{
	int ret = -ENODEV;

	mutex_lock(&touch_scp_lock);
	if (touch_scp.active && READ_ONCE(touch_scp.param.state) >= 2)
		ret = touch_scp_send(6, level, 0, 0);
	mutex_unlock(&touch_scp_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(xiaomi_touch_scp_log_level);


int xiaomi_touch_scp_read(u32 address, void *data, size_t size)
{
	unsigned long flags;
	int ret = -EHOSTDOWN;

	spin_lock_irqsave(&touch_scp_rx_lock, flags);
	if (!xiaomi_touch_scp_owned())
		goto out;
	if (address != touch_scp.param.gesture_address && address != touch_scp.param.fod_address)
		goto out;
	if (size > sizeof(touch_scp.param.data)) {
		ret = -EMSGSIZE;
		goto out;
	}
	memset(data, 0, size);
	memcpy(data, touch_scp.param.data, min_t(size_t, size, touch_scp.param.event_size));
	ret = 0;
out:
	spin_unlock_irqrestore(&touch_scp_rx_lock, flags);
	return ret;
}
EXPORT_SYMBOL_GPL(xiaomi_touch_scp_read);
