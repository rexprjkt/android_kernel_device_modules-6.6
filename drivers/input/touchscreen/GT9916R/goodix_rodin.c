// SPDX-License-Identifier: GPL-2.0
#include <linux/ktime.h>
#include <linux/input/mt.h>
#include <linux/rtc.h>
#include <linux/vmalloc.h>
#include "goodix_ts_core.h"
#include "../xiaomi_touch/xiaomi_touch_scp.h"

extern struct goodix_ts_core *goodix_core_data;
int goodix_short_open_test(void);

unsigned int goodix_debug_log_level = 3;

static int goodix_log_level(int level)
{
	goodix_debug_log_level = level;
	debug_log_flag = level > 3;
	if (!goodix_core_data->scp_mistouch_close)
		xiaomi_touch_scp_log_level(level);
	return level;
}

static int goodix_raw_enable(int enable)
{
	struct goodix_ts_core *cd = goodix_core_data;
	struct goodix_ts_cmd cmd = { .cmd = 0x90, .len = 6 };
	int ret;

	if (!cd)
		return -ENODEV;
	cmd.data[0] = 1;
	cmd.data[1] = !!enable;
	ret = cd->hw_ops->send_cmd(cd, &cmd);
	if (ret)
		return ret;
	memset(&cmd, 0, sizeof(cmd));
	cmd.cmd = 0x91;
	cmd.len = 5;
	cmd.data[0] = enable ? 0 : 0x81;
	ret = cd->hw_ops->send_cmd(cd, &cmd);
	if (!ret)
		WRITE_ONCE(cd->raw_enabled, !!enable);
	return ret;
}

int goodix_rodin_read_frame(struct goodix_ts_core *cd)
{
	u8 *frame = get_raw_data_base_common(0);
	struct timespec64 timestamp;
	size_t debug_size;
	int ret;

	if (!frame)
		return -ENODEV;
	ret = cd->hw_ops->read(cd, cd->ic_info.misc.frame_data_addr, frame + 24, 2500);
	if (ret)
		return ret;
	if (!(frame[24] & 0x80))
		return -EAGAIN;
	ktime_get_real_ts64(&timestamp);
	put_unaligned_le64(timespec64_to_ns(&timestamp), frame);
	put_unaligned_le64(cd->raw_frame_count, frame + 8);
	put_unaligned_le32(cd->fod_finger, frame + 16);
	put_unaligned_le32(0, frame + 20);
	put_unaligned_le32(cd->raw_debug_type, frame + 2524);
	if (cd->raw_debug_type == 1) {
		debug_size = cd->raw_debug_size;
		if (debug_size <= cd->ic_info.parm.drv_num * cd->ic_info.parm.sen_num * 2)
			debug_size = cd->ic_info.parm.drv_num * cd->ic_info.parm.sen_num * 2 + 0xb2;
		if (debug_size > 2 * PAGE_SIZE - 2528)
			return -EOVERFLOW;
		ret = cd->hw_ops->read(cd, cd->ic_info.misc.frame_data_addr,
				       frame + 2528, debug_size);
		if (ret)
			return ret;
	}
	notify_raw_data_update_common(0);
	cd->raw_frame_count++;
	return 0;
}

static int goodix_htc_write(common_data_t *data)
{
	struct goodix_ts_core *cd = goodix_core_data;

	if (!cd || !data->data_len || data->data_len > sizeof(data->data_buf) - 3)
		return -EFAULT;
	return cd->hw_ops->write(cd, data->data_buf[0],
				 (u8 *)&data->data_buf[1], data->data_len - 1);
}

static int goodix_htc_read(common_data_t *data)
{
	struct goodix_ts_core *cd = goodix_core_data;

	if (!cd || data->data_len > sizeof(data->data_buf) - sizeof(int))
		return -EFAULT;
	return cd->hw_ops->read(cd, data->data_buf[0],
				(u8 *)&data->data_buf[1], data->data_len);
}

static int goodix_lockdown_read(u8 *info)
{
	memcpy(info, goodix_core_data->lockdown_info, GOODIX_LOCKDOWN_SIZE);
	return 0;
}

static int goodix_version_read(char *version)
{
	struct goodix_fw_version *fw = &goodix_core_data->fw_version;

	goodix_core_data->hw_ops->read_version(goodix_core_data, fw);
	scnprintf(version, 64, "patch_pid:%.*s\npatch_vid:%02x%02x%02x%02x\nconfig_version:%x\n",
		  8, fw->patch_pid, fw->patch_vid[0], fw->patch_vid[1],
		  fw->patch_vid[2], fw->patch_vid[3], goodix_core_data->ic_info.version.config_version);
	return 0;
}

static int goodix_collect(char *output, int *length)
{
	struct goodix_ts_core *cd = goodix_core_data;
	struct ts_rawdata_info *info;
	static const char * const headings[] = {
		"mutual_rawdata:\n", "mutual_diffdata:\n", "mutual_refdata:\n",
	};
	int ret, rx, tx, count, set, i, used = 0;

	if (!cd || !output || !length)
		return -EINVAL;
	*length = 0;
	info = kvzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	ret = cd->hw_ops->get_capacitance_data(cd, info);
	if (ret < 0)
		goto out;
	rx = info->buff[0];
	tx = info->buff[1];
	count = rx * tx;
	if (rx <= 0 || tx <= 0 || count > (ARRAY_SIZE(info->buff) - 2) / 3) {
		ret = -EOVERFLOW;
		goto out;
	}
	used = scnprintf(output, 3 * PAGE_SIZE, "TX:%d  RX:%d\n", tx, rx);
	for (set = 0; set < 3; set++) {
		used += scnprintf(output + used, 3 * PAGE_SIZE - used, "%s", headings[set]);
		for (i = 0; i < count; i++) {
			used += scnprintf(output + used, 3 * PAGE_SIZE - used,
				set == 1 ? "%3d," : "%5d,", info->buff[2 + set * count + i]);
			if ((i + 1) % tx == 0)
				used += scnprintf(output + used, 3 * PAGE_SIZE - used, "\n");
		}
	}
	*length = used;
	ret = 0;
out:
	kvfree(info);
	return ret;
}

static int goodix_self_test(const char *command, int *result)
{
	struct goodix_ts_core *cd = goodix_core_data;
	struct goodix_fw_version version;

	if (!strncmp(command, "short", 5) || !strncmp(command, "open", 4))
		*result = goodix_short_open_test();
	else if (!strncmp(command, "i2c", 3))
		*result = cd->hw_ops->read_version(cd, &version) ? 1 : 2;
	else
		*result = 0;
	return 0;
}

static u8 goodix_panel_vendor(void) { return goodix_core_data->lockdown_info[0]; }
static u8 goodix_panel_color(void) { return goodix_core_data->lockdown_info[2]; }
static u8 goodix_panel_display(void) { return goodix_core_data->lockdown_info[1]; }
static char goodix_touch_vendor(void) { return '2'; }

static void goodix_charge(int charging)
{
	struct goodix_ts_core *cd = goodix_core_data;

	cd->charger_status = charging;
	if (!atomic_read(&cd->suspended))
		cd->hw_ops->charger_on(cd, !!charging);
}

static int goodix_palm(int enable)
{
	goodix_core_data->palm_status = !!enable;
	if (atomic_read(&goodix_core_data->suspended))
		return 0;
	return goodix_core_data->hw_ops->palm_on(goodix_core_data, !!enable);
}

static void goodix_gesture_state(int gestures)
{
	struct goodix_ts_core *cd = goodix_core_data;

	cd->gesture_enabled = (gestures & 1) << 2 | (gestures >> 1 & 3);
	cd->double_wakeup = !!(gestures & 4);
	cd->aod_status = !!(gestures & 2);
	cd->fod_status = driver_get_touch_mode_common(0, Touch_Fod_Enable);
	cd->fod_icon_status = driver_get_touch_mode_common(0, Touch_FodIcon_Enable);
	cd->nonui_status = driver_get_touch_mode_common(0, Touch_Nonui_Mode);
	goodix_gesture_enable(!!cd->gesture_enabled || cd->work_status == TP_GESTURE);
}

static void goodix_switch_gesture(int gestures)
{
	struct goodix_ts_core *cd = goodix_core_data;

	goodix_gesture_state(gestures);
	if (xiaomi_touch_scp_owned()) {
		xiaomi_touch_scp_update_gesture(cd->gesture_enabled, cd->nonui_status);
		return;
	}
	if (!atomic_read(&cd->suspended))
		return;
	mutex_lock(&cd->sleep_to_gesture_mutex);
	if (cd->gesture_enabled && cd->work_status == TP_SLEEP) {
		goodix_sleep_to_gesture(cd);
		if (cd->scp_registered && !cd->scp_mistouch_close)
			xiaomi_touch_scp_switch(true, cd->gesture_enabled);
	} else {
		cd->hw_ops->gesture(cd, cd->gesture_enabled);
	}
	mutex_unlock(&cd->sleep_to_gesture_mutex);
}

static int goodix_pm(int resume, int gestures)
{
	struct goodix_ts_core *cd = goodix_core_data;
	int ret;

	if (resume && xiaomi_touch_scp_owned())
		xiaomi_touch_scp_switch(false, cd->gesture_enabled);
	enable_temperature_detection_func(!!resume);
	goodix_gesture_state(gestures);
	ret = resume ? goodix_ts_resume(cd) : goodix_ts_suspend(cd);
	if (!ret && !resume && gestures) {
		if (cd->scp_registered && !cd->scp_mistouch_close)
			xiaomi_touch_scp_switch(true, cd->gesture_enabled);
		else if (cd->scp_mistouch_close) {
			struct goodix_ts_cmd cmd = { .cmd = 0x13, .len = 5, .data = { 1 } };

			cd->hw_ops->send_cmd(cd, &cmd);
		}
	}
	if (!ret && resume && cd->raw_debug_type) {
		struct goodix_ts_cmd cmd = { .cmd = 0xcd, .len = 5, .data = { 1 } };

		cd->hw_ops->send_cmd(cd, &cmd);
	}
	if (!ret && resume && cd->raw_enabled)
		ret = goodix_raw_enable(1);
	return ret;
}

static int goodix_set_value(int mode, int *values)
{
	struct goodix_ts_core *cd = goodix_core_data;
	struct goodix_ts_cmd cmd = { .len = 5 };
	int value = values[0];

	switch (mode) {
	case 100:
		cd->self_config_enabled = !!value;
		cd->self_config_pending = 1;
		return 0;
	case 103:
		cd->scp_mistouch_close = !value;
		return 0;
	case 200: {
		int unsupported = -1;

		add_common_data_to_buf_common(0, SET_CUR_VALUE, Touch_GAMETURBOTOOL_FOLLOWUP,
					     1, &unsupported);
		return 0;
	}
	case 201:
		cmd.cmd = 0x11;
		cmd.data[0] = !!value;
		break;
	case 202:
		cmd.cmd = 0xc1;
		cmd.len = 6;
		cmd.data[0] = !!value;
		break;
	case 1000:
		cmd.cmd = 0x9f;
		cmd.data[0] = value == 1;
		if (values[1]) {
			cmd.len = 9;
			cmd.data[0] = 5;
			put_unaligned_le16(values[1], cmd.data + 1);
			put_unaligned_le16(values[2], cmd.data + 3);
		}
		break;
	case 1001:
		cd->fod_finger = !!value;
		input_report_key(cd->input_dev, BTN_INFO, !!value);
		if (!value) {
			input_report_abs(cd->input_dev, ABS_MT_WIDTH_MAJOR, 0);
			input_report_abs(cd->input_dev, ABS_MT_WIDTH_MINOR, 0);
		}
		update_fod_press_status_common(!!value);
		input_sync(cd->input_dev);
		return 0;
	case 1011:
		cd->report_rate = value;
		cmd.cmd = 0x9d;
		cmd.data[0] = value == 60 ? 2 : value == 120 ? 4 :
			      value == 180 ? 3 : value == 240 ? 1 : 0;
		break;
	case 0x42c:
		cmd.cmd = 0xa8;
		cmd.len = 4;
		break;
	case 0x42e:
		cmd.cmd = 0x90;
		cmd.data[0] = 0x87;
		break;
	case 0x42f:
		cmd.cmd = 0xa2;
		cmd.len = 4;
		break;
	case 0x434:
		if (value < 0 || value > 1)
			return -EINVAL;
		cmd.cmd = 0xcd;
		cmd.data[0] = !!value;
		if (cd->hw_ops->send_cmd(cd, &cmd))
			return -EIO;
		cd->raw_debug_type = value;
		return 0;
	case 0x43c:
		cmd.cmd = 0xa1;
		cmd.data[0] = value;
		break;
	case 0x444:
		cmd.cmd = 0x72;
		cmd.len = 9;
		cmd.data[0] = value;
		put_unaligned_le16(values[1], cmd.data + 1);
		put_unaligned_le16(values[2], cmd.data + 3);
		break;
	case 0x44a:
		cmd.cmd = 0x9c;
		cmd.data[0] = value;
		break;
	case 0x44c:
		cmd.cmd = 0xbf;
		cmd.data[0] = value;
		break;
	case 0x44d:
		cmd.cmd = 0x86;
		cmd.data[0] = value;
		break;
	case 0x44e:
		cmd.cmd = 0xee;
		cmd.len = 6;
		cmd.data[0] = value;
		cmd.data[1] = values[1];
		break;
	case 0x44f:
		cmd.cmd = 0xd1;
		cmd.data[0] = value;
		break;
	default:
		dev_dbg(cd->bus->dev, "unhandled touch mode %d\n", mode);
		return 0;
	}
	return cd->hw_ops->send_cmd(cd, &cmd);
}

static void goodix_ic_buffer(u64 address, char *output)
{
	struct goodix_ts_core *cd = goodix_core_data;
	u8 bytes[256];
	unsigned int i, used = 0;

	if (cd->hw_ops->read(cd, cd->ic_info.misc.cmd_addr, bytes, sizeof(bytes)))
		return;
	for (i = 0; i < sizeof(bytes); i++) {
		used += scnprintf(output + used, PAGE_SIZE - used, "%02x ", bytes[i]);
		if ((i & 15) == 15)
			used += scnprintf(output + used, PAGE_SIZE - used, "\n");
	}
}

static void goodix_fod_test(int value)
{
	struct input_dev *input = goodix_core_data->input_dev;

	if (value) {
		input_report_key(input, BTN_INFO, 1);
		update_fod_press_status_common(1);
		input_sync(input);
		input_mt_slot(input, 0);
		input_mt_report_slot_state(input, MT_TOOL_FINGER, true);
		input_report_key(input, BTN_TOUCH, 1);
		input_report_key(input, BTN_TOOL_FINGER, 1);
		input_report_abs(input, ABS_MT_TRACKING_ID, 0);
		input_report_abs(input, ABS_MT_WIDTH_MINOR, 1);
		input_report_abs(input, ABS_MT_POSITION_X, 0x1c20);
		input_report_abs(input, ABS_MT_POSITION_Y, 0x7094);
	} else {
		input_mt_slot(input, 0);
		input_report_abs(input, ABS_MT_WIDTH_MINOR, 0);
		input_mt_report_slot_state(input, MT_TOOL_FINGER, false);
		input_report_abs(input, ABS_MT_TRACKING_ID, -1);
		input_report_key(input, BTN_INFO, 0);
		update_fod_press_status_common(0);
	}
	input_sync(input);
}

static int goodix_doze(int value)
{
	struct goodix_ts_core *cd = goodix_core_data;

	switch (value) {
	case POWER_RESET:
		return cd->hw_ops->reset(cd, 100);
	case RELOAD_FW:
		return goodix_do_fw_update(cd->ic_configs[CONFIG_TYPE_NORMAL],
			UPDATE_MODE_BLOCK | UPDATE_MODE_SRC_REQUEST | UPDATE_MODE_FORCE);
	case ENABLE_IRQ:
		return cd->hw_ops->irq_enable(cd, true);
	case DISABLE_IRQ:
		return cd->hw_ops->irq_enable(cd, false);
	case REGISTER_IRQ:
		return goodix_ts_irq_setup(cd);
	case IRQ_PIN_LEVEL:
		return !!gpio_get_value(cd->board_data.irq_gpio);
	case ENTER_SUSPEND:
		schedule_resume_suspend_work_common(0, 0);
		return 0;
	case ENTER_RESUME:
		schedule_resume_suspend_work_common(0, 1);
		return 0;
	case 8:
	case 9: {
		int ret = cd->hw_ops->power_on(cd, value == 8);

		if (!ret)
			cd->power_on = value == 8;
		return ret;
	}
	default:
		return -EINVAL;
	}
}

/* The stock callback has no controller transaction; the common driver
 * delivers temperature changes to the HAL.
 */
static int goodix_temperature(int temperature, bool read_bms)
{
	return 0;
}

static void goodix_game_update(u64 changed, int *values)
{
	struct goodix_ts_core *cd = goodix_core_data;

	if (!changed || cd->raw_enabled || cd->work_status == TP_SLEEP)
		return;
	mutex_lock(&cd->core_mutex);
	mutex_unlock(&cd->core_mutex);
}

static void goodix_scp_gesture(void *context)
{
	struct goodix_ts_core *cd = context;

	pm_stay_awake(cd->bus->dev);
	goodix_rodin_gesture(cd);
	pm_relax(cd->bus->dev);
}

static void goodix_scp_irq(void *context, bool scp)
{
	struct goodix_ts_core *cd = context;

	cd->hw_ops->irq_enable(cd, !scp);
	if (scp)
		synchronize_irq(cd->irq);
	if (atomic_read(&cd->suspended) && cd->gesture_enabled)
		irq_set_irq_wake(cd->irq, !scp);
}

static int goodix_dfs_test(int code)
{
	const char *fault;

	switch (code) {
	case 912001001: fault = "TpTransferErr"; break;
	case 912001002: fault = "TpFirmwareLoadFail"; break;
	case 912001003: fault = "TpParamParseFail"; break;
	case 912002001: fault = "TpOpenTestFail"; break;
	case 912002002: fault = "TpShortTestFail"; break;
	default: return 0;
	}
	if (code <= 912001002)
		xiaomi_touch_mievent_report_str_common(code, 0, fault, "goodix");
	else
		xiaomi_touch_mievent_report_int_common(code, 0, fault, "goodix",
			code == 912001003 ? 1 : 0);
	return 0;
}

int goodix_rodin_register(struct goodix_ts_core *cd)
{
	hardware_param_t param = {
		.x_resolution = cd->ic_info.other.screen_max_x ?: cd->board_data.panel_max_x ?: 1220,
		.y_resolution = cd->ic_info.other.screen_max_y ?: cd->board_data.panel_max_y ?: 2712,
		.rx_num = cd->ic_info.parm.sen_num ?: 17,
		.tx_num = cd->ic_info.parm.drv_num ?: 38,
		.frame_data_page_size = 2, .frame_data_buf_size = 10,
		.raw_data_page_size = 5, .raw_data_buf_size = 5,
		.temperature_change_threshold = 2,
	};
	hardware_operation_t ops = {
		.ic_data_collect = goodix_collect, .self_test = goodix_self_test, .lockdown_info_read = goodix_lockdown_read,
		.get_fw_version = goodix_version_read, .set_cur_value = goodix_set_value,
		.switch_mode = goodix_switch_gesture, .palm_sensor_write = goodix_palm,
		.enable_touch_raw = goodix_raw_enable, .panel_vendor_read = goodix_panel_vendor,
		.panel_color_read = goodix_panel_color, .panel_display_read = goodix_panel_display,
		.touch_vendor_read = goodix_touch_vendor,
		.log_level_control = goodix_log_level, .touch_dfs_test = goodix_dfs_test, .htc_ic_set_mode_value = goodix_htc_write,
		.htc_ic_get_mode_value = goodix_htc_read, .resume_suspend = goodix_pm,
		.set_charge_state = goodix_charge, .get_ic_buffer = goodix_ic_buffer,
		.fod_test = goodix_fod_test, .touch_doze_analysis = goodix_doze,
		.set_thermal_temp = goodix_temperature, .cmd_mode_update = goodix_game_update,
	};
	u32 factor = 100;
	int ret;
	struct xiaomi_touch_scp_param scp_param = {
		.vendor = 1, .gesture_address = cd->ic_info.misc.touch_data_addr,
		.acknowledge_address = cd->ic_info.misc.cmd_addr, .gesture_size = 32,
		.frame_address = cd->ic_info.misc.frame_data_addr, .frame_size = 2500,
	};
	const struct xiaomi_touch_scp_ops scp_ops = {
		.gesture = goodix_scp_gesture, .irq_owner = goodix_scp_irq,
	};

	of_property_read_u32(cd->bus->dev->of_node, "goodix,super-resolution-factor", &factor);
	param.super_resolution_factor = factor;
	memcpy(param.lockdown_info, cd->lockdown_info, sizeof(param.lockdown_info));
	strscpy(param.config_file_name, "rodin_gtp_thp_config.ini", sizeof(param.config_file_name));
	strscpy(param.driver_version, "gt9916-2025.07.24-01", sizeof(param.driver_version));
	goodix_version_read(param.fw_version);
	ret = xiaomi_touch_scp_register(cd->bus->dev, cd->pinctrl,
				       &scp_param, &scp_ops, cd);
	cd->scp_registered = !ret;
	ret = register_touch_panel_common(cd->bus->dev, 0, &param, &ops);
	if (ret) {
		if (cd->scp_registered)
			xiaomi_touch_scp_unregister();
		cd->scp_registered = false;
		return ret;
	}
	cd->common_registered = true;
	WRITE_ONCE(cd->raw_enabled, true);
	init_touch_irq(0, cd->bus->dev->of_node);
	return xiaomi_register_panel_notifier_common(cd->bus->dev, 0);
}
