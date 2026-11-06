// SPDX-License-Identifier: GPL-2.0
#include <linux/ktime.h>
#include <asm/unaligned.h>
#include "focaltech_core.h"
#include "../xiaomi_touch/xiaomi_touch_scp.h"

unsigned int fts_debug_log_level = 3;

static int focal_log_level(int level)
{
	fts_debug_log_level = level;
	if (!fts_data->scp_mistouch_close)
		xiaomi_touch_scp_log_level(level);
	return level;
}

static void focal_get_mode(common_data_t *data)
{
	data->data_buf[0] = fts_get_mode_value(data->mode, data->data_buf[0]);
}

static void focal_get_all(common_data_t *data)
{
	fts_get_mode_all(data->mode, data->data_buf);
}

static void focal_reset_mode(common_data_t *data)
{
	fts_reset_mode(data->mode);
}

static int focal_doze(int value)
{
	if (value == ENTER_SUSPEND || value == ENTER_RESUME) {
		schedule_resume_suspend_work_common(0, value == ENTER_RESUME);
		return 0;
	}
	return fts_touch_doze_analysis(value);
}

/* Stock HTC mode dispatcher, starting at 3001. */
static const u8 htc_registers[] = {
	0x10, 0x11, 0xff, 0xff, 0xff, 0xcf, 0x13, 0x14,
	0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0xff, 0x1b,
	0x1c, 0x1d, 0x99, 0x8b, 0, 0xd0, 0x21, 0x22,
	0x23, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x2f,
	0xff, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a,
	0x2b, 0xa5, 0x2d, 0x2e, 0x31, 0x30, 2, 2, 2, 0x3b,
};

static bool fts_byte_register(u8 reg)
{
	return reg == 0x8b || reg == 0x99 || reg == 0xa5 ||
	       reg == 0xcf || reg == 0xd0 || reg == 0xd1;
}

static int fts_htc_write(common_data_t *data)
{
	u8 bytes[CMD_DATA_BUF_SIZE];
	u8 cmd[3];
	unsigned int i, index = data->mode - 3001;
	int value = data->data_buf[0];

	if (index >= ARRAY_SIZE(htc_registers) || data->mode == 3021 ||
	    data->mode == 3050 || data->data_len > ARRAY_SIZE(bytes))
		return -EINVAL;
	cmd[0] = htc_registers[index];
	if (data->mode == 3045) {
		for (i = 0; i < data->data_len; i++)
			bytes[i] = data->data_buf[i];
		return fts_write(bytes, data->data_len);
	}
	if (fts_byte_register(cmd[0])) {
		if (cmd[0] == 0xa5 && value == 2)
			value = 0;
		if (cmd[0] == 0xd1) {
			u8 reg_value;
			int ret = fts_read_reg(0xd1, &reg_value);

			if (ret < 0)
				return ret;
			value = reg_value | (value & 0x90);
		}
		return fts_write_reg(cmd[0], value);
	}
	cmd[1] = value < 256 ? 0 : value >> 8;
	cmd[2] = value;
	return fts_write(cmd, sizeof(cmd));
}

static int fts_htc_read(common_data_t *data)
{
	u8 *bytes = (u8 *)data->data_buf;
	u8 reg;
	unsigned int index = data->mode - 3001, i;
	int ret;

	if (index >= ARRAY_SIZE(htc_registers) || data->mode == 3021 ||
	    !data->data_len || data->data_len > sizeof(data->data_buf))
		return -EINVAL;
	reg = htc_registers[index];
	ret = fts_read(&reg, 1, bytes, data->data_len);
	if (ret < 0)
		return ret;
	if (fts_byte_register(reg)) {
		u8 value = bytes[0];

		if (data->data_len < 2)
			return -EINVAL;
		if (reg == 0xa5)
			value = value == 0 ? 0 : value == 1 ? 1 : 3;
		bytes[0] = 0;
		bytes[1] = value;
	}
	for (i = 0; i + 1 < data->data_len; i += 2)
		if (data->mode != 3050)
			swap(bytes[i], bytes[i + 1]);
	if (data->mode == 3050) {
		static const u8 widths[] = { 2, 2, 2, 2, 2, 2, 1, 1, 1, 1,
			2, 2, 2, 2, 8, 1, 2, 2, 2 };
		unsigned int offset = 0, j;

		for (i = 0; i < ARRAY_SIZE(widths); i++) {
			if (offset + widths[i] > data->data_len)
				return -EINVAL;
			if (i == 14 || (BIT(i) & 0x73c3f))
				for (j = 0; j + 1 < widths[i]; j += 2)
					swap(bytes[offset + j], bytes[offset + j + 1]);
			offset += widths[i];
		}
	}
	return 0;
}

int fts_rodin_enable_raw(int enable)
{
	u8 value = enable ? 0x41 : 0;
	u8 readback;
	int ret = -EINVAL, retry;

	for (retry = 0; retry < 3; retry++) {
		ret = fts_write_reg(0x9e, value);
		if (ret >= 0) {
			ret = fts_read_reg(0x9e, &readback);
			if (ret >= 0 && readback == value) {
				WRITE_ONCE(fts_data->raw_enabled, !!enable);
				dev_info(fts_data->dev, "raw mode=%d register 0x9e=%02x\n", !!enable, readback);
				return 0;
			}
		}
		msleep(1);
	}
	return ret < 0 ? ret : -EINVAL;
}

int fts_rodin_read_frame(struct fts_ts_data *ts)
{
	u8 *frame = get_raw_data_base_common(0);
	u8 reg = 1;
	struct timespec64 timestamp;
	int ret;

	if (!frame)
		return -ENODEV;
	ret = fts_read(&reg, 1, frame + 24, 0xc30);
	if (ret < 0)
		return ret;
	if (!frame[24] || !frame[25]) {
		dev_warn_once(ts->dev, "raw frame header rejected: %02x %02x; frame not published\n",
			      frame[24], frame[25]);
		return -EAGAIN;
	}
	ktime_get_real_ts64(&timestamp);
	put_unaligned_le64(timespec64_to_ns(&timestamp), frame);
	put_unaligned_le64(ts->raw_frame_count, frame + 8);
	put_unaligned_le32(ts->finger_in_fod, frame + 16);
	put_unaligned_le32(0, frame + 20);
	put_unaligned_le32(ts->raw_debug_type, frame + 0x1018);
	memset(frame + 0x101c, 0, 0x80a);
	if (ts->raw_debug_type) {
		reg = 0x3d;
		ret = fts_read(&reg, 1, frame + 0x101c, 0x80a);
		if (ret < 0)
			dev_dbg(ts->dev, "raw debug read failed: %d\n", ret);
	}
	notify_raw_data_update_common(0);
	if (!ts->raw_frame_count)
		dev_info(ts->dev, "first raw frame published: header=%02x %02x\n", frame[24], frame[25]);
	ts->raw_frame_count++;
	return 0;
}

static int fts_lockdown(u8 *info)
{
	memcpy(info, fts_data->lockdown_info, 8);
	return 0;
}

static int fts_version(char *version)
{
	u8 value;
	int ret = fts_read_reg(0xa6, &value);

	if (ret < 0 || value == 0 || value == 0xff)
		strscpy(version, "get tp fw version fail\n", 64);
	else
		scnprintf(version, 64, "%02x\n", value);
	return ret;
}

static u8 fts_vendor(void) { return fts_data->lockdown_info[0]; }
static u8 fts_color(void) { return fts_data->lockdown_info[2]; }
static u8 fts_display(void) { return fts_data->lockdown_info[1]; }
static char fts_touch_vendor(void) { return '3'; }

static int fts_thermal(int temperature, bool read_bms)
{
	if (read_bms) {
		temperature = get_bms_temp_common();
		if (abs(temperature) >= 1000)
			return -EINVAL;
		temperature = (temperature + 5) / 10;
		add_common_data_to_buf_common(0, SET_CUR_VALUE, 0x446, 1, &temperature);
	}
	return fts_write_reg(0x97, temperature);
}

static void fts_charge(int charging)
{
	fts_data->charger_status = charging;
	fts_data->charger_mode = !!charging;
	if (!fts_data->suspended)
		fts_write_reg(0x8b, !!charging);
}

static void fts_gesture_state(int gesture)
{
	fts_data->doubletap_status = !!(gesture & 4);
	fts_data->aod_status = !!(gesture & 2);
	fts_data->fod_status = driver_get_touch_mode_common(0, Touch_Fod_Enable);
	fts_data->fod_icon_status = driver_get_touch_mode_common(0, Touch_FodIcon_Enable);
	fts_data->nonui_status = driver_get_touch_mode_common(0, Touch_Nonui_Mode);
	fts_update_gesture_state(fts_data, GESTURE_DOUBLETAP, !!(gesture & 4));
	fts_update_gesture_state(fts_data, GESTURE_AOD, !!(gesture & 2));
	fts_data->gesture_support = !!gesture;
}

static void fts_switch_gesture(int gesture)
{
	fts_gesture_state(gesture);
	if (xiaomi_touch_scp_owned()) {
		xiaomi_touch_scp_update_gesture((gesture & 1) << 2 | (gesture >> 1 & 3),
			fts_data->nonui_status);
		return;
	}
	if ((gesture & 1) && fts_data->suspended &&
	    fts_data->poweroff_on_sleep && fts_data->fod_status == 1) {
		fts_recover_gesture_from_sleep(fts_data);
		fts_data->poweroff_on_sleep = false;
		if (fts_data->scp_registered && !fts_data->scp_mistouch_close)
			xiaomi_touch_scp_switch(true, (gesture & 1) << 2 | (gesture >> 1 & 3));
		return;
	}
	fts_fod_reg_write(2, !!(gesture & 1));
	if ((gesture & 1) && (fts_data->fod_status == 1 || fts_data->fod_status == 2))
		fts_fod_recovery();
}

static int fts_pm(int resume, int gesture)
{
	bool was_gesture = fts_data->gesture_support;
	int ret;

	if (resume && xiaomi_touch_scp_owned())
		xiaomi_touch_scp_switch(false, (gesture & 1) << 2 | (gesture >> 1 & 3));
	enable_temperature_detection_func(!!resume);
	fts_gesture_state(gesture);
	/* Resume must balance the IRQ wake reference taken by suspend. */
	if (resume)
		fts_data->gesture_support = was_gesture;
	ret = resume ? fts_ts_resume(fts_data->dev) : fts_ts_suspend(fts_data->dev);
	if (resume)
		fts_data->gesture_support = !!gesture;
	if (!ret && !resume && fts_data->gesture_support) {
		if (fts_data->scp_registered && !fts_data->scp_mistouch_close)
			xiaomi_touch_scp_switch(true, (gesture & 1) << 2 | (gesture >> 1 & 3));
		else if (fts_data->scp_mistouch_close) {
			u8 state = 0xff;
			int retry;

			for (retry = 0; retry < 5; retry++) {
				fts_write_reg(0xbf, 0);
				msleep(1);
				if (!fts_read_reg(0xbf, &state) && !state)
					break;
			}
		}
	}
	if (!ret && resume) {
		fts_thermal(0, true);
		if (fts_data->raw_enabled)
			ret = fts_rodin_enable_raw(1);
	}
	return ret;
}

static int fts_palm(int enable)
{
	fts_data->palm_sensor_switch = !!enable;
	if (fts_data->suspended)
		return 0;
	return fts_write_reg(0x9a, enable ? 5 : 0);
}

static int fts_set_value(int mode, int *value)
{
	u8 cmd[5];
	int ret, enabled;
	common_data_t data = { .data_len = 1 };

	switch (mode) {
	case 103:
		fts_data->scp_mistouch_close = !value[0];
		return 0;
	case 200: {
		int unsupported = -1;

		add_common_data_to_buf_common(0, SET_CUR_VALUE, Touch_GAMETURBOTOOL_FOLLOWUP,
			1, &unsupported);
		return 0;
	}
	case 201:
		return fts_write_reg(0x94, !!value[0]);
	case 1000:
		enabled = value[0] == 1 && value[1] && value[2];
		cmd[0] = 0x35; cmd[1] = 0; cmd[2] = enabled;
		ret = fts_write(cmd, 3);
		if (ret < 0)
			return ret;
		if (enabled) {
			cmd[0] = 0x36;
			put_unaligned_be16(value[1], cmd + 1);
			ret = fts_write(cmd, 3);
			if (ret < 0)
				return ret;
			cmd[0] = 0x37;
			put_unaligned_be16(value[2], cmd + 1);
			ret = fts_write(cmd, 3);
			if (ret < 0)
				return ret;
		}
		return fts_write_reg(0xa5, value[0] == 2 ? 0 : value[0]);
	case 1001:
		update_fod_press_status_common(!!value[0]);
		if (!value[0])
			fts_data->finger_in_fod = false;
		return 0;
	case 1011:
		fts_data->report_rate_status = value[0];
		data.mode = 3007;
		data.data_buf[0] = value[0];
		return fts_htc_write(&data);
	case 0x42f:
		cmd[0] = 0x33; cmd[1] = 0; cmd[2] = 1;
		return fts_write(cmd, 3);
	case 0x434:
		if (value[0] < 0 || value[0] > 1)
			return -EINVAL;
		fts_data->raw_debug_type = !!value[0];
		return 0;
	case 0x43c:
		cmd[0] = 0x38; cmd[1] = value[0]; cmd[2] = 0;
		return fts_write(cmd, 3);
	case 0x444:
		ret = fts_write_reg(0xc0, value[0]);
		if (ret < 0)
			return ret;
		cmd[0] = 0x95;
		put_unaligned_be16(value[1], cmd + 1);
		put_unaligned_be16(value[2], cmd + 3);
		return fts_write(cmd, 5);
	case 0x44a:
		cmd[0] = 0x2b; cmd[1] = 0; cmd[2] = value[0] & 0x7f;
		ret = fts_write(cmd, 3);
		if (ret < 0)
			return ret;
		cmd[0] = 0x15; cmd[2] = value[0] & 0x80;
		return fts_write(cmd, 3);
	case 0x44b:
		cmd[0] = value[0] == 17 || value[0] == 18 ? 0x1c : 0x19;
		cmd[1] = 0; cmd[2] = value[0] & 0x0f;
		ret = fts_write(cmd, 3);
		if (ret < 0 || cmd[0] == 0x1c)
			return ret;
		cmd[0] = 0x1a; cmd[2] = value[0] & 0xf0;
		return fts_write(cmd, 3);
	case 0x44d:
		return fts_write_reg(0x9d, value[0]);
	case 0x44f:
		return fts_write_reg(0x98, value[0]);
	default:
		dev_dbg(fts_data->dev, "unhandled touch mode %d\n", mode);
		return 0;
	}
}

static void fts_game_update(u64 changed, int *values)
{
	common_data_t data = { .mode = 3019, .data_len = 1 };

	if (!fts_data->raw_enabled) {
		fts_data->report_rate_status = values[Touch_Game_Mode] ? 240 : 135;
		data.mode = 3007;
		data.data_buf[0] = fts_data->report_rate_status;
		fts_htc_write(&data);
		fts_rodin_game_values(changed, values);
		return;
	}
	data.data_buf[0] = values[Touch_Game_Mode];
	fts_htc_write(&data);
	data.mode = 3007;
	data.data_buf[0] = fts_data->report_rate_status;
	fts_htc_write(&data);

}

static void focal_fod_test(int value)
{
	struct input_dev *input = fts_data->input_dev;

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
		input_report_abs(input, ABS_MT_POSITION_X, 0x2610);
		input_report_abs(input, ABS_MT_POSITION_Y, 0x9850);
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

static void focal_scp_gesture(void *context)
{
	struct fts_ts_data *ts = context;

	pm_stay_awake(ts->dev);
	fts_read_and_report_foddata(ts);
	fts_gesture_readdata(ts, NULL);
	pm_relax(ts->dev);
}

static void focal_scp_irq(void *context, bool scp)
{
	struct fts_ts_data *ts = context;

	if (scp) {
		fts_irq_disable();
		synchronize_irq(ts->irq);
	} else
		fts_irq_enable();
	if (ts->suspended && ts->gesture_support)
		irq_set_irq_wake(ts->irq, !scp);
}

static int focal_dfs_test(int code)
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
		xiaomi_touch_mievent_report_str_common(code, 0, fault, "focal");
	else
		xiaomi_touch_mievent_report_int_common(code, 0, fault, "focal",
			code == 912001003 ? 1 : 0);
	return 0;
}

int fts_rodin_register(struct fts_ts_data *ts)
{
	hardware_param_t param = {
		.x_resolution = 1220, .y_resolution = 2712, .rx_num = 17, .tx_num = 38,
		.frame_data_page_size = 2, .frame_data_buf_size = 10,
		.raw_data_page_size = 5, .raw_data_buf_size = 5,
		.temperature_change_threshold = 2,
	};
	hardware_operation_t ops = {
		.ic_data_collect = fts_rodin_collect,
		.get_mode_value = focal_get_mode, .get_mode_all = focal_get_all,
		.reset_mode = focal_reset_mode, .self_test = fts_rodin_self_test, .lockdown_info_read = fts_lockdown,
		.get_fw_version = fts_version, .set_cur_value = fts_set_value,
		.switch_mode = fts_switch_gesture, .cmd_mode_update = fts_game_update,
		.enable_touch_raw = fts_rodin_enable_raw, .palm_sensor_write = fts_palm,
		.panel_vendor_read = fts_vendor, .panel_color_read = fts_color,
		.panel_display_read = fts_display, .touch_vendor_read = fts_touch_vendor,
		.touch_dfs_test = focal_dfs_test,
		.htc_ic_set_mode_value = fts_htc_write, .htc_ic_get_mode_value = fts_htc_read,
		.resume_suspend = fts_pm, .set_charge_state = fts_charge,
		.set_thermal_temp = fts_thermal, .fod_test = focal_fod_test,
		.touch_doze_analysis = focal_doze, .log_level_control = focal_log_level,
	};
	u32 factor = 100;
	int ret;
	const struct xiaomi_touch_scp_param scp_param = {
		.vendor = 2, .gesture_address = 0xd3, .gesture_size = 26,
		.fod_address = 0xe1, .fod_size = 9, .frame_address = 1, .frame_size = 0xc30,
	};
	const struct xiaomi_touch_scp_ops scp_ops = {
		.gesture = focal_scp_gesture, .irq_owner = focal_scp_irq,
	};

	of_property_read_u32(ts->dev->of_node, "focaltech,super-resolution-factors", &factor);
	param.super_resolution_factor = factor;
	memcpy(param.lockdown_info, ts->lockdown_info, sizeof(param.lockdown_info));
	strscpy(param.config_file_name, "rodin_fts_thp_config.ini", sizeof(param.config_file_name));
	strscpy(param.driver_version, "FT3683:2025.04.23-001", sizeof(param.driver_version));
	fts_version(param.fw_version);
	ret = xiaomi_touch_scp_register(ts->dev, ts->pinctrl, &scp_param, &scp_ops, ts);
	ts->scp_registered = !ret;
	ret = register_touch_panel_common(ts->dev, 0, &param, &ops);
	if (ret) {
		if (ts->scp_registered)
			xiaomi_touch_scp_unregister();
		ts->scp_registered = false;
		return ret;
	}
	ts->common_registered = true;
	WRITE_ONCE(ts->raw_enabled, true);
	init_touch_irq(0, ts->dev->of_node);
	return xiaomi_register_panel_notifier_common(ts->dev, 0);
}
