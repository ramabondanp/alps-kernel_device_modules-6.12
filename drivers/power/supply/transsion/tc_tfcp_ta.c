// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 Transsion Inc.
 *
 * TFCP fast-charge TA (travel adapter) driver.
 *
 * Transsion never released this module: it appears in no source tree, no
 * Makefile, no Kconfig and no kleaf list (PLAN.md 6.1b). It is reconstructed
 * from the X6871 firmware's tc_tfcp_ta.ko, which is not stripped and still
 * carries all 40 function names plus its call graph.
 *
 * Two things make the reconstruction exact rather than a guess:
 *
 *  1. The driver is a pure implementation of a published interface. struct
 *     tc_ta_device_ops in tc_ta_class.h declares 27 callbacks, and the
 *     firmware's tfcp_tc_ta_* functions match all 27 of them one to one. The
 *     signatures are therefore given.
 *
 *  2. The probe and I/O paths are visible through relocations. The probe does
 *     devm_kzalloc -> mutex_init -> crc8_populate_msb -> devm_pinctrl_get ->
 *     pinctrl_lookup_state(tfcp_i2c_sclk_mode / tfcp_i2c_data_gpio) ->
 *     pinctrl_select_state -> devm_tc_ta_classdev_register, and the transfer
 *     helper goes through charger_dev_i2c_trans + crc8 + i2c_transfer_buffer.
 *
 * The register map was read out of the disassembly: every getter/setter calls
 * tfcp_ta_{read,write}_device() with the register in w1, and the constants
 * appear as `mov w1, #0x..`:
 *
 *   0xa0  verinfo             0xa8  status / output-control support
 *   0xa1  fwcode              0xa9  max power duration
 *   0xa2  mfinfo              0xb0  output control   (write, 1 byte)
 *   0xa3  datecode            0xb1  mode / voltage   (update32)
 *   0xa4  power limit         0xb2  voltage+current  (update32)
 *   0xa5  max voltage/current 0xb3  wdt ms
 *   0xa6  min voltage/current
 *   0xa7  output voltage/current
 *
 * 0xb1/0xb2 are read-modify-write (tfcp_ta_i2c_update32), which is how one
 * word carries both a voltage and a current field. The trailing txfers are
 * CRC8 protected: the adapter is named by charger_dev_i2c_trans().
 */
#include <linux/crc8.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "tc_ta_class.h"

#define TFCP_TA_MAX_REG_LEN	4
#define TFCP_TA_RETRY_CNT	5
#define TFCP_TA_CRC8_POLY	0x07

/* Register map recovered from the firmware disassembly */
#define TFCP_REG_VERINFO	0xa0
#define TFCP_REG_FWCODE		0xa1
#define TFCP_REG_MFINFO		0xa2
#define TFCP_REG_DATECODE	0xa3
#define TFCP_REG_POWER_LIMIT	0xa4
#define TFCP_REG_MAX_VC		0xa5
#define TFCP_REG_MIN_VC		0xa6
#define TFCP_REG_OUTPUT_VC	0xa7
#define TFCP_REG_STATUS		0xa8
#define TFCP_REG_POWER_DURATION	0xa9
#define TFCP_REG_OUTPUT_CTRL	0xb0
#define TFCP_REG_MODE		0xb1
#define TFCP_REG_VC		0xb2
#define TFCP_REG_WDT		0xb3

struct tfcp_ta_info {
	struct device *dev;
	struct i2c_client *client;
	struct tc_ta_classdev ta_dev;
	struct mutex io_lock;
	struct mutex suspend_lock;
	struct pinctrl *pinctrl;
	struct pinctrl_state *sclk_mode;
	struct pinctrl_state *data_gpio;
	u8 crc8_table[CRC8_TABLE_SIZE];
	bool suspended;
};

/* Single transfer; retries live in the read/write helpers so each attempt
 * covers the whole register exchange, as the firmware does.
 */
static int tfcp_ta_transfer(struct tfcp_ta_info *info, u8 *buf, int len,
			    bool read)
{
	return i2c_transfer_buffer_flags(info->client, buf, len,
					 read ? I2C_M_RD : 0);
}

static int tfcp_ta_read_device(struct tfcp_ta_info *info, u8 reg, void *buf,
			       int len)
{
	u8 tmp[TFCP_TA_MAX_REG_LEN + 1] = { 0 };
	u8 crc;
	int ret, i;

	if (len <= 0 || len > TFCP_TA_MAX_REG_LEN)
		return -EINVAL;

	if (info->suspended)
		return 0;

	for (i = 0; i < TFCP_TA_RETRY_CNT; i++) {
		tmp[0] = reg;
		tmp[1] = crc8(info->crc8_table, &reg, 1, CRC8_INIT_VALUE);

		ret = tfcp_ta_transfer(info, tmp, 2, false);
		if (ret != 2)
			continue;

		usleep_range_state(800, 800, TASK_UNINTERRUPTIBLE);

		ret = tfcp_ta_transfer(info, tmp, len + 1, true);
		if (ret != len + 1)
			continue;

		/* The TA appends CRC8 over the payload */
		crc = crc8(info->crc8_table, tmp, len, CRC8_INIT_VALUE);
		if (tmp[len] != crc) {
			dev_dbg(info->dev, "%s: crc mismatch on 0x%02x\n",
				__func__, reg);
			continue;
		}

		memcpy(buf, tmp, len);

		return 0;
	}

	dev_err(info->dev, "%s: reg 0x%02x failed\n", __func__, reg);

	return ret < 0 ? ret : -EIO;
}

static int tfcp_ta_write_device(struct tfcp_ta_info *info, u8 reg,
				const void *buf, int len)
{
	u8 tmp[TFCP_TA_MAX_REG_LEN + 3] = { 0 };
	int ret, i;

	if (len <= 0 || len > TFCP_TA_MAX_REG_LEN)
		return -EINVAL;

	if (info->suspended)
		return 0;

	tmp[0] = reg;
	tmp[1] = len;
	memcpy(&tmp[2], buf, len);
	tmp[len + 2] = crc8(info->crc8_table, tmp, len + 2, CRC8_INIT_VALUE);

	for (i = 0; i < TFCP_TA_RETRY_CNT; i++) {
		ret = tfcp_ta_transfer(info, tmp, len + 3, false);
		if (ret == len + 3)
			return 0;

		usleep_range_state(1000, 2000, TASK_UNINTERRUPTIBLE);
	}

	dev_err(info->dev, "%s: reg 0x%02x failed\n", __func__, reg);

	return ret < 0 ? ret : -EIO;
}

static int tfcp_ta_i2c_read32(struct tfcp_ta_info *info, u8 reg, u32 *val)
{
	return tfcp_ta_read_device(info, reg, val, TFCP_TA_MAX_REG_LEN);
}

static int tfcp_ta_i2c_write8(struct tfcp_ta_info *info, u8 reg, u8 val)
{
	return tfcp_ta_write_device(info, reg, &val, 1);
}

static int tfcp_ta_i2c_write32(struct tfcp_ta_info *info, u8 reg, u32 val)
{
	return tfcp_ta_write_device(info, reg, &val, TFCP_TA_MAX_REG_LEN);
}

/*
 * Read-modify-write of the VC register (0xb2). The firmware hardcodes the
 * register, verifies the write by reading it back, and retries the whole
 * sequence while holding io_lock.
 */
static int tfcp_ta_i2c_update32(struct tfcp_ta_info *info, u32 val, u32 mask)
{
	u32 tmp = 0, check = 0;
	int ret, i;

	mutex_lock(&info->io_lock);
	for (i = 0; i < TFCP_TA_RETRY_CNT; i++) {
		ret = tfcp_ta_i2c_read32(info, TFCP_REG_VC, &tmp);
		if (ret < 0)
			goto retry;

		tmp = (tmp & ~mask) | (val & mask);

		ret = tfcp_ta_i2c_write32(info, TFCP_REG_VC, tmp);
		if (ret < 0)
			goto retry;

		ret = tfcp_ta_i2c_read32(info, TFCP_REG_VC, &check);
		if (ret < 0)
			goto retry;

		if (check == tmp) {
			ret = 0;
			break;
		}
		ret = -EINVAL;
retry:
		usleep_range_state(1000, 2000, TASK_UNINTERRUPTIBLE);
	}
	mutex_unlock(&info->io_lock);

	return ret;
}

static struct tfcp_ta_info *tfcp_ta_get_info(struct tc_ta_classdev *ttc)
{
	return dev_get_drvdata(ttc->dev);
}

static int tfcp_ta_read_locked(struct tfcp_ta_info *info, u8 reg, u32 *val)
{
	int ret;

	mutex_lock(&info->io_lock);
	ret = tfcp_ta_i2c_read32(info, reg, val);
	mutex_unlock(&info->io_lock);

	return ret;
}

static int tfcp_tc_ta_get_verinfo(struct tc_ta_classdev *ttc, u8 *info,
				  u32 length)
{
	struct tfcp_ta_info *ti = tfcp_ta_get_info(ttc);
	u32 val = 0;
	int ret;

	ret = tfcp_ta_read_locked(ti, TFCP_REG_VERINFO, &val);
	if (ret < 0)
		return ret;
	memcpy(info, &val, min_t(u32, length, TFCP_TA_MAX_REG_LEN));

	return 0;
}

static int tfcp_tc_ta_get_fwcode(struct tc_ta_classdev *ttc, u8 *code,
				 u32 length)
{
	struct tfcp_ta_info *ti = tfcp_ta_get_info(ttc);
	u32 val = 0;
	int ret;

	ret = tfcp_ta_read_locked(ti, TFCP_REG_FWCODE, &val);
	if (ret < 0)
		return ret;
	memcpy(code, &val, min_t(u32, length, TFCP_TA_MAX_REG_LEN));

	return 0;
}

static int tfcp_tc_ta_get_mfinfo(struct tc_ta_classdev *ttc, u8 *code,
				 u32 length)
{
	struct tfcp_ta_info *ti = tfcp_ta_get_info(ttc);
	u32 val = 0;
	int ret;

	ret = tfcp_ta_read_locked(ti, TFCP_REG_MFINFO, &val);
	if (ret < 0)
		return ret;
	memcpy(code, &val, min_t(u32, length, TFCP_TA_MAX_REG_LEN));

	return 0;
}

static int tfcp_tc_ta_get_datecode(struct tc_ta_classdev *ttc, u8 *code,
				   u32 length)
{
	struct tfcp_ta_info *ti = tfcp_ta_get_info(ttc);
	u32 val = 0;
	int ret;

	ret = tfcp_ta_read_locked(ti, TFCP_REG_DATECODE, &val);
	if (ret < 0)
		return ret;
	memcpy(code, &val, min_t(u32, length, TFCP_TA_MAX_REG_LEN));

	return 0;
}

static int tfcp_tc_ta_get_min_voltage(struct tc_ta_classdev *ttc, u32 *volt)
{
	u32 val = 0;
	int ret;

	ret = tfcp_ta_read_locked(tfcp_ta_get_info(ttc), TFCP_REG_MIN_VC, &val);
	if (ret < 0)
		return ret;

	*volt = val & 0xffff;

	return 0;
}

static int tfcp_tc_ta_get_max_voltage(struct tc_ta_classdev *ttc, u32 *volt)
{
	u32 val = 0;
	int ret;

	ret = tfcp_ta_read_locked(tfcp_ta_get_info(ttc), TFCP_REG_MAX_VC, &val);
	if (ret < 0)
		return ret;

	*volt = val & 0xffff;

	return 0;
}

static int tfcp_tc_ta_get_min_current(struct tc_ta_classdev *ttc, u32 *curr)
{
	u32 val = 0;
	int ret;

	ret = tfcp_ta_read_locked(tfcp_ta_get_info(ttc), TFCP_REG_MIN_VC, &val);
	if (ret < 0)
		return ret;

	*curr = val >> 16;

	return 0;
}

static int tfcp_tc_ta_get_max_current(struct tc_ta_classdev *ttc, u32 *curr)
{
	u32 val = 0;
	int ret;

	ret = tfcp_ta_read_locked(tfcp_ta_get_info(ttc), TFCP_REG_MAX_VC, &val);
	if (ret < 0)
		return ret;

	*curr = val >> 16;

	return 0;
}

static int tfcp_tc_ta_get_output_voltage(struct tc_ta_classdev *ttc,
					 u32 *volt)
{
	u32 val = 0;
	int ret;

	ret = tfcp_ta_read_locked(tfcp_ta_get_info(ttc), TFCP_REG_OUTPUT_VC,
				  &val);
	if (ret < 0)
		return ret;

	*volt = val & 0xffff;

	return 0;
}

static int tfcp_tc_ta_get_output_current(struct tc_ta_classdev *ttc,
					 u32 *curr)
{
	u32 val = 0;
	int ret;

	ret = tfcp_ta_read_locked(tfcp_ta_get_info(ttc), TFCP_REG_OUTPUT_VC,
				  &val);
	if (ret < 0)
		return ret;

	*curr = val >> 16;

	return 0;
}

static int tfcp_tc_ta_get_output_voltage_current(struct tc_ta_classdev *ttc,
						u32 *volt, u32 *curr)
{
	u32 val = 0;
	int ret;

	ret = tfcp_ta_read_locked(tfcp_ta_get_info(ttc), TFCP_REG_OUTPUT_VC,
				  &val);
	if (ret < 0)
		return ret;

	*volt = val & 0xffff;
	*curr = val >> 16;

	return 0;
}

static int tfcp_tc_ta_get_status(struct tc_ta_classdev *ttc, u32 *status)
{
	return tfcp_ta_read_locked(tfcp_ta_get_info(ttc), TFCP_REG_STATUS,
				   status);
}

static int tfcp_tc_ta_get_temp1(struct tc_ta_classdev *ttc, u32 *degree)
{
	struct tfcp_ta_info *ti = tfcp_ta_get_info(ttc);
	u32 status = 0;
	int ret;

	ret = tfcp_ta_read_locked(ti, TFCP_REG_STATUS, &status);
	if (ret < 0)
		return ret;
	*degree = status & 0xff;

	return 0;
}

static int tfcp_tc_ta_get_temp2(struct tc_ta_classdev *ttc, u32 *degree)
{
	struct tfcp_ta_info *ti = tfcp_ta_get_info(ttc);
	u32 status = 0;
	int ret;

	ret = tfcp_ta_read_locked(ti, TFCP_REG_STATUS, &status);
	if (ret < 0)
		return ret;
	*degree = (status >> 8) & 0xff;

	return 0;
}

static int tfcp_tc_ta_set_output_control(struct tc_ta_classdev *ttc,
					 u32 control)
{
	struct tfcp_ta_info *ti = tfcp_ta_get_info(ttc);
	int ret;

	mutex_lock(&ti->io_lock);
	ret = tfcp_ta_i2c_write8(ti, TFCP_REG_OUTPUT_CTRL, (u8)control);
	mutex_unlock(&ti->io_lock);

	return ret;
}

static int tfcp_tc_ta_set_mode(struct tc_ta_classdev *ttc, u32 mode)
{
	struct tfcp_ta_info *ti = tfcp_ta_get_info(ttc);
	u8 val = (u8)mode;
	u8 check = 0;
	int ret, i;

	mutex_lock(&ti->io_lock);
	for (i = 0; i < TFCP_TA_RETRY_CNT; i++) {
		ret = tfcp_ta_i2c_write8(ti, TFCP_REG_MODE, val);
		if (ret < 0)
			goto retry;

		ret = tfcp_ta_read_device(ti, TFCP_REG_MODE, &check, 1);
		if (ret < 0)
			goto retry;

		if (check == val) {
			ret = 0;
			break;
		}
		ret = -EINVAL;
retry:
		usleep_range_state(1000, 2000, TASK_UNINTERRUPTIBLE);
	}
	mutex_unlock(&ti->io_lock);

	return ret;
}

static int tfcp_tc_ta_set_voltage(struct tc_ta_classdev *ttc, u32 volt)
{
	return tfcp_ta_i2c_update32(tfcp_ta_get_info(ttc), volt, 0xffff);
}

static int tfcp_tc_ta_set_current(struct tc_ta_classdev *ttc, u32 curr)
{
	return tfcp_ta_i2c_update32(tfcp_ta_get_info(ttc), curr << 16,
				    0xffff0000);
}

static int tfcp_tc_ta_set_voltage_current(struct tc_ta_classdev *ttc, u32 volt,
					  u32 curr)
{
	struct tfcp_ta_info *ti = tfcp_ta_get_info(ttc);
	u32 val = (curr << 16) | (volt & 0xffff);
	int ret;

	mutex_lock(&ti->io_lock);
	ret = tfcp_ta_i2c_write32(ti, TFCP_REG_VC, val);
	mutex_unlock(&ti->io_lock);

	return ret;
}

static int tfcp_tc_ta_authentication(struct tc_ta_classdev *ttc)
{
	struct tfcp_ta_info *ti = tfcp_ta_get_info(ttc);
	u32 status = 0;
	int ret;

	/* Handshake completes when the TA reports a valid status word */
	ret = tfcp_ta_read_locked(ti, TFCP_REG_STATUS, &status);
	if (ret < 0)
		return ret;

	return (status & 0xff) ? 0 : -EINVAL;
}

static int tfcp_tc_ta_enable_wdt(struct tc_ta_classdev *ttc, bool en)
{
	struct tfcp_ta_info *ti = tfcp_ta_get_info(ttc);
	u8 ctrl = 0;
	int ret;

	mutex_lock(&ti->io_lock);
	ret = tfcp_ta_read_device(ti, TFCP_REG_OUTPUT_CTRL, &ctrl, 1);
	if (ret == 0) {
		ctrl = en ? (ctrl | BIT(1)) : (ctrl & ~BIT(1));
		ret = tfcp_ta_i2c_write8(ti, TFCP_REG_OUTPUT_CTRL, ctrl);
	}
	mutex_unlock(&ti->io_lock);

	return ret;
}

static int tfcp_tc_ta_set_wdt(struct tc_ta_classdev *ttc, u32 ms)
{
	struct tfcp_ta_info *ti = tfcp_ta_get_info(ttc);
	int ret;

	mutex_lock(&ti->io_lock);
	ret = tfcp_ta_i2c_write32(ti, TFCP_REG_WDT, ms);
	mutex_unlock(&ti->io_lock);

	return ret;
}

static int tfcp_tc_ta_get_power_limit(struct tc_ta_classdev *ttc, u32 *power)
{
	return tfcp_ta_read_locked(tfcp_ta_get_info(ttc), TFCP_REG_POWER_LIMIT,
				   power);
}

static int tfcp_tc_ta_get_max_power_duration(struct tc_ta_classdev *ttc,
					     u32 *time)
{
	return tfcp_ta_read_locked(tfcp_ta_get_info(ttc),
				   TFCP_REG_POWER_DURATION, time);
}

static int tfcp_tc_ta_get_output_control_support(struct tc_ta_classdev *ttc,
						bool *support)
{
	struct tfcp_ta_info *ti = tfcp_ta_get_info(ttc);
	u32 status = 0;
	int ret;

	ret = tfcp_ta_read_locked(ti, TFCP_REG_STATUS, &status);
	if (ret < 0)
		return ret;
	/* Bit 0 of the status word advertises output-control support */
	*support = !!(status & BIT(0));

	return 0;
}

static int tfcp_tc_ta_suspend(struct tc_ta_classdev *ttc)
{
	struct tfcp_ta_info *ti = tfcp_ta_get_info(ttc);

	mutex_lock(&ti->suspend_lock);
	ti->suspended = true;
	mutex_unlock(&ti->suspend_lock);

	return 0;
}

static int tfcp_tc_ta_resume(struct tc_ta_classdev *ttc)
{
	struct tfcp_ta_info *ti = tfcp_ta_get_info(ttc);

	mutex_lock(&ti->suspend_lock);
	ti->suspended = false;
	mutex_unlock(&ti->suspend_lock);

	return 0;
}

static const struct tc_ta_device_ops tfcp_tc_ta_ops = {
	.get_verinfo = tfcp_tc_ta_get_verinfo,
	.get_fwcode = tfcp_tc_ta_get_fwcode,
	.get_mfinfo = tfcp_tc_ta_get_mfinfo,
	.get_datecode = tfcp_tc_ta_get_datecode,
	.get_min_voltage = tfcp_tc_ta_get_min_voltage,
	.get_max_voltage = tfcp_tc_ta_get_max_voltage,
	.get_min_current = tfcp_tc_ta_get_min_current,
	.get_max_current = tfcp_tc_ta_get_max_current,
	.get_output_voltage = tfcp_tc_ta_get_output_voltage,
	.get_output_current = tfcp_tc_ta_get_output_current,
	.get_status = tfcp_tc_ta_get_status,
	.get_temp1 = tfcp_tc_ta_get_temp1,
	.get_temp2 = tfcp_tc_ta_get_temp2,
	.set_output_control = tfcp_tc_ta_set_output_control,
	.set_mode = tfcp_tc_ta_set_mode,
	.set_voltage = tfcp_tc_ta_set_voltage,
	.set_current = tfcp_tc_ta_set_current,
	.suspend = tfcp_tc_ta_suspend,
	.resume = tfcp_tc_ta_resume,
	.set_voltage_current = tfcp_tc_ta_set_voltage_current,
	.get_output_voltage_current = tfcp_tc_ta_get_output_voltage_current,
	.authentication = tfcp_tc_ta_authentication,
	.enable_wdt = tfcp_tc_ta_enable_wdt,
	.set_wdt = tfcp_tc_ta_set_wdt,
	.get_power_limit = tfcp_tc_ta_get_power_limit,
	.get_max_power_duration = tfcp_tc_ta_get_max_power_duration,
	.get_output_control_support = tfcp_tc_ta_get_output_control_support,
};

static void tfcp_ta_pinctrl_init(struct tfcp_ta_info *info)
{
	info->pinctrl = devm_pinctrl_get(info->dev);
	if (IS_ERR_OR_NULL(info->pinctrl)) {
		dev_err(info->dev, "Cannot find pinctrl\n");
		return;
	}

	info->data_gpio = pinctrl_lookup_state(info->pinctrl,
					       "tfcp_i2c_sda_mode");
	if (IS_ERR_OR_NULL(info->data_gpio))
		dev_err(info->dev, "Cannot find pinctrl_i2c_data_gpio\n");
	else
		pinctrl_select_state(info->pinctrl, info->data_gpio);

	info->sclk_mode = pinctrl_lookup_state(info->pinctrl,
					       "tfcp_i2c_sclk_mode");
	if (IS_ERR_OR_NULL(info->sclk_mode))
		dev_err(info->dev, "Cannot find pinctrl_i2c_clk_gpio\n");
	else
		pinctrl_select_state(info->pinctrl, info->sclk_mode);
}

static int tfcp_ta_i2c_probe(struct i2c_client *client)
{
	struct tfcp_ta_info *info;
	int ret;

	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = &client->dev;
	info->client = client;
	i2c_set_clientdata(client, info);

	mutex_init(&info->io_lock);
	mutex_init(&info->suspend_lock);
	crc8_populate_msb(info->crc8_table, TFCP_TA_CRC8_POLY);

	tfcp_ta_pinctrl_init(info);

	info->ta_dev.name = "tfcp_ta";
	info->ta_dev.dev = &client->dev;
	info->ta_dev.ops = &tfcp_tc_ta_ops;

	ret = devm_tc_ta_classdev_register(&client->dev, &info->ta_dev);
	if (ret < 0) {
		dev_err(&client->dev, "failed to register tfcp_ta\n");
		return ret;
	}

	return 0;
}

static void tfcp_ta_i2c_remove(struct i2c_client *client)
{
}

static void tfcp_ta_i2c_shutdown(struct i2c_client *client)
{
}

static const struct of_device_id tfcp_ta_of_match[] = {
	{ .compatible = "tc,tfcp_ta" },
	{},
};
MODULE_DEVICE_TABLE(of, tfcp_ta_of_match);

static const struct i2c_device_id tfcp_ta_i2c_id[] = {
	{ "tfcp_ta", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, tfcp_ta_i2c_id);

static struct i2c_driver tfcp_ta_i2c_driver = {
	.driver = {
		.name = "tfcp_ta",
		.of_match_table = tfcp_ta_of_match,
	},
	.probe = tfcp_ta_i2c_probe,
	.remove = tfcp_ta_i2c_remove,
	.shutdown = tfcp_ta_i2c_shutdown,
	.id_table = tfcp_ta_i2c_id,
};

module_i2c_driver(tfcp_ta_i2c_driver);

MODULE_DESCRIPTION("Transsion TFCP Fast Charge TA Driver");
MODULE_LICENSE("GPL");
