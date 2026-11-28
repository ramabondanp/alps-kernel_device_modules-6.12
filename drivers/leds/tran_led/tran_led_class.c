// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 Transsion Inc.
 *
 * Transsion LED class dispatch layer.
 *
 * Reconstructed from the X6871 firmware tran-led-core.ko (PLAN.md 6.2q).
 * The firmware keeps two class files linked into one module:
 *
 *   tran_led_class.c  -- these 12 accessors plus register/unregister/lookup
 *   tran_led_core.c   -- the platform probe, sysfs, and the char device
 *
 * Every accessor is the same shape in the disassembly: NULL-check the device,
 * NULL-check dev->ops (+0x08), NULL-check the slot, then tail-call it. When
 * the chain fails they return -EOPNOTSUPP (-95), which shows up as
 * `mov w0, #-0x5f` at the end of each function.
 */
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>

#include "tran_led_class.h"

static struct tran_led_device *ptlc_led;
struct class *tlc_led_class;
static int tlc_log_level = TLC_DEBUG_LEVEL;

int tlc_get_debug_level(void)
{
	return tlc_log_level;
}
EXPORT_SYMBOL(tlc_get_debug_level);

#define TLC_FORWARD(_slot, _args, _call)				\
do {									\
	if (!dev || !dev->ops || !dev->ops->_slot)			\
		return -EOPNOTSUPP;					\
	return dev->ops->_slot _call;					\
} while (0)

int tran_led_dev_set_i2c_retries(struct tran_led_device *dev, int retries)
{
	TLC_FORWARD(set_i2c_retries, , (dev, retries));
}
EXPORT_SYMBOL(tran_led_dev_set_i2c_retries);

int tran_led_dev_set_i2c_timeout(struct tran_led_device *dev, int ms)
{
	TLC_FORWARD(set_i2c_timeout, , (dev, ms));
}
EXPORT_SYMBOL(tran_led_dev_set_i2c_timeout);

int tran_led_dev_rw_i2c_data(struct tran_led_device *dev, unsigned long arg)
{
	TLC_FORWARD(rw_i2c_data, , (dev, arg));
}
EXPORT_SYMBOL(tran_led_dev_rw_i2c_data);

int tran_led_dev_get_chip_type(struct tran_led_device *dev, int *type)
{
	TLC_FORWARD(get_chip_type, , (dev, type));
}
EXPORT_SYMBOL(tran_led_dev_get_chip_type);

int tran_led_dev_get_cmd(struct tran_led_device *dev, char *buf, int len)
{
	TLC_FORWARD(get_cmd, , (dev, buf, len));
}
EXPORT_SYMBOL(tran_led_dev_get_cmd);

/*
 * set_cmd is the only accessor with an extra branch: after the slot returns,
 * the firmware compares the result against 1 and falls through to the debug
 * log when the slot is missing entirely (see the `bl tlc_get_debug_level`
 * at the -EOPNOTSUPP path). Keep the same shape.
 */
int tran_led_dev_set_cmd(struct tran_led_device *dev, const char *buf, int len)
{
	if (!dev || !dev->ops || !dev->ops->set_cmd) {
		if (tlc_get_debug_level() >= TLC_INFO_LEVEL)
			pr_notice("[tlc]tran_led_dev_set_cmd failed vendor dev = null!\n");
		return -EOPNOTSUPP;
	}

	return dev->ops->set_cmd(dev, buf, len);
}
EXPORT_SYMBOL(tran_led_dev_set_cmd);

int tran_led_dev_store_tran_led_cmd(struct tran_led_device *dev, const char *cmd)
{
	if (!dev || !dev->ops || !dev->ops->store_tran_led_cmd)
		return -EOPNOTSUPP;

	return dev->ops->store_tran_led_cmd(dev, cmd);
}
EXPORT_SYMBOL(tran_led_dev_store_tran_led_cmd);

int tran_led_dev_show_tran_led_cmd(struct tran_led_device *dev, char *buf)
{
	if (!dev || !dev->ops || !dev->ops->show_tran_led_cmd)
		return -EOPNOTSUPP;

	return dev->ops->show_tran_led_cmd(dev, buf);
}
EXPORT_SYMBOL(tran_led_dev_show_tran_led_cmd);

int tran_led_dev_store_tran_led_check(struct tran_led_device *dev, const char *cmd)
{
	if (!dev || !dev->ops || !dev->ops->store_tran_led_check)
		return -EOPNOTSUPP;

	return dev->ops->store_tran_led_check(dev, cmd);
}
EXPORT_SYMBOL(tran_led_dev_store_tran_led_check);

int tran_led_dev_show_tran_led_check(struct tran_led_device *dev, char *buf)
{
	if (!dev || !dev->ops || !dev->ops->show_tran_led_check)
		return -EOPNOTSUPP;

	return dev->ops->show_tran_led_check(dev, buf);
}
EXPORT_SYMBOL(tran_led_dev_show_tran_led_check);

int tran_led_dev_store_tran_multicolor(struct tran_led_device *dev,
				       const char *cmd)
{
	if (!dev || !dev->ops || !dev->ops->store_tran_multicolor)
		return -EOPNOTSUPP;

	return dev->ops->store_tran_multicolor(dev, cmd);
}
EXPORT_SYMBOL(tran_led_dev_store_tran_multicolor);

int tran_led_dev_show_tran_multicolor(struct tran_led_device *dev, char *buf)
{
	if (!dev || !dev->ops || !dev->ops->show_tran_multicolor)
		return -EOPNOTSUPP;

	return dev->ops->show_tran_multicolor(dev, buf);
}
EXPORT_SYMBOL(tran_led_dev_show_tran_multicolor);

static void tran_led_device_release(struct device *dev)
{
	kfree(to_tran_led_device(dev));
}

/*
 * register/unregister.
 *
 * The firmware allocates 0x778 bytes (1912) from the 3520-byte kmalloc cache,
 * sets dev_set_name(parent=NULL, name) on the embedded device at +0x220,
 * copies the optional properties into +0x00, calls device_register(), and only
 * then publishes dev->ops at +0x08. The order matters: ops is visible to
 * sysfs only after the device exists.
 */
struct tran_led_device *tran_led_device_register(const char *name,
						 struct device *parent,
						 void *devdata,
						 const struct tran_led_ops *ops,
						 const struct tran_led_properties *props)
{
	struct tran_led_device *led_dev;
	int rc;

	led_dev = kzalloc(sizeof(*led_dev), GFP_KERNEL);
	if (!led_dev)
		return ERR_PTR(-ENOMEM);

	/*
	 * Order matters and matches the firmware: parent (+0x280), the class
	 * (+0x538, so device_register can attach to it), the name, the driver
	 * data (+0x2b8) and the properties (+0x00), all before
	 * device_register(). dev->ops (+0x08) is published only afterwards.
	 */
	led_dev->dev.parent = parent;
	led_dev->dev.class = tlc_led_class;
	led_dev->dev.release = tran_led_device_release;
	dev_set_name(&led_dev->dev, "%s", name);
	dev_set_drvdata(&led_dev->dev, devdata);

	/*
	 * struct tran_led_properties is a single pointer. The firmware does
	 * not import memcpy at all, so this is a plain assignment rather
	 * than a byte copy.
	 */
	if (props)
		led_dev->props = *props;

	rc = device_register(&led_dev->dev);
	if (rc) {
		put_device(&led_dev->dev);
		return ERR_PTR(rc);
	}

	led_dev->ops = ops;
	ptlc_led = led_dev;

	return led_dev;
}
EXPORT_SYMBOL(tran_led_device_register);

void tran_led_device_unregister(struct tran_led_device *led_dev)
{
	if (!led_dev)
		return;

	if (ptlc_led == led_dev)
		ptlc_led = NULL;

	led_dev->ops = NULL;
	device_unregister(&led_dev->dev);
}
EXPORT_SYMBOL(tran_led_device_unregister);

/*
 * Lookup by name.
 *
 * The firmware walks the class and then does `sub x8, x0, #0x220` before
 * returning, i.e. class_find_device() gives back a struct device * and the
 * result is converted back to the enclosing tran_led_device. That confirms
 * the +0x220 embedding used throughout this file.
 */
static int tranled_match_device_by_name(struct device *dev, const void *data)
{
	const char *name = data;

	return dev_name(dev) && !strcmp(dev_name(dev), name);
}

struct tran_led_device *get_leddev_by_name(const char *name)
{
	struct device *dev;

	if (!name || !tlc_led_class)
		return NULL;

	dev = class_find_device(tlc_led_class, NULL, name,
				tranled_match_device_by_name);
	if (!dev)
		return NULL;

	return to_tran_led_device(dev);
}
EXPORT_SYMBOL(get_leddev_by_name);

MODULE_AUTHOR("Transsion Inc.");
MODULE_LICENSE("GPL");
