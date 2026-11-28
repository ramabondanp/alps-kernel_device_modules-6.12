// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 Transsion Inc.
 *
 * Transsion LED core: platform probe, sysfs attributes and the char device.
 *
 * Reconstructed from the X6871 firmware tran-led-core.ko (PLAN.md 6.2q).
 *
 * What the disassembly fixes, and why each piece is not a guess:
 *
 *  - probe allocates 0x178 (376) bytes via devm_kmalloc, zeroes
 *    support_pdlc at +0x128, reads the "support_pdlc" DT property with
 *    of_find_property and stores the boolean at the same offset.
 *  - it then creates the "led" sysfs group on the parent device, a "led"
 *    kobject with a symlink, an optional "pdlc" kobject + symlink (only when
 *    the DT property is present), a 65536-byte led_buff via
 *    __kmalloc_large_noprof, alloc_chrdev_region(..., 0, 2, "tran_led"),
 *    two cdevs (command at +0x08, buffer at +0x90), two device_create()
 *    nodes numbered N and N+1, and finally tran_led_class_init().
 *  - the sysfs handlers all load `tlc->primary_led` (+0x130) for
 *    tran_led_cmd/tran_led_check/... and `tlc->second_led` (+0x138) for
 *    tran_pdlc_cmd, and the multicolor handler consults both.
 *  - remove mirrors probe: device_destroy twice, cdev_del twice,
 *    unregister_chrdev_region(..., 2), tran_led_class_destroy, kfree(led_buff).
 *
 * The ioctl command numbers are literal immediates in the switch:
 *   0x40041202 set_i2c_timeout   0x40041203 rw_i2c_data
 *   0x40041204 chip_type         0x40041205 set_i2c_retries
 *   0x40041206 user command      0x40041207 buffer
 *   0x40041208 something+16      0x40041209 buffer size
 *   0x40101207 / 0x40101208      0x40201202 / 0x40201207
 *   0x80011200 get_cmd           0x80201201 get_cmd
 */

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>

#include "tran_led_class.h"

#define TLC_CMD_BUFFER_SIZE	0x10000
#define TLC_CMD_MAX_SIZE	0x20
#define TLC_DEVICE_NAME		"tran_led"
#define TLC_DEV_NUM		2
#define TLC_LOG_LEVEL_DEFAULT	TLC_DEBUG_LEVEL

/*
 * ioctl commands, recovered as literal immediates from the firmware switch.
 * The type byte is 0x12, not 'L':
 *   0x80011200 get_chip_type   0x80201201 get_cmd
 *   0x40201202 set_cmd         0x40041203 rw_i2c_data
 *   0x40041204 set_i2c_retries 0x40041205 set_i2c_timeout
 */
#define TRAN_LED_IOC_MAGIC	0x12
#define TRAN_GET_CHIP_TYPE	_IOR(TRAN_LED_IOC_MAGIC, 0x00, char)
#define TRAN_GET_CMD		_IOR(TRAN_LED_IOC_MAGIC, 0x01, \
				     char[TLC_CMD_MAX_SIZE])
#define TRAN_SET_CMD		_IOW(TRAN_LED_IOC_MAGIC, 0x02, \
				     char[TLC_CMD_MAX_SIZE])
#define TRAN_RW_I2C_DATA	_IOW(TRAN_LED_IOC_MAGIC, 0x03, int)
#define TRAN_SET_I2C_RETRIES	_IOW(TRAN_LED_IOC_MAGIC, 0x04, int)
#define TRAN_SET_I2C_TIMEOUT	_IOW(TRAN_LED_IOC_MAGIC, 0x05, int)

/*
 * Offsets are load-bearing: the firmware's probe and ioctl use them directly
 * and the module is only 0x178 bytes (devm_kmalloc argument). They are
 * asserted below.
 */
struct tran_led_core {
	struct platform_device *pdev;		/* +0x000 */
	struct cdev cmd_cdev;			/* +0x008 */
	struct cdev buff_cdev;			/* +0x090 */
	dev_t devno;				/* +0x118 */
	struct class *class;			/* +0x120 */
	bool support_pdlc;			/* +0x128 */
	u8 __pad0[3];				/* +0x129 */
	int chip_type;				/* +0x12c */
	struct tran_led_device *primary_led;	/* +0x130 */
	struct tran_led_device *second_led;	/* +0x138 */
	struct mutex tran_led_mutex;		/* +0x140 */
	u8 __pad1[0x170 - 0x140 - sizeof(struct mutex)];
	int log_level;				/* +0x170 */
	u8 __tail[0x178 - 0x174];
};

_Static_assert(offsetof(struct tran_led_core, class) == 0x120,
	       "class must stay at +0x120");
_Static_assert(offsetof(struct tran_led_core, support_pdlc) == 0x128,
	       "support_pdlc must stay at +0x128");
_Static_assert(offsetof(struct tran_led_core, chip_type) == 0x12c,
	       "chip_type must stay at +0x12c");
_Static_assert(offsetof(struct tran_led_core, primary_led) == 0x130,
	       "primary_led must stay at +0x130");
_Static_assert(offsetof(struct tran_led_core, second_led) == 0x138,
	       "second_led must stay at +0x138");
_Static_assert(offsetof(struct tran_led_core, tran_led_mutex) == 0x140,
	       "tran_led_mutex must stay at +0x140");
_Static_assert(offsetof(struct tran_led_core, log_level) == 0x170,
	       "log_level must stay at +0x170");
_Static_assert(sizeof(struct tran_led_core) == 0x178,
	       "sizeof(struct tran_led_core) must be 0x178");

extern struct class *tlc_led_class;

static struct tran_led_core *ptlc;
static char *led_buff;
static struct kobject *tlc_led_kobj;
static struct kobject *tlc_pdlc_kobj;

static int tran_led_dev_check(struct tran_led_core *tlc);

static ssize_t show_tran_led_log_level(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct tran_led_core *tlc = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", tlc->log_level);
}

static ssize_t store_tran_led_log_level(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct tran_led_core *tlc = dev_get_drvdata(dev);
	int val = 0;

	if (kstrtoint(buf, 10, &val) < 0) {
		tlc_err("%s: format error!\n", __func__);
		goto log;
	}

	tlc->log_level = val;
	log:
	tlc_info("%s: log_level=%d\n", __func__, tlc->log_level);

	return size;
}
static DEVICE_ATTR(tran_led_log_level, 0664, show_tran_led_log_level,
		   store_tran_led_log_level);

static ssize_t show_tran_led_cmd(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct tran_led_core *tlc = dev_get_drvdata(dev);
	char pbuf = 0;
	int ret;

	ret = tran_led_dev_check(tlc);
	if (ret < 0)
		return ret;

	tran_led_dev_show_tran_led_cmd(tlc->primary_led, &pbuf);

	return sprintf(buf, "%c\n", pbuf);
}

static ssize_t store_tran_led_cmd(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	struct tran_led_core *tlc = dev_get_drvdata(dev);

	if (!size)
		return 0;

	if (!tlc->primary_led || !tlc->second_led)
		tran_led_dev_check(tlc);

	if (tlc->primary_led)
		tran_led_dev_store_tran_led_cmd(tlc->primary_led, buf);
	if (tlc->second_led)
		tran_led_dev_store_tran_led_cmd(tlc->second_led, buf);

	return size;
}
static DEVICE_ATTR(tran_led_cmd, 0664, show_tran_led_cmd, store_tran_led_cmd);

static ssize_t show_tran_led_check(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct tran_led_core *tlc = dev_get_drvdata(dev);
	int ret;

	ret = tran_led_dev_check(tlc);
	if (ret < 0)
		return ret;

	return tran_led_dev_show_tran_led_check(tlc->primary_led, buf);
}

static ssize_t store_tran_led_check(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{
	struct tran_led_core *tlc = dev_get_drvdata(dev);
	int ret;

	if (!size)
		return 0;

	ret = tran_led_dev_check(tlc);
	if (ret < 0)
		return ret;

	tran_led_dev_store_tran_led_check(tlc->primary_led, buf);

	return size;
}
static DEVICE_ATTR(tran_led_check, 0664, show_tran_led_check,
		   store_tran_led_check);

static ssize_t show_tran_pdlc_cmd(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct tran_led_core *tlc = dev_get_drvdata(dev);
	char pbuf = 0;
	int ret;

	ret = tran_led_dev_check(tlc);
	if (ret < 0)
		return ret;

	tran_led_dev_show_tran_led_cmd(tlc->second_led, &pbuf);

	return sprintf(buf, "%c\n", pbuf);
}

static ssize_t store_tran_pdlc_cmd(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct tran_led_core *tlc = dev_get_drvdata(dev);
	int ret;

	if (!size)
		return 0;

	ret = tran_led_dev_check(tlc);
	if (ret < 0)
		return ret;

	tran_led_dev_store_tran_led_cmd(tlc->second_led, buf);

	return size;
}
static DEVICE_ATTR(tran_pdlc_cmd, 0664, show_tran_pdlc_cmd, store_tran_pdlc_cmd);

/*
 * multicolor is the only handler that talks to both LEDs: it walks primary
 * then second, appends to the same buffer, and falls back to "none" when
 * neither produced anything (the firmware builds the 4-byte "none\n" with
 * `mov w9, #0x6f6e / movk w9, #0x656e`, i.e. 'n''o''n''e').
 */
static ssize_t show_tran_multicolor(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct tran_led_core *tlc = dev_get_drvdata(dev);
	size_t len = 0;
	int ret;

	ret = tran_led_dev_check(tlc);
	if (ret < 0)
		return ret;

	if (tlc->primary_led) {
		ret = tran_led_dev_show_tran_multicolor(tlc->primary_led, buf);
		if (ret > 0)
			len = ret;
	}

	if (tlc->second_led) {
		ret = tran_led_dev_show_tran_multicolor(tlc->second_led,
							buf + len);
		if (ret > 0)
			len += ret;
	}

	if (!len)
		return sprintf(buf, "none\n");

	return len;
}

static ssize_t store_tran_multicolor(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct tran_led_core *tlc = dev_get_drvdata(dev);
	int ret;

	if (!size)
		return 0;

	ret = tran_led_dev_check(tlc);
	if (ret < 0)
		return ret;

	if (!tlc->primary_led && !tlc->second_led)
		return -ENODEV;

	ret = tran_led_dev_store_tran_multicolor(tlc->primary_led ?
						 tlc->primary_led :
						 tlc->second_led, buf);

	return ret < 0 ? ret : size;
}
static DEVICE_ATTR(tran_multicolor, 0664, show_tran_multicolor,
		   store_tran_multicolor);

static struct attribute *tran_led_attrs[] = {
	&dev_attr_tran_led_log_level.attr,
	&dev_attr_tran_led_cmd.attr,
	&dev_attr_tran_led_check.attr,
	&dev_attr_tran_pdlc_cmd.attr,
	&dev_attr_tran_multicolor.attr,
	NULL,
};

static const struct attribute_group tran_led_group = {
	.attrs = tran_led_attrs,
};

/*
 * Resolve the two LED devices the sysfs handlers operate on. The firmware
 * looks them up by the fixed names "primary_led" and "second_led" and caches
 * them at +0x130 and +0x138, returning -ENODEV when the primary is missing.
 */
static int tran_led_dev_check(struct tran_led_core *tlc)
{
	if (tlc->primary_led && !IS_ERR(tlc->primary_led))
		return 0;

	tlc->primary_led = get_leddev_by_name("primary_led");
	if (!tlc->primary_led) {
		tlc_err("%s: leddev is invalid\n", __func__);
		return -ENODEV;
	}

	tlc->second_led = get_leddev_by_name("second_led");
	if (!tlc->second_led)
		tlc_info("%s: leddev1 is invalid\n", __func__);

	return 0;
}

int tran_led_class_init(struct tran_led_core *tlc)
{
	struct class *class;

	class = class_create(TLC_DEVICE_NAME);
	if (IS_ERR(class))
		return PTR_ERR(class);

	tlc->class = class;
	tlc_led_class = class;

	return 0;
}

void tran_led_class_destroy(struct tran_led_core *tlc)
{
	if (!tlc->class || IS_ERR(tlc->class))
		return;

	class_destroy(tlc->class);
	tlc->class = NULL;
	tlc_led_class = NULL;
}

static int tran_led_open(struct inode *inode, struct file *file)
{
	struct tran_led_core *tlc = container_of(inode->i_cdev,
						 struct tran_led_core, cmd_cdev);

	if (tlc && !IS_ERR(tlc))
		file->private_data = tlc;

	tlc_info("%s: open\n", __func__);

	return 0;
}

static int tran_led_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;

	return 0;
}

/*
 * The ioctl surface is what the userspace LED HAL drives. Commands are
 * matched in the same order as the firmware switch; anything unrecognised
 * returns -ENOTTY, and out-of-range arguments return -EINVAL.
 */
static long tran_led_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct tran_led_core *tlc = file->private_data;
	char kbuf[TLC_CMD_MAX_SIZE] = { 0 };
	int ret = 0, val = 0;
	void __user *uarg = (void __user *)arg;

	if (!tlc)
		return -ENODEV;

	ret = tran_led_dev_check(tlc);
	if (ret < 0)
		return ret;

	mutex_lock(&tlc->tran_led_mutex);

	switch (cmd) {
	case TRAN_GET_CMD:
		ret = tran_led_dev_get_cmd(tlc->primary_led, kbuf,
					   sizeof(kbuf));
		if (ret >= 0 && copy_to_user(uarg, kbuf, sizeof(kbuf)))
			ret = -EFAULT;
		break;
	case TRAN_GET_CHIP_TYPE:
		ret = tran_led_dev_get_chip_type(tlc->primary_led, &val);
		if (ret < 0)
			break;
		tlc->chip_type = val;
		if (copy_to_user(uarg, &val, sizeof(val)))
			ret = -EFAULT;
		break;
	case TRAN_SET_I2C_TIMEOUT:
		if ((long)arg < 0) {
			ret = -EINVAL;
			break;
		}
		ret = tran_led_dev_set_i2c_timeout(tlc->primary_led, arg);
		break;
	case TRAN_SET_I2C_RETRIES:
		ret = tran_led_dev_set_i2c_retries(tlc->primary_led, arg);
		break;
	case TRAN_RW_I2C_DATA:
		ret = tran_led_dev_rw_i2c_data(tlc->primary_led, arg);
		break;
	case TRAN_SET_CMD:
		if (copy_from_user(kbuf, uarg, sizeof(kbuf))) {
			ret = -EFAULT;
			break;
		}
		ret = tran_led_dev_set_cmd(tlc->primary_led, kbuf,
					   sizeof(kbuf));
		break;
	default:
		tlc_info("ioctl, cmd=0x%02x, arg=0x%02lx\n", cmd, arg);
		ret = -ENOTTY;
		break;
	}

	mutex_unlock(&tlc->tran_led_mutex);

	return ret;
}

static const struct file_operations tran_led_fops = {
	.owner = THIS_MODULE,
	.open = tran_led_open,
	.release = tran_led_release,
	.unlocked_ioctl = tran_led_ioctl,
	.compat_ioctl = tran_led_ioctl,
};

static int tran_buff_open(struct inode *inode, struct file *file)
{
	tlc_dbg("%s\n", __func__);

	return 0;
}

static int tran_buff_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long size = vma->vm_end - vma->vm_start;
	unsigned long pfn;

	if (!led_buff || size > TLC_CMD_BUFFER_SIZE) {
		tlc_err("%s: size is invalid!\n", __func__);
		return -EINVAL;
	}

	pfn = virt_to_phys(led_buff) >> PAGE_SHIFT;

	if (remap_pfn_range(vma, vma->vm_start, pfn, size,
			    vma->vm_page_prot)) {
		tlc_err("%s: remap failed!\n", __func__);
		return -EAGAIN;
	}

	return 0;
}

static const struct file_operations tran_buff_fops = {
	.owner = THIS_MODULE,
	.open = tran_buff_open,
	.mmap = tran_buff_mmap,
};

static int tran_led_core_probe(struct platform_device *pdev)
{
	struct tran_led_core *tlc;
	struct kobject *led_kobj = NULL, *pdlc_kobj = NULL;
	struct device *cdev_dev;
	int ret;

	tlc_info("%s: starts\n", __func__);

	tlc = devm_kzalloc(&pdev->dev, sizeof(*tlc), GFP_KERNEL);
	if (!tlc)
		return -ENOMEM;

	tlc->pdev = pdev;
	tlc->log_level = TLC_LOG_LEVEL_DEFAULT;
	mutex_init(&tlc->tran_led_mutex);
	platform_set_drvdata(pdev, tlc);
	ptlc = tlc;

	tlc->support_pdlc = of_find_property(pdev->dev.of_node,
					     "support_pdlc", NULL) != NULL;

	ret = sysfs_create_group(&pdev->dev.kobj, &tran_led_group);
	if (ret < 0) {
		tlc_err("%s : sysfs_create_group failed ret=%d\n", __func__, ret);
		goto err_ptlc;
	}

	led_kobj = kobject_create_and_add("led", NULL);
	if (!led_kobj) {
		tlc_err("%s:sysfs_create_group fail\n", __func__);
		ret = -ENOMEM;
		goto err_group;
	}

	ret = sysfs_create_link(led_kobj, &pdev->dev.kobj, "led");
	if (ret < 0) {
		tlc_err("%s : sysfs_create_link failed ret=%d\n", __func__, ret);
		goto err_led_kobj;
	}

	if (tlc->support_pdlc) {
		pdlc_kobj = kobject_create_and_add("pdlc", NULL);
		if (!pdlc_kobj) {
			tlc_err("%s:sysfs_create_group fail\n", __func__);
			ret = -ENOMEM;
			goto err_led_link;
		}

		ret = sysfs_create_link(pdlc_kobj, &pdev->dev.kobj, "pdlc");
		if (ret < 0) {
			tlc_err("%s : sysfs_create_link failed ret=%d\n",
				__func__, ret);
			goto err_pdlc_kobj;
		}
	}

	led_buff = kmalloc(TLC_CMD_BUFFER_SIZE, GFP_KERNEL);
	if (!led_buff) {
		tlc_err("%s: malloc fail!\n", __func__);
		ret = -ENOMEM;
		goto err_pdlc_link;
	}
	memset(led_buff, 0, TLC_CMD_BUFFER_SIZE);

	ret = alloc_chrdev_region(&tlc->devno, 0, TLC_DEV_NUM, TLC_DEVICE_NAME);
	if (ret < 0) {
		tlc_err("Failed to allocate char device region\n");
		goto err_buff;
	}

	cdev_init(&tlc->cmd_cdev, &tran_led_fops);
	tlc->cmd_cdev.owner = THIS_MODULE;

	ret = cdev_add(&tlc->cmd_cdev, tlc->devno, 1);
	if (ret < 0) {
		tlc_err("Failed to add cdev\n");
		goto err_region;
	}

	cdev_init(&tlc->buff_cdev, &tran_buff_fops);
	tlc->buff_cdev.owner = THIS_MODULE;

	ret = cdev_add(&tlc->buff_cdev, tlc->devno + 1, 1);
	if (ret < 0) {
		tlc_err("Failed to add mem cdev\n");
		goto err_cmd_cdev;
	}

	ret = tran_led_class_init(tlc);
	if (ret < 0) {
		tlc_err("Failed to add led class\n");
		goto err_buff_cdev;
	}

	cdev_dev = device_create(tlc->class, NULL, tlc->devno, NULL,
				 TLC_DEVICE_NAME);
	if (IS_ERR(cdev_dev)) {
		ret = PTR_ERR(cdev_dev);
		tlc_err("Failed to create device\n");
		goto err_class;
	}

	cdev_dev = device_create(tlc->class, NULL, tlc->devno + 1, NULL,
				 "tran_led_buff");
	if (IS_ERR(cdev_dev)) {
		ret = PTR_ERR(cdev_dev);
		tlc_err("Failed to create device\n");
		goto err_device;
	}

	tlc_led_kobj = led_kobj;
	tlc_pdlc_kobj = pdlc_kobj;

	tlc_info("%s: done\n", __func__);

	return 0;

err_device:
	device_destroy(tlc->class, tlc->devno);
err_class:
	tran_led_class_destroy(tlc);
err_buff_cdev:
	cdev_del(&tlc->buff_cdev);
err_cmd_cdev:
	cdev_del(&tlc->cmd_cdev);
err_region:
	unregister_chrdev_region(tlc->devno, TLC_DEV_NUM);
err_buff:
	kfree(led_buff);
	led_buff = NULL;
err_pdlc_link:
	if (pdlc_kobj)
		sysfs_remove_link(pdlc_kobj, "pdlc");
err_pdlc_kobj:
	if (pdlc_kobj)
		kobject_put(pdlc_kobj);
err_led_link:
	sysfs_remove_link(led_kobj, "led");
err_led_kobj:
	kobject_put(led_kobj);
err_group:
	sysfs_remove_group(&pdev->dev.kobj, &tran_led_group);
err_ptlc:
	ptlc = NULL;

	return ret;
}

static void tran_led_core_teardown(struct platform_device *pdev)
{
	struct tran_led_core *tlc = platform_get_drvdata(pdev);

	if (!tlc)
		return;

	device_destroy(tlc->class, tlc->devno);
	device_destroy(tlc->class, tlc->devno + 1);
	cdev_del(&tlc->cmd_cdev);
	cdev_del(&tlc->buff_cdev);
	unregister_chrdev_region(tlc->devno, TLC_DEV_NUM);
	tran_led_class_destroy(tlc);

	kfree(led_buff);
	led_buff = NULL;

	if (tlc_pdlc_kobj) {
		sysfs_remove_link(tlc_pdlc_kobj, "pdlc");
		kobject_put(tlc_pdlc_kobj);
		tlc_pdlc_kobj = NULL;
	}
	if (tlc_led_kobj) {
		sysfs_remove_link(tlc_led_kobj, "led");
		kobject_put(tlc_led_kobj);
		tlc_led_kobj = NULL;
	}
	sysfs_remove_group(&pdev->dev.kobj, &tran_led_group);
	platform_set_drvdata(pdev, NULL);
	ptlc = NULL;
}

static void tran_led_core_remove(struct platform_device *pdev)
{
	tran_led_core_teardown(pdev);
}

static void tran_led_core_shutdown(struct platform_device *pdev)
{
	tran_led_core_teardown(pdev);
}

static const struct of_device_id tran_led_core_of_match[] = {
	{ .compatible = "tran,led_core" },
	{},
};
MODULE_DEVICE_TABLE(of, tran_led_core_of_match);

static struct platform_driver tran_led_core_driver = {
	.probe = tran_led_core_probe,
	.remove = tran_led_core_remove,
	.shutdown = tran_led_core_shutdown,
	.driver = {
		.name = "tran_led_core",
		.of_match_table = tran_led_core_of_match,
	},
};
module_platform_driver(tran_led_core_driver);

MODULE_DESCRIPTION("Tran Led Core Driver");
MODULE_LICENSE("GPL");
