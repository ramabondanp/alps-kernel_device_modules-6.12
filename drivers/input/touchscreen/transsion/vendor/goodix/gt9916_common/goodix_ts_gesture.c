// SPDX-License-Identifier: GPL-2.0
/*
 * Goodix GT9916 Gesture & FOD Module for Transsion X6871
 */
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/atomic.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include "goodix_ts_core.h"

extern void tran_gesture_set_code(u8 code);

#define GESTURE_DOUBLE_TAP_BIT		BIT(0)
#define GESTURE_SINGLE_TAP_BIT		BIT(1)
#define GESTURE_FOD_PRESS_BIT		BIT(2)

struct gesture_module {
	atomic_t registered;
	bool irq_wake_enabled;
	bool gesture_suspended;
	u8 gesture_type;
	struct goodix_ts_core *ts_core;
	struct goodix_ext_module module;
	struct proc_dir_entry *fp_proc;
	int fp_status;
};

static struct gesture_module *gsx_gesture;
static bool module_initialized;
int tran_fingerprint_status;
int g_fp_flag;

void goodix_gesture_enable(int enable)
{
	if (!gsx_gesture)
		return;
	if (enable)
		gsx_gesture->gesture_type |= (GESTURE_DOUBLE_TAP_BIT | GESTURE_SINGLE_TAP_BIT);
	else
		gsx_gesture->gesture_type &= ~(GESTURE_DOUBLE_TAP_BIT | GESTURE_SINGLE_TAP_BIT);
	ts_info("gesture switch: %s (type=0x%02x)",
		enable ? "ON" : "OFF", gsx_gesture->gesture_type);
}

static ssize_t fingerprint_status_read(struct file *file, char __user *buf,
				       size_t count, loff_t *ppos)
{
	char tmp[8];
	int len;

	if (!gsx_gesture)
		return 0;
	len = scnprintf(tmp, sizeof(tmp), "%d\n", tran_fingerprint_status);
	return simple_read_from_buffer(buf, count, ppos, tmp, len);
}

static ssize_t fingerprint_status_write(struct file *file, const char __user *buf,
					size_t count, loff_t *ppos)
{
	struct goodix_ts_core *cd;
	struct goodix_ts_cmd cmd;
	char tmp[4] = { 0 };
	int val;

	if (!count || !gsx_gesture)
		return -EINVAL;
	if (copy_from_user(tmp, buf, 1))
		return -EFAULT;

	cd = gsx_gesture->ts_core;
	switch (tmp[0]) {
	case '0':
		val = 0;
		break;
	case '1':
		val = 1;
		if (cd && cd->hw_ops && cd->hw_ops->send_cmd) {
			memset(&cmd, 0, sizeof(cmd));
			cmd.len = 5;
			cmd.cmd = 0x30;
			cmd.data[0] = 0x00;
			if (cd->hw_ops->send_cmd(cd, &cmd))
				ts_err("failed send fp enable cmd (1)");
		}
		break;
	case '2':
		val = 2;
		if (cd && cd->hw_ops && cd->hw_ops->send_cmd) {
			memset(&cmd, 0, sizeof(cmd));
			cmd.len = 5;
			cmd.cmd = 0x30;
			cmd.data[0] = 0x01;
			if (cd->hw_ops->send_cmd(cd, &cmd))
				ts_err("failed send fp enable cmd (2)");
		}
		break;
	default:
		ts_info("invalid fingerprint_status value: 0x%02x", tmp[0]);
		return count;
	}

	tran_fingerprint_status = val;
	gsx_gesture->fp_status = val;
	if (val)
		gsx_gesture->gesture_type |= GESTURE_FOD_PRESS_BIT;
	else
		gsx_gesture->gesture_type &= ~GESTURE_FOD_PRESS_BIT;
	ts_info("tran_fingerprint_status = %d", tran_fingerprint_status);
	return count;
}

static const struct proc_ops fp_status_proc_ops = {
	.proc_read = fingerprint_status_read,
	.proc_write = fingerprint_status_write,
};

static int gsx_gesture_init(struct goodix_ts_core *cd,
			    struct goodix_ext_module *module)
{
	struct gesture_module *gsx = module->priv_data;

	if (!cd || !cd->hw_ops->gesture) {
		ts_err("gesture unsupported");
		return -EINVAL;
	}

	gsx->ts_core = cd;
	/* Enable double-tap + FOD by default so wake/FOD work immediately */
	gsx->gesture_type = GESTURE_DOUBLE_TAP_BIT | GESTURE_SINGLE_TAP_BIT | GESTURE_FOD_PRESS_BIT;
	atomic_set(&gsx->registered, 1);

	return 0;
}

static int gsx_gesture_exit(struct goodix_ts_core *cd,
			    struct goodix_ext_module *module)
{
	struct gesture_module *gsx = module->priv_data;

	if (!cd || !cd->hw_ops->gesture) {
		ts_err("gesture unsupported");
		return -EINVAL;
	}

	atomic_set(&gsx->registered, 0);
	return 0;
}

#define TRAN_KEY_SINGLE_TAP	190 /* 0xbe - stock GestureHandle single-tap */
#define TRAN_KEY_FOD		195 /* 0xc3 - stock GestureHandle FOD press/release */
#define TRAN_KEY_DT2W		198 /* 0xc6 - stock GestureHandle KEY_F26 (double-tap/gestures) */

static int gsx_gesture_ist(struct goodix_ts_core *cd,
			   struct goodix_ext_module *module)
{
	struct gesture_module *gsx = module->priv_data;
	struct goodix_ts_hw_ops *hw_ops = cd->hw_ops;
	struct goodix_ts_event gs_event = { 0 };
	int ret;

	if (atomic_read(&cd->suspended) == 0 || gsx->gesture_type == 0)
		return EVT_CONTINUE;

	ts_info("gesture irq: suspended=%d type=0x%02x",
		atomic_read(&cd->suspended), gsx->gesture_type);

	memset(&gs_event, 0, sizeof(gs_event));
	ret = hw_ops->event_handler(cd, &gs_event);
	if (ret) {
		ts_err("failed get gesture data (ret=%d)", ret);
		goto re_send_ges_cmd;
	}

	if (!(gs_event.event_type & EVENT_GESTURE)) {
		ts_debug("invalid event type: 0x%x", gs_event.event_type);
		goto re_send_ges_cmd;
	}

	ts_info("got valid gesture type 0x%x", gs_event.gesture_type);

	switch (gs_event.gesture_type) {
	case GESTURE_SINGLECLICK:
		if (gsx->gesture_type & GESTURE_SINGLE_TAP_BIT) {
			tran_gesture_set_code(GESTURE_SINGLECLICK);
			input_report_key(cd->input_dev, TRAN_KEY_SINGLE_TAP, 1);
			input_sync(cd->input_dev);
			input_report_key(cd->input_dev, TRAN_KEY_SINGLE_TAP, 0);
			input_sync(cd->input_dev);
		}
		break;
	case GESTURE_DOUBLECLICK:
		if (gsx->gesture_type & GESTURE_DOUBLE_TAP_BIT) {
			tran_gesture_set_code(GESTURE_DOUBLECLICK);
			input_report_key(cd->input_dev, TRAN_KEY_DT2W, 1);
			input_sync(cd->input_dev);
			input_report_key(cd->input_dev, TRAN_KEY_DT2W, 0);
			input_sync(cd->input_dev);
		}
		break;
	case GESTURE_FOD_DOWN:
		if (gsx->gesture_type & GESTURE_FOD_PRESS_BIT) {
			tran_gesture_set_code(GESTURE_FOD_DOWN);
			input_report_key(cd->input_dev, TRAN_KEY_FOD, 1);
			input_sync(cd->input_dev);
		}
		break;
	case GESTURE_FOD_UP:
		if (gsx->gesture_type & GESTURE_FOD_PRESS_BIT) {
			tran_gesture_set_code(GESTURE_FOD_UP);
			input_report_key(cd->input_dev, TRAN_KEY_FOD, 0);
			input_sync(cd->input_dev);
		}
		break;
	default:
		ts_info("unsupported gesture: 0x%x", gs_event.gesture_type);
		break;
	}

	/*
	 * The core returns early on EVT_CANCEL_IRQEVT, so after_event_handler()
	 * would not run.  Without clearing the IC event the same gesture stays
	 * pending and every subsequent IRQ is consumed as 0xcc (interrupt
	 * storm, normal touch never reported).  Stock clears it here.
	 */
	if (hw_ops->after_event_handler)
		hw_ops->after_event_handler(cd);

	return EVT_CANCEL_IRQEVT;

re_send_ges_cmd:
	if (hw_ops->gesture(cd, 0))
		ts_info("warning: failed re_send gesture cmd");

	/* Stock clears the event on the invalid/error path too. */
	if (hw_ops->after_event_handler)
		hw_ops->after_event_handler(cd);

	return EVT_CANCEL_IRQEVT;
}

static int gsx_gesture_before_suspend(struct goodix_ts_core *cd,
				      struct goodix_ext_module *module)
{
	struct gesture_module *gsx = module->priv_data;
	int ret;
	const struct goodix_ts_hw_ops *hw_ops = cd->hw_ops;

	if (gsx->gesture_type == 0)
		return EVT_CONTINUE;

	ret = hw_ops->gesture(cd, 0);
	if (ret)
		ts_err("failed enter gesture mode");
	else
		ts_info("enter gesture mode, type[0x%02x]", gsx->gesture_type);

	hw_ops->irq_enable(cd, true);
	if (!enable_irq_wake(cd->irq))
		gsx->irq_wake_enabled = true;
	else
		ts_err("failed to enable irq wake");

	gsx->gesture_suspended = true;

	return EVT_CANCEL_SUSPEND;
}

static int gsx_gesture_before_resume(struct goodix_ts_core *cd,
				     struct goodix_ext_module *module)
{
	struct gesture_module *gsx = module->priv_data;
	const struct goodix_ts_hw_ops *hw_ops = cd->hw_ops;

	if (gsx->irq_wake_enabled) {
		disable_irq_wake(cd->irq);
		gsx->irq_wake_enabled = false;
	}

	if (!gsx->gesture_suspended)
		return EVT_CONTINUE;

	gsx->gesture_suspended = false;
	hw_ops->reset(cd, GOODIX_NORMAL_RESET_DELAY_MS);

	return EVT_CANCEL_RESUME;
}

static struct goodix_ext_module_funcs gsx_gesture_funcs = {
	.irq_event = gsx_gesture_ist,
	.init = gsx_gesture_init,
	.exit = gsx_gesture_exit,
	.before_suspend = gsx_gesture_before_suspend,
	.before_resume = gsx_gesture_before_resume,
};

int gesture_module_init(void)
{
	if (module_initialized)
		return 0;

	gsx_gesture = kzalloc(sizeof(*gsx_gesture), GFP_KERNEL);
	if (!gsx_gesture)
		return -ENOMEM;

	gsx_gesture->module.funcs = &gsx_gesture_funcs;
	gsx_gesture->module.priority = EXTMOD_PRIO_GESTURE;
	gsx_gesture->module.name = "Goodix_gsx_gesture";
	gsx_gesture->module.priv_data = gsx_gesture;
	atomic_set(&gsx_gesture->registered, 0);

	gsx_gesture->fp_proc = proc_create("fingerprint_status", 0666,
					   NULL, &fp_status_proc_ops);
	if (!gsx_gesture->fp_proc)
		ts_err("create proc/fingerprint_status Failed!");
	else
		ts_info("create proc/fingerprint_status Succeeded!");

	module_initialized = true;
	goodix_register_ext_module_no_wait(&gsx_gesture->module);
	ts_info("gesture module init success");

	return 0;
}

void gesture_module_exit(void)
{
	ts_info("gesture module exit");
	if (!module_initialized || !gsx_gesture)
		return;
	if (gsx_gesture->fp_proc) {
		proc_remove(gsx_gesture->fp_proc);
		gsx_gesture->fp_proc = NULL;
	}
	goodix_unregister_ext_module(&gsx_gesture->module);
	kfree(gsx_gesture);
	gsx_gesture = NULL;
	module_initialized = false;
}
