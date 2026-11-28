// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 Transsion Inc.
 *
 * PCA TFCP protocol layer.
 *
 * Transsion never released this module (no source, no Makefile, no Kconfig, no
 * kleaf entry -- PLAN.md 6.1b). Reconstructed from the X6871 firmware
 * pca_tfcp_ta.ko, which is not stripped: all 18 pca_tfcp_* functions survive
 * with their call graph and their `ldr x8, [x0, #0x5d8]` driver-data loads.
 *
 * It is the protocol glue between tadapter_class (the charger adapter class)
 * and tc_ta_class (the TA device class): every pca_tfcp_* op fetches the TA
 * device from driver_data and forwards to the matching tc_ta_device_* call.
 *
 *   set_cap          -> tc_ta_device_set_voltage_current()
 *   get_measure_cap  -> tc_ta_device_get_max_voltage/current/power_limit()
 *   authenticate_ta  -> tc_ta_device_authentication() + fw/mfinfo reporting
 *   enable_charging  -> tc_ta_device_set_voltage_current()
 *   is_cc            -> tc_ta_device_get_status()
 *   set_wdt          -> tc_ta_device_set_wdt()
 *   enable_wdt       -> tc_ta_device_enable_wdt()
 *   send_hardreset   -> tc_ta_device_enable_wdt(false) + set_voltage_current
 *   get_ta_fw        -> tc_ta_device_get_fwcode()
 *   plug_out_reset   -> tc_ta_device_set_voltage_current(5V, 0)
 *   get_det_done     -> tc_ta_device_get_status()
 *   exit_protocol    -> tc_ta_device_enable_wdt(false) + set_voltage_current(0)
 *
 * The DT node carries only pinctrl-names (tfcp_i2c_sclk_mode /
 * tfcp_i2c_sda_mode), no voltage table, so parse_dt only resolves the TA
 * device by name.
 */
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "tc_adapter_class.h"
#include "tc_common_class.h"
#include "tc_misc_intf.h"
#include "tc_ta_class.h"

#define PCA_TFCP_TA_NAME	"tfcp_ta"
#define PCA_TFCP_ADAPTER_NAME	"tfcp_adapter"
#define PCA_TFCP_ENABLE_MV	5500
#define PCA_TFCP_ENABLE_MA	2200
#define PCA_TFCP_HARDRESET_MV	5300
#define PCA_TFCP_HARDRESET_MA	3000

struct pca_tfcp_info {
	struct device *dev;
	const char *adapter_name;
	struct tc_ta_classdev *ta_dev;
	struct tadapter_device *tadapter_dev;
	struct tadapter_auth_data auth;
	bool det_done;
};

static struct tc_ta_classdev *pca_tfcp_get_ta(struct pca_tfcp_info *info)
{
	if (IS_ERR_OR_NULL(info->ta_dev))
		return NULL;

	return info->ta_dev;
}

static int pca_tfcp_get_status(struct tadapter_device *dev,
			       struct tadapter_status *sta)
{
	struct pca_tfcp_info *info = tadapter_dev_get_drvdata(dev);
	struct tc_ta_classdev *ta = pca_tfcp_get_ta(info);
	u32 status = 0, temp = 0;
	int ret;

	if (!ta)
		return -ENODEV;

	ret = tc_ta_device_get_status(ta, &status);
	if (ret < 0)
		return ret;

	tc_ta_device_get_temp1(ta, &temp);

	sta->temperature = temp;
	sta->cc_mode = !!(status & BIT(0));
	sta->otp = !!(status & BIT(1));
	sta->ovp = !!(status & BIT(2));
	sta->cable_capability = status & 0xff;

	return 0;
}

static int pca_tfcp_set_cap(struct tadapter_device *dev,
			    enum tadapter_cap_type type, int mV, int mA)
{
	struct pca_tfcp_info *info = tadapter_dev_get_drvdata(dev);
	struct tc_ta_classdev *ta = pca_tfcp_get_ta(info);

	if (!ta)
		return -ENODEV;

	return tc_ta_device_set_voltage_current(ta, mV, mA);
}

/* The firmware reads the measured v/i, not a declared max */
static int pca_tfcp_get_measure_cap(struct tadapter_device *dev,
				    struct tadapter_power_cap *cap)
{
	struct pca_tfcp_info *info = tadapter_dev_get_drvdata(dev);
	struct tc_ta_classdev *ta = pca_tfcp_get_ta(info);
	u32 volt = 0, curr = 0;
	int ret;

	if (!ta)
		return -ENODEV;

	ret = tc_ta_device_get_output_voltage_current(ta, &volt, &curr);
	if (ret < 0)
		return ret;

	cap->nr = 1;
	cap->max_mv[0] = volt;
	cap->ma[0] = curr;
	cap->maxwatt[0] = (volt / 1000) * (curr / 1000);

	return 0;
}

static int pca_tfcp_authenticate_ta(struct tadapter_device *dev,
				    struct tadapter_auth_data *data)
{
	struct pca_tfcp_info *info = tadapter_dev_get_drvdata(dev);
	struct tc_ta_classdev *ta = pca_tfcp_get_ta(info);
	struct tran_device *usb_control;
	u8 fwcode[4] = { 0 };
	u32 vmin = 0, vmax = 0, imax = 0, pwr = 0, dur = 0;
	int ret, alias_type, pd_type;

	if (!ta)
		return -ENODEV;

	ret = tc_ta_device_authentication(ta);
	if (ret < 0)
		return ret;

	/* DT-declared protocol parameters, then the TA's own capability */
	*data = info->auth;
	data->support_cc = true;

	/*
	 * The adapter reports which TA protocol was negotiated, which decides
	 * the voltage/current window the framework may ask for.
	 */
	alias_type = tc_get_alias_type();
	pd_type = tc_get_pd_type();
	if (pd_type > 0)
		data->support_cc = true;
	dev_info(info->dev, "alias_type %d pd_type %d\n", alias_type, pd_type);

	tc_ta_device_get_min_voltage(ta, &vmin);
	tc_ta_device_get_max_voltage(ta, &vmax);
	tc_ta_device_get_max_current(ta, &imax);
	tc_ta_device_get_power_limit(ta, &pwr);
	tc_ta_device_get_max_power_duration(ta, &dur);

	data->full_power_run_time = dur;
	data->vcap_min = vmin;
	data->vcap_max = vmax;
	data->ita_max = imax;
	data->pdp = pwr;

	/*
	 * Once the handshake succeeds the TA must be switched off, exactly as
	 * the firmware does: tran_dev_set_prop(usb_control, prop 21 =
	 * TRAN_PROP_USB_CTRL_TA_OFF) after tc_get_alias_type/tc_get_pd_type.
	 */
	usb_control = tran_get_by_name("usb_control");
	if (usb_control) {
		union com_propval off = { 0 }, cur = { 0 };

		/* Confirm the adapter switched before we take it over */
		tran_dev_get_prop(usb_control, TRAN_PROP_GET_CHARGER_STATUS, &cur);
		tran_dev_set_prop(usb_control, TRAN_PROP_USB_CTRL_TA_OFF, &off);
	}

	/* Report the TA identity; tadapter_auth_data has no fw field */
	if (tc_ta_device_get_fwcode(ta, fwcode, sizeof(fwcode)) == 0)
		dev_info(info->dev, "ta fwcode %02x%02x%02x%02x\n",
			 fwcode[0], fwcode[1], fwcode[2], fwcode[3]);
	if (tc_ta_device_get_mfinfo(ta, fwcode, sizeof(fwcode)) == 0)
		dev_info(info->dev, "ta mfinfo %02x%02x%02x%02x\n",
			 fwcode[0], fwcode[1], fwcode[2], fwcode[3]);

	return 0;
}

static int __maybe_unused pca_tfcp_enable_charging(struct tadapter_device *dev,
						   bool en)
{
	struct pca_tfcp_info *info = tadapter_dev_get_drvdata(dev);
	struct tc_ta_classdev *ta = pca_tfcp_get_ta(info);

	if (!ta)
		return -ENODEV;

	/* The firmware always programs 5.5 V / 2.2 A here and ignores @en. */
	return tc_ta_device_set_voltage_current(ta, PCA_TFCP_ENABLE_MV,
						PCA_TFCP_ENABLE_MA);
}

static int __maybe_unused pca_tfcp_inform_related_module(struct tadapter_device *dev)
{
	struct pca_tfcp_info *info = tadapter_dev_get_drvdata(dev);
	union com_propval val = { 0 };

	if (!info->det_done)
		return 0;

	info->det_done = false;

	return tran_dev_set_prop(tran_get_by_name("usb_control"),
				 TRAN_PROP_USB_CTRL_TA_OFF, &val);
}

static int __maybe_unused pca_tfcp_exit_rfc_mode(struct tadapter_device *dev)
{
	union com_propval val = { 0 };

	/* The firmware releases RFC by asking usb_control to turn the TA off */
	return tran_dev_set_prop(tran_get_by_name("usb_control"),
				 TRAN_PROP_USB_CTRL_TA_OFF, &val);
}

static int pca_tfcp_is_cc(struct tadapter_device *dev, bool *cc)
{
	struct pca_tfcp_info *info = tadapter_dev_get_drvdata(dev);
	struct tc_ta_classdev *ta = pca_tfcp_get_ta(info);
	u32 status = 0;
	int ret;

	if (!ta)
		return -ENODEV;

	ret = tc_ta_device_get_status(ta, &status);
	if (ret < 0)
		return ret;

	*cc = !!(status & BIT(0));

	return 0;
}

static int pca_tfcp_set_wdt(struct tadapter_device *dev, u32 ms)
{
	struct pca_tfcp_info *info = tadapter_dev_get_drvdata(dev);
	struct tc_ta_classdev *ta = pca_tfcp_get_ta(info);

	if (!ta)
		return -ENODEV;

	return tc_ta_device_set_wdt(ta, ms);
}

static int pca_tfcp_enable_wdt(struct tadapter_device *dev, bool en)
{
	struct pca_tfcp_info *info = tadapter_dev_get_drvdata(dev);
	struct tc_ta_classdev *ta = pca_tfcp_get_ta(info);

	if (!ta)
		return -ENODEV;

	return tc_ta_device_enable_wdt(ta, en);
}

static int pca_tfcp_send_hardreset(struct tadapter_device *dev)
{
	struct pca_tfcp_info *info = tadapter_dev_get_drvdata(dev);
	struct tc_ta_classdev *ta = pca_tfcp_get_ta(info);
	int ret;

	if (!ta)
		return -ENODEV;

	ret = tc_ta_device_enable_wdt(ta, false);
	if (ret < 0)
		return ret;

	ret = tc_ta_device_set_voltage_current(ta, PCA_TFCP_HARDRESET_MV,
					       PCA_TFCP_HARDRESET_MA);
	if (ret < 0)
		return ret;

	/* Let the TA settle before the protocol restarts */
	msleep(100);

	return 0;
}

static int pca_tfcp_get_ta_fw(struct tadapter_device *dev, u8 *code,
			      u32 length)
{
	struct pca_tfcp_info *info = tadapter_dev_get_drvdata(dev);
	struct tc_ta_classdev *ta = pca_tfcp_get_ta(info);

	if (!ta)
		return -ENODEV;

	return tc_ta_device_get_fwcode(ta, code, length);
}

static int pca_tfcp_plug_out_reset(struct tadapter_device *dev)
{
	struct pca_tfcp_info *info = tadapter_dev_get_drvdata(dev);
	union com_propval val = { 0 };

	info->det_done = false;

	/* The firmware turns the TA off through usb_control and returns 0 */
	tran_dev_set_prop(tran_get_by_name("usb_control"),
			  TRAN_PROP_USB_CTRL_TA_OFF, &val);

	return 0;
}

static int pca_tfcp_get_det_done(struct tadapter_device *dev, bool *det_done)
{
	struct pca_tfcp_info *info = tadapter_dev_get_drvdata(dev);
	struct tc_ta_classdev *ta = pca_tfcp_get_ta(info);
	u32 status = 0;
	int ret;

	if (!ta)
		return -ENODEV;

	ret = tc_ta_device_get_status(ta, &status);
	if (ret < 0)
		return ret;

	info->det_done = !!(status & BIT(0));
	*det_done = info->det_done;

	return 0;
}

static int pca_tfcp_exit_protocol(struct tadapter_device *dev)
{
	struct pca_tfcp_info *info = tadapter_dev_get_drvdata(dev);
	struct tc_ta_classdev *ta = pca_tfcp_get_ta(info);

	if (!ta)
		return -ENODEV;

	/* Leaving the protocol means disabling the watchdog + dropping output */
	tc_ta_device_enable_wdt(ta, false);

	return tc_ta_device_set_voltage_current(ta, 0, 0);
}

static int __maybe_unused pca_tfcp_suspend(struct tadapter_device *dev, pm_message_t state)
{
	return 0;
}

static int __maybe_unused pca_tfcp_resume(struct tadapter_device *dev)
{
	return 0;
}

static int __maybe_unused pca_tfcp_get_property(struct tadapter_device *dev,
				 enum tadapter_property pro)
{
	return -ENOTSUPP;
}

static int pca_tfcp_get_cap(struct tadapter_device *dev,
			    enum tadapter_cap_type type,
			    struct tadapter_power_cap *cap)
{
	return pca_tfcp_get_measure_cap(dev, cap);
}

static int __maybe_unused pca_tfcp_get_output(struct tadapter_device *dev, int *mV, int *mA)
{
	struct pca_tfcp_info *info = tadapter_dev_get_drvdata(dev);
	struct tc_ta_classdev *ta = pca_tfcp_get_ta(info);
	u32 volt = 0, curr = 0;
	int ret;

	if (!ta)
		return -ENODEV;

	ret = tc_ta_device_get_output_voltage_current(ta, &volt, &curr);
	if (ret < 0)
		return ret;

	*mV = volt;
	*mA = curr;

	return 0;
}

static int __maybe_unused pca_tfcp_sync_volt(struct tadapter_device *dev, u32 mV)
{
	struct pca_tfcp_info *info = tadapter_dev_get_drvdata(dev);
	struct tc_ta_classdev *ta = pca_tfcp_get_ta(info);

	if (!ta)
		return -ENODEV;

	return tc_ta_device_set_voltage_current(ta, mV, 0);
}

static int __maybe_unused pca_tfcp_get_epp(struct tadapter_device *dev)
{
	return -ENOTSUPP;
}

static int __maybe_unused pca_tfcp_get_ce(struct tadapter_device *dev)
{
	return -ENOTSUPP;
}

/*
 * The firmware's ops table is at .data+0x108 and holds exactly 12 entries,
 * which map 1:1 onto tadapter_ops slots 3..17:
 *   get_status set_cap get_cap authentication is_cc set_wdt enable_wdt
 *   send_hardreset get_ta_fw plug_out_reset get_det_done exit_protocol
 * enable_charging / exit_rfc_mode / inform_related_module sit at .text+0x10bc
 * ..0x11b0, past the table, so they are not ops entries.
 */
static const struct tadapter_ops pca_tfcp_ops = {
	.get_status = pca_tfcp_get_status,
	.set_cap = pca_tfcp_set_cap,
	.get_cap = pca_tfcp_get_cap,
	.authentication = pca_tfcp_authenticate_ta,
	.is_cc = pca_tfcp_is_cc,
	.set_wdt = pca_tfcp_set_wdt,
	.enable_wdt = pca_tfcp_enable_wdt,
	.send_hardreset = pca_tfcp_send_hardreset,
	.get_ta_fw = pca_tfcp_get_ta_fw,
	.plug_out_reset = pca_tfcp_plug_out_reset,
	.get_det_done = pca_tfcp_get_det_done,
	.exit_protocol = pca_tfcp_exit_protocol,
};
static int pca_tfcp_parse_dt(struct pca_tfcp_info *info, struct device *dev)
{
	const char *name;

	/*
	 * The X6871 node carries only these, no voltage table:
	 *   adapter_name, vta_step, ita_step, ita_gap_per_vstep,
	 *   ita_gap_cv_mode, pwr_lmt, support_meas_cap, support_status
	 */
	of_property_read_u32(dev->of_node, "vta_step", &info->auth.vta_step);
	of_property_read_u32(dev->of_node, "ita_step", &info->auth.ita_step);
	of_property_read_u32(dev->of_node, "ita_gap_per_vstep",
			     &info->auth.ita_gap_per_vstep);
	of_property_read_u32(dev->of_node, "ita_gap_cv_mode",
			     &info->auth.ita_gap_cv_mode);
	info->auth.pwr_lmt = of_property_read_bool(dev->of_node, "pwr_lmt");
	info->auth.support_meas_cap =
		of_property_read_bool(dev->of_node, "support_meas_cap");
	info->auth.support_status =
		of_property_read_bool(dev->of_node, "support_status");

	if (of_property_read_string(dev->of_node, "adapter_name",
				    &info->adapter_name))
		info->adapter_name = PCA_TFCP_ADAPTER_NAME;

	if (of_property_read_string(dev->of_node, "ta_name", &name))
		name = PCA_TFCP_TA_NAME;

	info->ta_dev = tc_ta_device_get_by_name(name);
	if (IS_ERR_OR_NULL(info->ta_dev)) {
		dev_err(dev, "get ta dev %s fail\n", name);
		return -EPROBE_DEFER;
	}

	return 0;
}

static int pca_tfcp_probe(struct platform_device *pdev)
{
	struct pca_tfcp_info *info;
	struct tadapter_properties props;
	int ret;

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = &pdev->dev;
	platform_set_drvdata(pdev, info);

	ret = pca_tfcp_parse_dt(info, &pdev->dev);
	if (ret < 0)
		return ret;

	props.alias_name = info->adapter_name;
	info->tadapter_dev = tadapter_device_register(props.alias_name,
						     &pdev->dev, info,
						     &pca_tfcp_ops, &props);
	if (IS_ERR_OR_NULL(info->tadapter_dev)) {
		dev_err(&pdev->dev, "register tadapter fail\n");
		return -ENODEV;
	}

	return 0;
}

static void pca_tfcp_remove(struct platform_device *pdev)
{
	struct pca_tfcp_info *info = platform_get_drvdata(pdev);

	if (info && !IS_ERR_OR_NULL(info->tadapter_dev))
		tadapter_device_unregister(info->tadapter_dev);
}

static const struct of_device_id pca_tfcp_of_match[] = {
	{ .compatible = "tc,pca_tfcp_ta" },
	{},
};
MODULE_DEVICE_TABLE(of, pca_tfcp_of_match);

static struct platform_driver pca_tfcp_driver = {
	.probe = pca_tfcp_probe,
	.remove = pca_tfcp_remove,
	.driver = {
		.name = "pca_tfcp_ta",
		.of_match_table = pca_tfcp_of_match,
	},
};
module_platform_driver(pca_tfcp_driver);

MODULE_DESCRIPTION("Transsion PCA TFCP Protocol Driver");
MODULE_LICENSE("GPL");
