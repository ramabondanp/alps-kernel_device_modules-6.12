// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024 Transsion Inc.
 *
 * Transsion extcon USB driver.
 *
 * This is the firmware's tran_extcon_usb.ko, reconstructed from the X6871
 * .ko (PLAN.md 6.2t).  It is a descendant of the MediaTek
 * drivers/misc/mediatek/extcon/extcon-mtk-usb.c driver that is already in
 * this tree, but it talks to the Transsion tc_tcpc class instead of the MTK
 * TCP notifier and it reports OTG changes to userspace through extcon
 * uevents.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/extcon-provider.h>
#include <linux/power_supply.h>
#include <linux/usb/role.h>
#include <linux/workqueue.h>
#include <linux/namei.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/kobject.h>
#include <linux/string.h>

#include "tc_tcpc.h"
#include "tc_common_class.h"
#include "extcon.h"

static const unsigned int usb_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

struct usb_role_info {
	struct tran_usb_extcon_info *extcon;
	struct delayed_work dwork;
	unsigned int d_role;
};

struct tran_usb_extcon_info {
	struct device *dev;
	struct extcon_dev *edev;
	struct usb_role_switch *role_sw;
	unsigned int c_role;
	struct workqueue_struct *extcon_wq;
	struct power_supply *usb_psy;
	struct notifier_block psy_nb;
	struct delayed_work wq_psy;
	struct notifier_block tcpc_nb;
	bool bypss_typec_sink;
	bool otg_nb_registered;
	bool usb_psy_non_devm;
	unsigned int otg_event;
	struct notifier_block otg_nb;
	struct delayed_work otg_work;
	struct proc_dir_entry *smt_pde;
};

static void tran_usb_extcon_update_role(struct work_struct *work)
{
	struct usb_role_info *role = container_of(to_delayed_work(work),
						  struct usb_role_info, dwork);
	struct tran_usb_extcon_info *extcon = role->extcon;
	unsigned int cur_dr, new_dr;

	cur_dr = extcon->c_role;
	new_dr = role->d_role;

	dev_info(extcon->dev, "cur_dr(%d) new_dr(%d)\n", cur_dr, new_dr);

	/* none -> device */
	if (cur_dr == USB_ROLE_NONE && new_dr == USB_ROLE_DEVICE) {
		extcon_set_state_sync(extcon->edev, EXTCON_USB, true);
	/* none -> host */
	} else if (cur_dr == USB_ROLE_NONE && new_dr == USB_ROLE_HOST) {
		extcon_set_state_sync(extcon->edev, EXTCON_USB_HOST, true);
	/* device -> none */
	} else if (cur_dr == USB_ROLE_DEVICE && new_dr == USB_ROLE_NONE) {
		extcon_set_state_sync(extcon->edev, EXTCON_USB, false);
	/* host -> none */
	} else if (cur_dr == USB_ROLE_HOST && new_dr == USB_ROLE_NONE) {
		extcon_set_state_sync(extcon->edev, EXTCON_USB_HOST, false);
	/* device -> host */
	} else if (cur_dr == USB_ROLE_DEVICE && new_dr == USB_ROLE_HOST) {
		extcon_set_state_sync(extcon->edev, EXTCON_USB, false);
		extcon_set_state_sync(extcon->edev, EXTCON_USB_HOST, true);
	/* host -> device */
	} else if (cur_dr == USB_ROLE_HOST && new_dr == USB_ROLE_DEVICE) {
		extcon_set_state_sync(extcon->edev, EXTCON_USB_HOST, false);
		extcon_set_state_sync(extcon->edev, EXTCON_USB, true);
	}

	/* usb role switch */
	if (extcon->role_sw)
		usb_role_switch_set_role(extcon->role_sw, new_dr);

	extcon->c_role = new_dr;

	kfree(role);
}

static int tran_usb_extcon_set_role(struct tran_usb_extcon_info *extcon,
				    unsigned int role)
{
	struct usb_role_info *role_info;

	/* create and prepare worker */
	role_info = kzalloc(sizeof(*role_info), GFP_ATOMIC);
	if (!role_info)
		return -ENOMEM;

	INIT_DELAYED_WORK(&role_info->dwork, tran_usb_extcon_update_role);

	role_info->extcon = extcon;
	role_info->d_role = role;
	/* issue connection work */
	queue_delayed_work(extcon->extcon_wq, &role_info->dwork, 0);

	return 0;
}

static bool usb_is_online(struct tran_usb_extcon_info *extcon)
{
	union power_supply_propval pval;
	union power_supply_propval tval;
	int ret;

	ret = power_supply_get_property(extcon->usb_psy,
					POWER_SUPPLY_PROP_ONLINE, &pval);
	if (ret < 0) {
		dev_info(extcon->dev, "failed to get online prop\n");
		return false;
	}

	ret = power_supply_get_property(extcon->usb_psy,
					POWER_SUPPLY_PROP_USB_TYPE, &tval);
	if (ret < 0) {
		dev_info(extcon->dev, "failed to get usb type\n");
		return false;
	}

	dev_info(extcon->dev, "online=%d, type=%d\n", pval.intval, tval.intval);

	if (pval.intval && (tval.intval == POWER_SUPPLY_USB_TYPE_SDP ||
			    tval.intval == POWER_SUPPLY_USB_TYPE_CDP))
		return true;

	return false;
}

static void tran_usb_extcon_psy_detector(struct work_struct *work)
{
	struct tran_usb_extcon_info *extcon =
		container_of(to_delayed_work(work),
			     struct tran_usb_extcon_info, wq_psy);

	if (!extcon->usb_psy)
		return;

	if (!usb_is_online(extcon))
		return;

	if (extcon->c_role == USB_ROLE_NONE) {
		dev_info(extcon->dev, "Type-C SINK plug in of psy\n");
		tran_usb_extcon_set_role(extcon, USB_ROLE_DEVICE);
	}
}

static int tran_usb_extcon_psy_notifier(struct notifier_block *nb,
					unsigned long event, void *data)
{
	struct tran_usb_extcon_info *extcon =
		container_of(nb, struct tran_usb_extcon_info, psy_nb);

	if (event != 0)
		return 0;

	if (data != extcon->usb_psy)
		return 0;

	queue_delayed_work(system_power_efficient_wq, &extcon->wq_psy, 0);

	return 0;
}

static int extcon_otg_notifier(struct notifier_block *nb,
			       unsigned long event, void *data)
{
	struct tran_usb_extcon_info *extcon =
		container_of(nb, struct tran_usb_extcon_info, otg_nb);

	extcon->otg_event = event;
	queue_delayed_work(system_wq, &extcon->otg_work, 0);

	return 0;
}

static void do_otg_work(struct work_struct *work)
{
	struct tran_usb_extcon_info *extcon =
		container_of(to_delayed_work(work),
			     struct tran_usb_extcon_info, otg_work);
	char *page;
	char env_name[128];
	char env_state[128];
	char *envp[3];
	int len;
	int n = 0;

	page = (char *)get_zeroed_page(GFP_KERNEL);
	if (!page) {
		dev_err(&extcon->edev->dev,
			"out of memory in extcon_set_state\n");
		return;
	}

	len = sprintf(page, "%s\n", extcon->edev->dev.kobj.name);
	if (len > 0 && page[len - 1] == '\n')
		page[len - 1] = '\0';
	snprintf(env_name, sizeof(env_name), "SWITCH_NAME=%s", page);
	envp[n++] = env_name;

	len = sprintf(page, "%d\n", extcon->otg_event);
	if (len > 0 && page[len - 1] == '\n')
		page[len - 1] = '\0';
	snprintf(env_state, sizeof(env_state), "SWITCH_STATE=%s", page);
	envp[n++] = env_state;

	envp[n] = NULL;

	kobject_uevent_env(&extcon->edev->dev.kobj, KOBJ_CHANGE, envp);

	free_pages((unsigned long)page, 0);
}

static int tran_extcon_pd_tcpc_notifier(struct notifier_block *nb,
					unsigned long event, void *data)
{
	struct tran_usb_extcon_info *extcon =
		container_of(nb, struct tran_usb_extcon_info, tcpc_nb);
	struct device *dev = extcon->dev;

	dev_info(dev, "%s event:%lu\n", __func__, event);

	/* event 0/1 are decorative only, as in the stock build */
	if (event == TC_PD_SRC_TO_SNK) {
		dev_info(dev, "Type-C pw_swap Source to Sink!\n");
		return 0;
	}

	if (event == TC_PD_SNK_TO_SRC) {
		dev_info(dev, "Type-C pw_swap Sink to Source!\n");
		return 0;
	}

	/*
	 * USB_PLUG_IN/OUT and OTG_PLUG_IN/OUT are the events that the stock
	 * tc_tcpc uses for a data cable.  The pre-port source mixed these up
	 * with the OTG/AUDIO events, so a sink re-plug never restored the
	 * device role and adb stayed dead until reboot.
	 */
	if (event == TC_TYPEC_USB_PLUG_IN) {
		if (extcon->bypss_typec_sink)
			return 0;
		dev_info(dev, "Type-C SINK plug in\n");
		tran_usb_extcon_set_role(extcon, USB_ROLE_DEVICE);
		return 0;
	}

	if (event == TC_TYPEC_USB_PLUG_OUT ||
	    event == TC_TYPEC_OTG_PLUG_OUT) {
		dev_info(dev, "Type-C plug out\n");
		tran_usb_extcon_set_role(extcon, USB_ROLE_NONE);
		return 0;
	}

	if (event == TC_TYPEC_OTG_PLUG_IN) {
		dev_info(dev, "Type-C SRC plug in\n");
		tran_usb_extcon_set_role(extcon, USB_ROLE_HOST);
		return 0;
	}

	return 0;
}

#define PROC_FILE_SMT		"mtk_typec"
#define FILE_SMT_U2_CC_MODE	"smt_u2_cc_mode"

static int usb_cc_smt_procfs_show(struct seq_file *s, void *unused)
{
	struct tran_device *tcpc_dev;
	union com_propval val;

	tcpc_dev = tran_get_by_name("tc_tcpc");
	if (IS_ERR_OR_NULL(tcpc_dev)) {
		seq_write(s, "1\n", 2);
		pr_info("tc_tcpc_dev is NULL");
		return -ENODEV;
	}

	memset(&val, 0, sizeof(val));
	tran_dev_get_prop(tcpc_dev, TRAN_PROP_TYPE_CC_SMT, &val);
	seq_printf(s, "%d\n", val.intval & 0xff);

	return 0;
}

static int usb_cc_smt_procfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, usb_cc_smt_procfs_show, pde_data(inode));
}

static const struct proc_ops usb_cc_smt_procfs_fops = {
	.proc_open	= usb_cc_smt_procfs_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
};

static int tran_usb_extcon_procfs_init(struct tran_usb_extcon_info *extcon)
{
	struct proc_dir_entry *file, *root = NULL;
	struct path proc_path;
	int ret = 0;

	if (kern_path("/proc/" PROC_FILE_SMT, 0, &proc_path) == 0) {
		path_put(&proc_path);
		file = proc_create_data(PROC_FILE_SMT "/" FILE_SMT_U2_CC_MODE,
					0400, NULL, &usb_cc_smt_procfs_fops,
					extcon);
	} else {
		root = proc_mkdir(PROC_FILE_SMT, NULL);
		if (!root) {
			dev_info(extcon->dev, "fail creating proc dir: %s\n",
				 PROC_FILE_SMT);
			ret = -ENOMEM;
			goto error;
		}

		file = proc_create_data(FILE_SMT_U2_CC_MODE, 0400, root,
					&usb_cc_smt_procfs_fops, extcon);
	}
	if (!file) {
		dev_info(extcon->dev, "fail creating proc file: %s\n",
			 FILE_SMT_U2_CC_MODE);
		ret = -ENOMEM;
		goto error;
	}
	extcon->smt_pde = file;

	dev_info(extcon->dev, "success creating proc file: %s\n",
		 FILE_SMT_U2_CC_MODE);

error:
	dev_info(extcon->dev, "%s ret:%d\n", __func__, ret);
	return ret;
}

static int tran_usb_extcon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct tran_usb_extcon_info *extcon;
	const char *tcpc_name;
	int ret;

	dev_info(dev, "%s:start!\n", __func__);

	extcon = devm_kzalloc(dev, sizeof(*extcon), GFP_KERNEL);
	if (!extcon)
		return -ENOMEM;

	extcon->dev = dev;

	extcon->edev = devm_extcon_dev_allocate(dev, usb_extcon_cable);
	if (IS_ERR(extcon->edev)) {
		dev_err(dev, "failed to allocate extcon device\n");
		return PTR_ERR(extcon->edev);
	}

	ret = devm_extcon_dev_register(dev, extcon->edev);
	if (ret < 0) {
		dev_info(dev, "failed to register extcon device\n");
		return ret;
	}

	INIT_DELAYED_WORK(&extcon->otg_work, do_otg_work);

	extcon->otg_nb.notifier_call = extcon_otg_notifier;
	ret = extcon_register_notifier(extcon->edev, EXTCON_USB_HOST,
				       &extcon->otg_nb);
	if (ret < 0)
		dev_err(dev, "fail to register notifer\n");
	else
		extcon->otg_nb_registered = true;

	extcon->role_sw = usb_role_switch_get(dev);
	if (IS_ERR(extcon->role_sw)) {
		dev_err(dev, "failed to get usb role\n");
		ret = PTR_ERR(extcon->role_sw);
		extcon->role_sw = NULL;
		goto err_otg;
	}

	if (extcon->role_sw)
		extcon->c_role = USB_ROLE_NONE;

	extcon->bypss_typec_sink =
		of_find_property(np, "mediatek,bypss-typec-sink", NULL) != NULL;

	if (of_find_property(np, "mediatek,u2", NULL) &&
	    of_property_read_string(np, "tcpc", &tcpc_name) == 0 &&
	    strcmp(tcpc_name, "type_c_port0") == 0)
		tran_usb_extcon_procfs_init(extcon);

	extcon->extcon_wq = alloc_workqueue("tran-extcon-usb",
					    WQ_HIGHPRI | WQ_UNBOUND, 1);
	if (!extcon->extcon_wq) {
		ret = -ENOMEM;
		goto err_role_sw;
	}

	extcon->usb_psy = devm_power_supply_get_by_reference(dev, "charger");
	if (IS_ERR_OR_NULL(extcon->usb_psy)) {
		extcon->usb_psy = power_supply_get_by_name("primary_chg");
		if (extcon->usb_psy)
			extcon->usb_psy_non_devm = true;
		else
			dev_info(dev, "fail to get usb_psy\n");
	}

	if (extcon->usb_psy) {
		INIT_DELAYED_WORK(&extcon->wq_psy,
				  tran_usb_extcon_psy_detector);
		extcon->psy_nb.notifier_call =
			tran_usb_extcon_psy_notifier;
		if (power_supply_reg_notifier(&extcon->psy_nb)) {
			dev_err(dev, "fail to register notifer\n");
			dev_info(dev, "failed to init psy\n");
		}
	} else {
		dev_info(dev, "failed to init psy\n");
	}

	extcon->tcpc_nb.notifier_call = tran_extcon_pd_tcpc_notifier;
	ret = register_tc_tcpc_notifier(&extcon->tcpc_nb);
	if (ret < 0) {
		dev_info(dev, "extcon register tc_tcpc failed!\n");
		dev_err(dev, "failed to init tc_tcpc\n");
	}

	platform_set_drvdata(pdev, extcon);

	return 0;

err_role_sw:
	if (extcon->smt_pde) {
		proc_remove(extcon->smt_pde);
		extcon->smt_pde = NULL;
	}
	if (extcon->role_sw) {
		usb_role_switch_put(extcon->role_sw);
		extcon->role_sw = NULL;
	}
err_otg:
	if (extcon->otg_nb_registered) {
		extcon_unregister_notifier(extcon->edev, EXTCON_USB_HOST,
					   &extcon->otg_nb);
		extcon->otg_nb_registered = false;
	}
	cancel_delayed_work_sync(&extcon->otg_work);
	return ret;
}

static void tran_usb_extcon_remove(struct platform_device *pdev)
{
	struct tran_usb_extcon_info *extcon = platform_get_drvdata(pdev);

	if (extcon->smt_pde) {
		proc_remove(extcon->smt_pde);
		extcon->smt_pde = NULL;
	}

	unregister_tc_tcpc_notifier(&extcon->tcpc_nb);
	if (extcon->usb_psy) {
		power_supply_unreg_notifier(&extcon->psy_nb);
		cancel_delayed_work_sync(&extcon->wq_psy);
		if (extcon->usb_psy_non_devm) {
			power_supply_put(extcon->usb_psy);
			extcon->usb_psy = NULL;
			extcon->usb_psy_non_devm = false;
		}
	}
	if (extcon->otg_nb_registered) {
		extcon_unregister_notifier(extcon->edev, EXTCON_USB_HOST,
					   &extcon->otg_nb);
		extcon->otg_nb_registered = false;
	}
	cancel_delayed_work_sync(&extcon->otg_work);

	if (extcon->extcon_wq) {
		destroy_workqueue(extcon->extcon_wq);
		extcon->extcon_wq = NULL;
	}

	if (extcon->role_sw) {
		usb_role_switch_put(extcon->role_sw);
		extcon->role_sw = NULL;
	}
}

static void tran_usb_extcon_shutdown(struct platform_device *pdev)
{
	struct tran_usb_extcon_info *extcon = platform_get_drvdata(pdev);

	dev_info(extcon->dev, "shutdown\n");
}

static const struct of_device_id tran_usb_extcon_of_match[] = {
	{ .compatible = "transsion,extcon-usb" },
	{ },
};
MODULE_DEVICE_TABLE(of, tran_usb_extcon_of_match);

static struct platform_driver tran_usb_extcon_driver = {
	.probe		= tran_usb_extcon_probe,
	.remove		= tran_usb_extcon_remove,
	.shutdown	= tran_usb_extcon_shutdown,
	.driver		= {
		.name		= "tran-extcon-usb",
		.of_match_table	= tran_usb_extcon_of_match,
	},
};

static int __init tran_usb_extcon_init(void)
{
	return platform_driver_register(&tran_usb_extcon_driver);
}

static void __exit tran_usb_extcon_exit(void)
{
	platform_driver_unregister(&tran_usb_extcon_driver);
}

module_init(tran_usb_extcon_init);
module_exit(tran_usb_extcon_exit);

MODULE_DESCRIPTION("Transsion Extcon USB Driver");
MODULE_LICENSE("GPL");
