// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024 Transsion Inc.
 *
 * Transsion TKV (bootloader key/value region) driver, reconstructed from
 * the X6871 tkv.ko.
 *
 * The bootloader passes a physical base in the kernel command line
 * ("tkv=<decimal>").  The 32 KiB region holds NUL-separated key=value
 * pairs.  This module maps it, exposes /proc/tkv for reading the pairs and
 * for writing the LCM inversion value (g_tkv).
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/kstrtox.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/overflow.h>
#include <linux/pfn.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <asm/pgtable.h>

#define TKV_REGION_SIZE		0x8000
#define TKV_CMDLINE_SIZE	0x800
#define TKV_PAIR_MAX		0x5a

int g_tkv;
unsigned long tkv_base;
unsigned long g_tkv_base;
char tkv_value[TKV_REGION_SIZE];
static char __tkv_cmdline[TKV_CMDLINE_SIZE];
static struct proc_dir_entry *tkv_pde;


char *tkv_get_cmd(void)
{
	struct device_node *node;
	const char *bootargs;
	size_t len;

	if (__tkv_cmdline[0])
		return __tkv_cmdline;

	node = of_find_node_opts_by_path("/chosen", NULL);
	if (!node) {
		pr_info("%s: failed to get /chosen \n", __func__);
		return __tkv_cmdline;
	}

	bootargs = of_get_property(node, "bootargs", NULL);
	if (!bootargs) {
		pr_info("%s: failed to get bootargs\n", __func__);
		of_node_put(node);
		return __tkv_cmdline;
	}

	len = strlen(bootargs);
	if (len + 1 < TKV_CMDLINE_SIZE)
		memcpy(__tkv_cmdline, bootargs, len + 1);

	pr_info("%s: bootargs: %s\n", __func__, bootargs);
	of_node_put(node);

	return __tkv_cmdline;
}

int get_tkv_base(void)
{
	char *cmd = tkv_get_cmd();
	char *p = strstr(cmd, "tkv=");

	if (!p)
		return -1;

	g_tkv_base = simple_strtoull(p + 4, NULL, 10);
	pr_info("g_tkv_base =0x%lx\n", g_tkv_base);

	return 0;
}

/*
 * The firmware trusts the bootloader address and ioremaps it blindly.  A
 * wrong or malicious tkv= value would then read 32 KiB from an arbitrary
 * physical address, which can hit device registers with read side effects.
 * Only map the region when every page is System RAM or falls within the
 * stock /reserved-memory/mblock-24-tkv-reserved DT node (which uses no-map
 * and is therefore excluded from System RAM).  If both checks fail we skip
 * /proc/tkv instead of risking a bus hang.
 */
static bool tkv_region_in_reserved_dt(unsigned long base, size_t size)
{
	struct device_node *np;
	struct resource res;
	unsigned long end;

	if (!size || check_add_overflow(base, size - 1, &end))
		return false;

	np = of_find_node_by_path("/reserved-memory/mblock-24-tkv-reserved");
	if (!np)
		return false;

	if (of_address_to_resource(np, 0, &res)) {
		of_node_put(np);
		return false;
	}
	of_node_put(np);

	return base >= res.start && end <= res.end;
}

static bool tkv_region_is_valid(unsigned long base, size_t size)
{
	unsigned long pfn = PHYS_PFN(base);
	unsigned long pages = DIV_ROUND_UP(size, PAGE_SIZE);
	unsigned long i;

	if (!base || !IS_ALIGNED(base, PAGE_SIZE))
		return false;

	if (tkv_region_in_reserved_dt(base, size))
		return true;

	for (i = 0; i < pages; i++)
		if (!page_is_ram(pfn + i))
			return false;

	return true;
}

static bool tkv_is_hidden_key(const char *key, size_t len)
{
	static const char hidden[] = "ro.bootimage.build.fingerprint";
	size_t i;

	if (len != sizeof(hidden) - 1)
		return false;

	for (i = 0; i < len; i++) {
		if (key[i] != hidden[i])
			return false;
	}

	return true;
}

static int tkv_proc_show(struct seq_file *s, void *unused)
{
	char *p = tkv_value;
	char *end = tkv_value + TKV_REGION_SIZE;

	while (p < end && *p) {
		size_t len = strnlen(p, end - p);
		char *eq;
		size_t klen, vlen;

		if (p + len >= end)
			break;

		eq = memchr(p, '=', len);
		if (!eq) {
			p += len + 1;
			continue;
		}

		klen = eq - p;
		vlen = len - klen - 1;

		if (klen > TKV_PAIR_MAX || vlen > TKV_PAIR_MAX) {
			pr_info("k/v pair corrupted.\n");
		} else if (!tkv_is_hidden_key(p, klen)) {
			char key[TKV_PAIR_MAX + 1];
			char val[TKV_PAIR_MAX + 1];

			memcpy(key, p, klen);
			key[klen] = '\0';
			memcpy(val, eq + 1, vlen);
			val[vlen] = '\0';

			pr_info("tkv proc [%s]: [%s]\n", key, val);
			seq_printf(s, "%s=%s\n", key, val);
		}

		p += len + 1;
	}

	return 0;
}

static int tkv_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, tkv_proc_show, NULL);
}

static ssize_t tkv_proc_write(struct file *file, const char __user *buf,
			      size_t count, loff_t *ppos)
{
	char tmp[64];
	unsigned long long val;

	if (count > 0x3f)
		return -EINVAL;

	memset(tmp, 0, sizeof(tmp));
	if (copy_from_user(tmp, buf, count))
		return -EFAULT;

	tmp[count] = '\0';

	if (kstrtoull(tmp, 10, &val))
		return -EINVAL;

	g_tkv = (int)val;
	pr_info("write lcm invertion to %d\n", g_tkv);

	return count;
}

static const struct proc_ops tkv_proc_fops = {
	.proc_open	= tkv_proc_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
	.proc_write	= tkv_proc_write,
};

static int __init tkv_init(void)
{
	struct proc_dir_entry *pde;

	if (get_tkv_base() < 0) {
		pr_info("get_tkv_base fail\n");
		return 0;
	}

	if (!tkv_region_is_valid(g_tkv_base, TKV_REGION_SIZE)) {
		pr_err("tkv: 0x%lx..0x%lx is outside System RAM and reserved DT, refusing to map\n",
		       g_tkv_base, g_tkv_base + TKV_REGION_SIZE - 1);
		return 0;
	}

	tkv_base = (unsigned long)ioremap_prot(g_tkv_base, TKV_REGION_SIZE,
					       pgprot_val(PAGE_KERNEL));
	if (!tkv_base) {
		pr_info("tkv_base map fail\n");
		return -ENOMEM;
	}

	pr_info("tkv_base =0x%lx\n", tkv_base);

	memcpy(tkv_value, (void *)tkv_base, TKV_REGION_SIZE);
	pr_info("tkv_value=%.*s tkv_base=%.*s\n",
		(int)strnlen(tkv_value, TKV_REGION_SIZE), tkv_value,
		(int)strnlen((char *)tkv_base, TKV_REGION_SIZE),
		(char *)tkv_base);

	pr_info("create /fs/tran_tkv\n");
	pde = proc_create("tkv", 0, NULL, &tkv_proc_fops);
	if (!pde) {
		pr_err(" Fail\n");
		iounmap((void __iomem *)tkv_base);
		tkv_base = 0;
		return -ENOMEM;
	}
	tkv_pde = pde;
	pr_info(" OK\n");

	return 0;
}

static void __exit tkv_exit(void)
{
	proc_remove(tkv_pde);
	tkv_pde = NULL;

	if (tkv_base) {
		iounmap((void __iomem *)tkv_base);
		tkv_base = 0;
	}
}

module_init(tkv_init);
module_exit(tkv_exit);

MODULE_LICENSE("GPL");
