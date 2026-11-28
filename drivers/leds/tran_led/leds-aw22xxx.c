// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2017 Transsion Inc.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/leds.h>
#include "leds-aw22xxx.h"
#include "leds-aw22xxx-reg.h"
#include "leds-aw22xxx-cfg.h"
#include "tran_led_class.h"

EXPORT_SYMBOL(aw22xxx_reg_access);

static int aw22xxx_audio_gain_led_effects(struct aw22xxx *aw22xxx,
					  unsigned int gain);
static int tran_led_cmd_cfg_2(struct aw22xxx *aw22xxx, unsigned int type,
			      unsigned int num2, unsigned int num3);

/******************************************************
 *
 * Marco
 *
 ******************************************************/
#define AW22XXX_I2C_NAME "aw22xxx_led"

#define AW22XXX_DRIVER_VERSION "V1.3.0"

#define AW_I2C_RETRIES 2
#define AW_I2C_RETRY_DELAY 1
#define AW_READ_CHIPID_RETRIES 2
#define AW_READ_CHIPID_RETRY_DELAY 1

/******************************************************
 *
 * aw22xxx led parameter
 *
 ******************************************************/
static char *aw22xxx_fw_name = "aw22xxx_fw.bin";

/*
 * The firmware's default music gain table, read out of .rodata+0x1b0.
 * aw22xxx_cover_gain() is exported and walks whatever table the DT provided;
 * this one is the fallback when the property is absent or invalid.
 */
static const struct aw22xxx_music_gain aw_music_gain_table[] = {
	{0, 0x00}, {1, 0x3f}, {2, 0x3b}, {3, 0x29},
	{4, 0x25}, {5, 0x22}, {6, 0x20}, {7, 0x1f},
	{8, 0x1e}, {9, 0x1a}, {10, 0x13}, {11, 0x12},
	{12, 0x12}, {13, 0x11}, {14, 0x0e}, {15, 0x0d},
};

/* Default multi-color names, also from the firmware .rodata. */
static const char *aw22xxx_default_multicolor[] = { "B85", "E23", "S30" };

#define AW22XXX_IMAX_NAME_MAX       32
static char aw22xxx_imax_name[][AW22XXX_IMAX_NAME_MAX] = {
    {"AW22XXX_IMAX_2mA"},
    {"AW22XXX_IMAX_3mA"},
    {"AW22XXX_IMAX_4mA"},
    {"AW22XXX_IMAX_6mA"},
    {"AW22XXX_IMAX_9mA"},
    {"AW22XXX_IMAX_10mA"},
    {"AW22XXX_IMAX_15mA"},
    {"AW22XXX_IMAX_20mA"},
    {"AW22XXX_IMAX_30mA"},
    {"AW22XXX_IMAX_40mA"},
    {"AW22XXX_IMAX_45mA"},
    {"AW22XXX_IMAX_60mA"},
    {"AW22XXX_IMAX_75mA"},
};
static char aw22xxx_imax_code[] = {
    AW22XXX_IMAX_2mA,
    AW22XXX_IMAX_3mA,
    AW22XXX_IMAX_4mA,
    AW22XXX_IMAX_6mA,
    AW22XXX_IMAX_9mA,
    AW22XXX_IMAX_10mA,
    AW22XXX_IMAX_15mA,
    AW22XXX_IMAX_20mA,
    AW22XXX_IMAX_30mA,
    AW22XXX_IMAX_40mA,
    AW22XXX_IMAX_45mA,
    AW22XXX_IMAX_60mA,
    AW22XXX_IMAX_75mA,
};
/******************************************************
 *
 * aw22xxx i2c write/read
 *
 ******************************************************/
static int aw22xxx_i2c_write(struct aw22xxx *aw22xxx, 
         unsigned char reg_addr, unsigned char reg_data)
{
    int ret = -1;
    unsigned char cnt = 0;

    while(cnt < AW_I2C_RETRIES) {
        ret = i2c_smbus_write_byte_data(aw22xxx->i2c, reg_addr, reg_data);
        if(ret < 0) {
            pr_err("%s: i2c_write cnt=%d error=%d\n", __func__, cnt, ret);
        } else {
            break;
        }
        cnt ++;
        msleep(AW_I2C_RETRY_DELAY);
    }

    return ret;
}

static int aw22xxx_i2c_read(struct aw22xxx *aw22xxx, 
        unsigned char reg_addr, unsigned char *reg_data)
{
    int ret = -1;
    unsigned char cnt = 0;

    while(cnt < AW_I2C_RETRIES) {
        ret = i2c_smbus_read_byte_data(aw22xxx->i2c, reg_addr);
        if(ret < 0) {
            pr_err("%s: i2c_read cnt=%d error=%d\n", __func__, cnt, ret);
        } else {
            *reg_data = ret;
            break;
        }
        cnt ++;
        msleep(AW_I2C_RETRY_DELAY);
    }

    return ret;
}

static int aw22xxx_i2c_write_bits(struct aw22xxx *aw22xxx,
         unsigned char reg_addr, unsigned char mask, unsigned char reg_data)
{
    unsigned char reg_val;

    aw22xxx_i2c_read(aw22xxx, reg_addr, &reg_val);
    reg_val &= mask;
    reg_val |= (reg_data & (~mask));
    aw22xxx_i2c_write(aw22xxx, reg_addr, reg_val);

    return 0;
}

#ifdef AW22XXX_FLASH_I2C_WRITES
static int aw22xxx_i2c_writes(struct aw22xxx *aw22xxx,
        unsigned char reg_addr, unsigned char *buf, unsigned int len)
{
    int ret = -1;
    unsigned char *data;

    data = kmalloc(len+1, GFP_KERNEL);
    if (data == NULL) {
        pr_err("%s: can not allocate memory\n", __func__);
        return  -ENOMEM;
    }

    data[0] = reg_addr;
    memcpy(&data[1], buf, len);

    ret = i2c_master_send(aw22xxx->i2c, data, len+1);
    if (ret < 0) {
        pr_err("%s: i2c master send error\n", __func__);
    }

    kfree(data);

    return ret;
}
#endif

/*****************************************************
 *
 * aw22xxx led cfg
 *
 *****************************************************/
static int aw22xxx_reg_page_cfg(struct aw22xxx *aw22xxx, unsigned char page)
{
    aw22xxx_i2c_write(aw22xxx, REG_PAGE, page);
    return 0;
}

static int aw22xxx_sw_reset(struct aw22xxx *aw22xxx)
{
    aw22xxx_i2c_write(aw22xxx, REG_SRST, AW22XXX_SRSTW);
    msleep(2);
    return 0;
}

static int aw22xxx_chip_enable(struct aw22xxx *aw22xxx, bool flag)
{
    if(flag) {
        aw22xxx_i2c_write_bits(aw22xxx, REG_GCR,
                BIT_GCR_CHIPEN_MASK, BIT_GCR_CHIPEN_ENABLE);
    } else {
        aw22xxx_i2c_write_bits(aw22xxx, REG_GCR,
                BIT_GCR_CHIPEN_MASK, BIT_GCR_CHIPEN_DISABLE);
    }
    msleep(2);
    return 0;
}

/*
static int aw22xxx_mcu_wakeup(struct aw22xxx *aw22xxx, bool flag)
{
    if(flag) {
        aw22xxx_i2c_write_bits(aw22xxx, REG_MCUCTR,
                BIT_MCUCTR_MCU_WAKEUP_MASK, BIT_MCUCTR_MCU_WAKEUP_ENABLE);
    } else {
        aw22xxx_i2c_write_bits(aw22xxx, REG_MCUCTR,
                BIT_MCUCTR_MCU_WAKEUP_MASK, BIT_MCUCTR_MCU_WAKEUP_DISABLE);
    }
    return 0;
}
*/

static int aw22xxx_mcu_reset(struct aw22xxx *aw22xxx, bool flag)
{
    if(flag) {
        aw22xxx_i2c_write_bits(aw22xxx, REG_MCUCTR,
                BIT_MCUCTR_MCU_RESET_MASK, BIT_MCUCTR_MCU_RESET_ENABLE);
    } else {
        aw22xxx_i2c_write_bits(aw22xxx, REG_MCUCTR,
                BIT_MCUCTR_MCU_RESET_MASK, BIT_MCUCTR_MCU_RESET_DISABLE);
    }
    return 0;
}

static int aw22xxx_mcu_enable(struct aw22xxx *aw22xxx, bool flag)
{
    if(flag) {
        aw22xxx_i2c_write_bits(aw22xxx, REG_MCUCTR,
                BIT_MCUCTR_MCU_WORK_MASK, BIT_MCUCTR_MCU_WORK_ENABLE);
    } else {
        aw22xxx_i2c_write_bits(aw22xxx, REG_MCUCTR,
                BIT_MCUCTR_MCU_WORK_MASK, BIT_MCUCTR_MCU_WORK_DISABLE);
    }
    return 0;
}

static int aw22xxx_led_task0_cfg(struct aw22xxx *aw22xxx, unsigned char task)
{
    aw22xxx_i2c_write(aw22xxx, REG_TASK0, task);
    return 0;
}

static int aw22xxx_led_task1_cfg(struct aw22xxx *aw22xxx, unsigned char task)
{
    aw22xxx_i2c_write(aw22xxx, REG_TASK1, task);
    return 0;
}

/*
static int aw22xxx_get_pst(struct aw22xxx *aw22xxx, unsigned char *pst)
{
    int ret = -1;
    ret = aw22xxx_i2c_read(aw22xxx, REG_PST, pst);
    if(ret < 0) {
        pr_err("%s: error=%d\n", __func__, ret);
    }
    return ret;
}
*/

static int aw22xxx_imax_cfg(struct aw22xxx *aw22xxx, unsigned char imax)
{
    if(imax > 0x0f) {
        imax = 0x0f;
    }
    aw22xxx_reg_page_cfg(aw22xxx, AW22XXX_REG_PAGE0);
    aw22xxx_i2c_write(aw22xxx, REG_IMAX, imax);

    return 0;
}

/*
static int aw22xxx_audio_enable(struct aw22xxx *aw22xxx, bool flag)
{
    if(flag) {
        aw22xxx_i2c_write_bits(aw22xxx, REG_AUDCTR,
                BIT_AUDCTR_AUDEN_MASK, BIT_AUDCTR_AUDEN_ENABLE);
    } else {
        aw22xxx_i2c_write_bits(aw22xxx, REG_AUDCTR,
                BIT_AUDCTR_AUDEN_MASK, BIT_AUDCTR_AUDEN_DISABLE);
    }
    return 0;
}

static int aw22xxx_agc_enable(struct aw22xxx *aw22xxx, bool flag)
{
    if(flag) {
        aw22xxx_i2c_write_bits(aw22xxx, REG_AUDCTR,
                BIT_AUDCTR_AGCEN_MASK, BIT_AUDCTR_AGCEN_ENABLE);
    } else {
        aw22xxx_i2c_write_bits(aw22xxx, REG_AUDCTR,
                BIT_AUDCTR_AGCEN_MASK, BIT_AUDCTR_AGCEN_DISABLE);
    }
    return 0;
}

static int aw22xxx_agc_igain_cfg(struct aw22xxx *aw22xxx, unsigned char igain)
{
    if(igain > 0x3f) {
        igain = 0x3f;
    }
    aw22xxx_i2c_write(aw22xxx, REG_IGAIN, igain);

    return 0;
}

static int aw22xxx_get_agc_gain(struct aw22xxx *aw22xxx, unsigned char *gain)
{
    int ret = -1;
    ret = aw22xxx_i2c_read(aw22xxx, REG_GAIN, gain);
    if(ret < 0) {
        pr_err("%s: error=%d\n", __func__, ret);
    }
    return ret;
}
*/

static int aw22xxx_dbgctr_cfg(struct aw22xxx *aw22xxx, unsigned char cfg)
{
    if(cfg >= (AW22XXX_DBGCTR_MAX-1)) {
        cfg = AW22XXX_DBGCTR_NORMAL;
    }
    aw22xxx_i2c_write(aw22xxx, REG_DBGCTR, cfg);

    return 0;
}

static int aw22xxx_addr_cfg(struct aw22xxx *aw22xxx, unsigned int addr)
{
    aw22xxx_i2c_write(aw22xxx, REG_ADDR1, (unsigned char)((addr>>0)&0xff));
    aw22xxx_i2c_write(aw22xxx, REG_ADDR2, (unsigned char)((addr>>8)&0xff));

    return 0;
}

static int aw22xxx_data_cfg(struct aw22xxx *aw22xxx, unsigned int data)
{
    aw22xxx_i2c_write(aw22xxx, REG_DATA, data);

    return 0;
}
/*
static int aw22xxx_get_fw_version(struct aw22xxx *aw22xxx)
{
    unsigned char reg_val;
    unsigned char i;

    pr_info("%s: enter\n", __func__);

    aw22xxx_reg_page_cfg(aw22xxx, AW22XXX_REG_PAGE0);
    aw22xxx_sw_reset(aw22xxx);
    aw22xxx_chip_enable(aw22xxx, true);
    aw22xxx_mcu_enable(aw22xxx, true);
    aw22xxx_led_task0_cfg(aw22xxx, 0xff);
    aw22xxx_mcu_reset(aw22xxx, false);
    msleep(2);
    aw22xxx_mcu_reset(aw22xxx, true);

    aw22xxx_reg_page_cfg(aw22xxx, AW22XXX_REG_PAGE2);
    aw22xxx->fw_version = 0;
    for(i=0; i<4; i++) {
        aw22xxx_i2c_read(aw22xxx, 0xe9+i, &reg_val);
        aw22xxx->fw_version |= (reg_val<<((3-i)*8));
    }
    pr_info("%s: flash fw version: 0x%04x\n", __func__, aw22xxx->fw_version);

    aw22xxx_i2c_read(aw22xxx, 0xd0, &reg_val);
    switch(reg_val) {
        case AW22118_CHIPID:
            aw22xxx->chip_type= AW22118;
            pr_info("%s: flash chipid: aw22118\n", __func__);
            break;
        case AW22127_CHIPID:
            aw22xxx->chip_type= AW22127;
            pr_info("%s: flash chipid: aw22127\n", __func__);
            break;
        default:
            pr_err("%s: unknown id=0x%02x\n", __func__, reg_val);
    }

    aw22xxx_reg_page_cfg(aw22xxx, AW22XXX_REG_PAGE0);

    return 0;
}
*/

static int aw22xxx_led_display(struct aw22xxx *aw22xxx)
{
    aw22xxx_addr_cfg(aw22xxx, 0x00e1);
    aw22xxx_dbgctr_cfg(aw22xxx, AW22XXX_DBGCTR_SFR);
    aw22xxx_data_cfg(aw22xxx, 0x3d);
    aw22xxx_dbgctr_cfg(aw22xxx, AW22XXX_DBGCTR_NORMAL);
    return 0;
}

static int aw22xxx_led_off(struct aw22xxx *aw22xxx)
{
    aw22xxx_led_task0_cfg(aw22xxx, 0xff);
    aw22xxx_mcu_reset(aw22xxx, true);
    return 0;
}

static void aw22xxx_brightness_work(struct work_struct *work)
{
    struct aw22xxx *aw22xxx = container_of(work, struct aw22xxx,
          brightness_work);

    pr_info("%s: enter\n", __func__);

    aw22xxx_led_off(aw22xxx);
    aw22xxx_chip_enable(aw22xxx, false);
    if(aw22xxx->cdev.brightness) {
        aw22xxx_chip_enable(aw22xxx, true);
        aw22xxx_mcu_enable(aw22xxx, true);

        aw22xxx_imax_cfg(aw22xxx, (unsigned char)aw22xxx->imax);
        aw22xxx_led_display(aw22xxx);

        aw22xxx_led_task0_cfg(aw22xxx, 0x82);
        aw22xxx_mcu_reset(aw22xxx, false);
    }
}

static void aw22xxx_set_brightness(struct led_classdev *cdev,
           enum led_brightness brightness)
{
    struct aw22xxx *aw22xxx = container_of(cdev, struct aw22xxx, cdev);

    aw22xxx->cdev.brightness = brightness;

    schedule_work(&aw22xxx->brightness_work);
}

static void aw22xxx_task_work(struct work_struct *work)
{
    struct aw22xxx *aw22xxx = container_of(work, struct aw22xxx,
          task_work);

    pr_info("%s: enter\n", __func__);

    aw22xxx_led_off(aw22xxx);
    aw22xxx_chip_enable(aw22xxx, false);
    if(aw22xxx->task0) {
        aw22xxx_chip_enable(aw22xxx, true);
        aw22xxx_mcu_enable(aw22xxx, true);

        aw22xxx_imax_cfg(aw22xxx, (unsigned char)aw22xxx->imax);
        aw22xxx_led_display(aw22xxx);

        aw22xxx_led_task0_cfg(aw22xxx, aw22xxx->task0);
        aw22xxx_led_task1_cfg(aw22xxx, aw22xxx->task1);
        aw22xxx_mcu_reset(aw22xxx, false);
    }
}

static int aw22xxx_led_init(struct aw22xxx *aw22xxx)
{
    unsigned int imax = aw22xxx->imax;

    pr_info("%s: enter\n", __func__);

    if (imax >= ARRAY_SIZE(aw22xxx_imax_code))
        imax = 0;

    aw22xxx_sw_reset(aw22xxx);
    aw22xxx_chip_enable(aw22xxx, true);
    aw22xxx_imax_cfg(aw22xxx, aw22xxx_imax_code[imax]);
    aw22xxx_chip_enable(aw22xxx, false);

    pr_info("%s: exit\n", __func__);

    return 0;
}

/*****************************************************
 *
 * firmware/cfg update
 *
 *****************************************************/
static void aw22xxx_cfg_loaded(const struct firmware *cont, void *context)
{
    struct aw22xxx *aw22xxx = context;
    int i = 0;
    int retry = 0;
    unsigned char page = 0;
    unsigned char reg_addr = 0;
    unsigned char reg_val = 0;

    pr_info("%s: enter\n", __func__);

    mutex_lock(&aw22xxx->cfg_lock);

    if (!cont) {
        pr_err("%s: failed to read %s\n", __func__, aw22xxx_cfg_name[aw22xxx->effect]);
        release_firmware(cont);
        mutex_unlock(&aw22xxx->cfg_lock);
        return;
    }

    for (;;) {
        pr_info("%s: loaded %s - size: %zu\n", __func__,
                aw22xxx_cfg_name[aw22xxx->effect], cont->size);

        for (i = 0; i < cont->size; i += 2) {
            if (*(cont->data + i) == 0xff) {
                page = *(cont->data + i + 1);
            }
            if (aw22xxx->cfg == 1) {
                aw22xxx_i2c_write(aw22xxx, *(cont->data + i),
                                  *(cont->data + i + 1));
            } else if (page == AW22XXX_REG_PAGE1) {
                reg_addr = *(cont->data + i);
                if ((reg_addr < 0x2b) && (reg_addr > 0x0f)) {
                    reg_addr -= 0x10;
                    reg_val = (unsigned char)
                        ((aw22xxx->rgb[reg_addr / 3] >>
                          (8 * (2 - reg_addr % 3))) & 0xff);
                    aw22xxx_i2c_write(aw22xxx, *(cont->data + i), reg_val);
                } else {
                    aw22xxx_i2c_write(aw22xxx, *(cont->data + i),
                                      *(cont->data + i + 1));
                }
            } else {
                aw22xxx_i2c_write(aw22xxx, *(cont->data + i),
                                  *(cont->data + i + 1));
            }

            if (page == AW22XXX_REG_PAGE0) {
                reg_addr = *(cont->data + i);
                reg_val = *(cont->data + i + 1);
                /* gcr chip enable delay */
                if ((reg_addr == REG_GCR) &&
                    ((reg_val & BIT_GCR_CHIPEN_ENABLE) ==
                     BIT_GCR_CHIPEN_ENABLE))
                    msleep(2);
            }
        }

        /*
         * Firmware retries the whole table up to three times while the chip
         * reports GCR disabled and PST not yet 1.  All three checks were
         * visible in cfg_loaded: GCR (0x02), PST (0x07), 10 ms polls.
         */
        aw22xxx_i2c_read(aw22xxx, REG_GCR, &reg_val);
        if (reg_val)
            break;

        aw22xxx_i2c_read(aw22xxx, REG_PST, &reg_val);
        if (reg_val == 1) {
            pr_info("%s: reg_val == 0x01,goto out\n", __func__);
            break;
        }

        if (retry > 2)
            break;

        pr_info("%s: cfg update check failed,need retry! count(%d)\n",
                __func__, retry);
        msleep(50);
        retry++;
    }

    release_firmware(cont);

    pr_info("%s: cfg update complete\n", __func__);
    mutex_unlock(&aw22xxx->cfg_lock);

    /*
     * Re-apply the audio gain after a cfg update if the audio path is on;
     * the register write in aw22xxx_audio_gain_led_effects() is otherwise
     * lost by the config table.
     */
    aw22xxx_i2c_read(aw22xxx, REG_AUDCTR, &reg_val);
    if (reg_val & BIT_AUDCTR_AUDEN_ENABLE)
        aw22xxx_audio_gain_led_effects(aw22xxx, aw22xxx->audio_gain);
}

static int aw22xxx_cfg_update(struct aw22xxx *aw22xxx)
{
    unsigned int imax;
    unsigned int code;

    pr_info("%s: enter\n", __func__);

    if (aw22xxx->effect < ARRAY_SIZE(aw22xxx_cfg_name)) {
        pr_info("%s: cfg name=%s\n", __func__,
                aw22xxx_cfg_name[aw22xxx->effect]);
    } else {
        pr_err("%s: effect 0x%02x over max value \n", __func__,
               aw22xxx->effect);
        return -1;
    }

    if (aw22xxx->fw_flags != AW22XXX_FLAG_FW_OK) {
        pr_err("%s: fw update error: not compelte \n", __func__);
        return -2;
    }

    if (aw22xxx->reset_imax) {
        imax = aw22xxx->imax;
        if (imax < ARRAY_SIZE(aw22xxx_imax_code)) {
            code = aw22xxx_imax_code[imax];
            if (code > 0x0f)
                code = 0x0f;
            aw22xxx_i2c_write(aw22xxx, REG_PAGE, AW22XXX_REG_PAGE0);
            aw22xxx_i2c_write(aw22xxx, REG_IMAX, code);
        }
    }

    return request_firmware_nowait(THIS_MODULE, true,
                aw22xxx_cfg_name[aw22xxx->effect], aw22xxx->dev, GFP_KERNEL,
                aw22xxx, aw22xxx_cfg_loaded);
}

static int aw22xxx_container_update(struct aw22xxx *aw22xxx,
        struct aw22xxx_container *aw22xxx_fw)
{
    unsigned int i;
    unsigned char reg_val;
    unsigned int tmp_bist;
#ifdef AW22XXX_FLASH_I2C_WRITES
    unsigned int tmp_len;
#endif

    /* chip enable*/
    aw22xxx_reg_page_cfg(aw22xxx, AW22XXX_REG_PAGE0);
    aw22xxx_sw_reset(aw22xxx);
    aw22xxx_chip_enable(aw22xxx, true);
    aw22xxx_mcu_enable(aw22xxx, true);

    /* flash cfg */
    aw22xxx_i2c_write(aw22xxx, 0x80, 0xec);
    aw22xxx_i2c_write(aw22xxx, 0x35, 0x29);
    //aw22xxx_i2c_write(aw22xxx, 0x37, 0xba);
    aw22xxx_i2c_write(aw22xxx, 0x38, aw22xxx_fw->key);

    /* flash erase*/
    aw22xxx_i2c_write(aw22xxx, 0x22, 0x00);
    aw22xxx_i2c_write(aw22xxx, 0x21, 0x00);
    aw22xxx_i2c_write(aw22xxx, 0x20, 0x03);
    aw22xxx_i2c_write(aw22xxx, 0x30, 0x03);
    aw22xxx_i2c_write(aw22xxx, 0x23, 0x00);
    msleep(40);
    aw22xxx_i2c_write(aw22xxx, 0x30, 0x00);
    aw22xxx_i2c_write(aw22xxx, 0x22, 0x40);
    aw22xxx_i2c_write(aw22xxx, 0x21, 0x00);
    aw22xxx_i2c_write(aw22xxx, 0x30, 0x02);
    aw22xxx_i2c_write(aw22xxx, 0x23, 0x00);
    msleep(6);
    aw22xxx_i2c_write(aw22xxx, 0x30, 0x00);
    aw22xxx_i2c_write(aw22xxx, 0x22, 0x42);
    aw22xxx_i2c_write(aw22xxx, 0x21, 0x00);
    aw22xxx_i2c_write(aw22xxx, 0x30, 0x02);
    aw22xxx_i2c_write(aw22xxx, 0x23, 0x00);
    msleep(6);
    aw22xxx_i2c_write(aw22xxx, 0x30, 0x00);
    aw22xxx_i2c_write(aw22xxx, 0x22, 0x44);
    aw22xxx_i2c_write(aw22xxx, 0x21, 0x00);
    aw22xxx_i2c_write(aw22xxx, 0x30, 0x02);
    aw22xxx_i2c_write(aw22xxx, 0x23, 0x00);
    msleep(6);
    aw22xxx_i2c_write(aw22xxx, 0x30, 0x00);
    aw22xxx_i2c_write(aw22xxx, 0x20, 0x00);

#ifdef AW22XXX_FLASH_I2C_WRITES
    /* flash writes */
    aw22xxx_i2c_write(aw22xxx, 0x20, 0x03);
    for(i=0; i<aw22xxx_fw->len; i+=tmp_len) {
        aw22xxx_i2c_write(aw22xxx, 0x22, ((i>>8)&0xff));
        aw22xxx_i2c_write(aw22xxx, 0x21, ((i>>0)&0xff));
        aw22xxx_i2c_write(aw22xxx, 0x11, 0x01);
        aw22xxx_i2c_write(aw22xxx, 0x30, 0x04);
        if((aw22xxx_fw->len - i) < MAX_FLASH_WRITE_BYTE_SIZE) {
            tmp_len = aw22xxx_fw->len - i;
        } else {
            tmp_len = MAX_FLASH_WRITE_BYTE_SIZE;
        }
        aw22xxx_i2c_writes(aw22xxx, 0x23, &aw22xxx_fw->data[i], tmp_len);
        aw22xxx_i2c_write(aw22xxx, 0x11, 0x00);
        aw22xxx_i2c_write(aw22xxx, 0x30, 0x00);
    }
    aw22xxx_i2c_write(aw22xxx, 0x20, 0x00);
#else
    /* flash write */
    aw22xxx_i2c_write(aw22xxx, 0x20, 0x03);
    for(i=0; i<aw22xxx_fw->len; i++) {
        aw22xxx_i2c_write(aw22xxx, 0x22, ((i>>8)&0xff));
        aw22xxx_i2c_write(aw22xxx, 0x21, ((i>>0)&0xff));
        aw22xxx_i2c_write(aw22xxx, 0x30, 0x04);
        aw22xxx_i2c_write(aw22xxx, 0x23, aw22xxx_fw->data[i]);
        aw22xxx_i2c_write(aw22xxx, 0x30, 0x00);
    }
    aw22xxx_i2c_write(aw22xxx, 0x20, 0x00);
#endif

    /* bist check */
    aw22xxx_sw_reset(aw22xxx);
    aw22xxx_chip_enable(aw22xxx, true);
    aw22xxx_mcu_enable(aw22xxx, true);
    aw22xxx_i2c_write(aw22xxx, 0x22, (((aw22xxx_fw->len-1)>>8)&0xff));
    aw22xxx_i2c_write(aw22xxx, 0x21, (((aw22xxx_fw->len-1)>>0)&0xff));
    aw22xxx_i2c_write(aw22xxx, 0x24, 0x07);
    msleep(5);
    aw22xxx_i2c_read(aw22xxx, 0x24, &reg_val);
    if(reg_val == 0x05) {
        aw22xxx_i2c_read(aw22xxx, 0x25, &reg_val);
        tmp_bist = reg_val;
        aw22xxx_i2c_read(aw22xxx, 0x26, &reg_val);
        tmp_bist |= (reg_val<<8);
        if(tmp_bist == aw22xxx_fw->bist) {
            pr_info("%s: bist check pass, bist=0x%04x\n", __func__, aw22xxx_fw->bist);
        } else {
            pr_err("%s: bist check fail, bist=0x%04x\n", __func__, aw22xxx_fw->bist);
            pr_err("%s: fw update failed, please reset phone\n", __func__);
            return -1;
        }
    } else {
        pr_err("%s: bist check is running, reg0x24=0x%02x\n", __func__, reg_val);
    }
    aw22xxx_i2c_write(aw22xxx, 0x24, 0x00);

    return 0;
}

static void aw22xxx_fw_loaded(const struct firmware *cont, void *context)
{
    struct aw22xxx *aw22xxx = context;
    struct aw22xxx_container *aw22xxx_fw;
    int i = 0;
    int ret = -1;
    char tmp_buf[32] = {0};
    unsigned int shift = 0;
    unsigned short check_sum = 0;
    unsigned char reg_val = 0;
    unsigned int tmp_bist = 0;

    pr_info("%s: enter\n", __func__);

    if (!cont) {
        pr_err("%s: failed to read %s\n", __func__, aw22xxx_fw_name);
        release_firmware(cont);
        return;
    }

    pr_info("%s: loaded %s - size: %zu\n", __func__, aw22xxx_fw_name,
                    cont ? cont->size : 0);
/*
    for(i=0; i<cont->size; i++) {
        pr_info("%s: addr:0x%04x, data:0x%02x\n", __func__, i, *(cont->data+i));
    }
*/
    /* check sum */
    for(i=2; i<cont->size; i++) {
        check_sum += cont->data[i];
    }
    if(check_sum != (unsigned short)((cont->data[0]<<8)|(cont->data[1]))) {
        pr_err("%s: check sum err: check_sum=0x%04x\n", __func__, check_sum);
        release_firmware(cont);
        return;
    } else {
        pr_info("%s: check sum pass : 0x%04x\n", __func__, check_sum);
    }

    /* get fw info */
    aw22xxx_fw = kzalloc(cont->size + 4*sizeof(unsigned int), GFP_KERNEL);
    if (!aw22xxx_fw) {
        release_firmware(cont);
        pr_err("%s: Error allocating memory\n", __func__);
        return;
    }
    shift += 2;

    pr_info("%s: fw chip_id : 0x%02x\n", __func__, cont->data[0+shift]);
    shift += 1;

    memcpy(tmp_buf, &cont->data[0+shift], 16);
    pr_info("%s: fw customer: %s\n", __func__, tmp_buf);
    shift += 16;

    memcpy(tmp_buf, &cont->data[0+shift], 8);
    pr_info("%s: fw project: %s\n", __func__, tmp_buf);
    shift += 8;

    aw22xxx_fw->version = (cont->data[0+shift]<<24) | (cont->data[1+shift]<<16) |
            (cont->data[2+shift]<<8) | (cont->data[3+shift]<<0);
    pr_info("%s: fw version : 0x%04x\n", __func__, aw22xxx_fw->version);
    shift += 4;

    // reserved
    shift += 3;

    aw22xxx_fw->bist = (cont->data[0+shift]<<8) | (cont->data[1+shift]<<0);
    pr_info("%s: fw bist : 0x%04x\n", __func__, aw22xxx_fw->bist);
    shift += 2;

    aw22xxx_fw->key = cont->data[0+shift];
    pr_info("%s: fw key : 0x%04x\n", __func__, aw22xxx_fw->key);
    shift += 1;

    // reserved
    shift += 1;

    aw22xxx_fw->len = (cont->data[0+shift]<<8) | (cont->data[1+shift]<<0);
    pr_info("%s: fw len : 0x%04x\n", __func__, aw22xxx_fw->len);
    shift += 2;

    memcpy(aw22xxx_fw->data, &cont->data[shift], aw22xxx_fw->len);
    release_firmware(cont);

    /* check version */
    //aw22xxx_get_fw_version(aw22xxx);

    /* bist check */
    aw22xxx_sw_reset(aw22xxx);
    aw22xxx_chip_enable(aw22xxx, true);
    aw22xxx_mcu_enable(aw22xxx, true);
    aw22xxx_i2c_write(aw22xxx, 0x22, (((aw22xxx_fw->len-1)>>8)&0xff));
    aw22xxx_i2c_write(aw22xxx, 0x21, (((aw22xxx_fw->len-1)>>0)&0xff));
    aw22xxx_i2c_write(aw22xxx, 0x24, 0x07);
    msleep(5);
    aw22xxx_i2c_read(aw22xxx, 0x24, &reg_val);
    if(reg_val == 0x05) {
        aw22xxx_i2c_read(aw22xxx, 0x25, &reg_val);
        tmp_bist = reg_val;
        aw22xxx_i2c_read(aw22xxx, 0x26, &reg_val);
        tmp_bist |= (reg_val<<8);
        if(tmp_bist == aw22xxx_fw->bist) {
            pr_info("%s: bist check pass, bist=0x%04x\n",
                    __func__, aw22xxx_fw->bist);
            if(aw22xxx->fw_update == 0) {
                kfree(aw22xxx_fw);
                aw22xxx_i2c_write(aw22xxx, 0x24, 0x00);
                aw22xxx_led_init(aw22xxx);
                aw22xxx->fw_flags = AW22XXX_FLAG_FW_OK;
                return;
            } else {
                pr_info("%s: fw version: 0x%04x, force update fw\n",
                    __func__, aw22xxx_fw->version);
            }
        } else {
            pr_info("%s: bist check fail, fw bist=0x%04x, flash bist=0x%04x\n",
                    __func__, aw22xxx_fw->bist, tmp_bist);
            pr_info("%s: find new fw: 0x%04x, need update\n",
                    __func__, aw22xxx_fw->version);
        }
    } else {
        pr_err("%s: bist check is running, reg0x24=0x%02x\n", __func__, reg_val);
        pr_info("%s: fw need update\n", __func__);
    }
    aw22xxx_i2c_write(aw22xxx, 0x24, 0x00);

    /* fw update */
    ret = aw22xxx_container_update(aw22xxx, aw22xxx_fw);
    if(ret) {
        aw22xxx->fw_flags = AW22XXX_FLAG_FW_FAIL;
    } else {
        aw22xxx->fw_flags = AW22XXX_FLAG_FW_OK;
    }
    kfree(aw22xxx_fw);

    aw22xxx->fw_update = 0;

    aw22xxx_led_init(aw22xxx);

    pr_info("%s: exit\n", __func__);
}

static int aw22xxx_fw_update(struct aw22xxx *aw22xxx)
{
    pr_info("%s: enter\n", __func__);

    aw22xxx->fw_flags = AW22XXX_FLAG_FW_UPDATE;

    return request_firmware_nowait(THIS_MODULE, true,
                aw22xxx_fw_name, aw22xxx->dev, GFP_KERNEL,
                aw22xxx, aw22xxx_fw_loaded);
}

#ifdef AWINIC_FW_UPDATE_DELAY
static enum hrtimer_restart aw22xxx_fw_timer_func(struct hrtimer *timer)
{
    struct aw22xxx *aw22xxx = container_of(timer, struct aw22xxx, fw_timer);

    pr_info("%s: enter\n", __func__);

    schedule_work(&aw22xxx->fw_work);
    
    return HRTIMER_NORESTART;
}
#endif

static void aw22xxx_fw_work_routine(struct work_struct *work)
{
    struct aw22xxx *aw22xxx = container_of(work, struct aw22xxx, fw_work);

    pr_info("%s: enter\n", __func__);
    
    aw22xxx_fw_update(aw22xxx);
}

static void aw22xxx_cfg_work_routine(struct work_struct *work)
{
    struct aw22xxx *aw22xxx = container_of(work, struct aw22xxx, cfg_work);

    pr_info("%s: enter\n", __func__);

    aw22xxx_cfg_update(aw22xxx);
}

static int aw22xxx_fw_init(struct aw22xxx *aw22xxx)
{
#ifdef AWINIC_FW_UPDATE_DELAY
    int fw_timer_val = 2000;

    atomic_set(&aw22xxx->soft_show_en, 0);
    atomic_set(&aw22xxx->lamp_init_done, 0);
    hrtimer_init(&aw22xxx->fw_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    aw22xxx->fw_timer.function = aw22xxx_fw_timer_func;
    INIT_WORK(&aw22xxx->fw_work, aw22xxx_fw_work_routine);
    INIT_WORK(&aw22xxx->cfg_work, aw22xxx_cfg_work_routine);
    hrtimer_start(&aw22xxx->fw_timer,
            ktime_set(fw_timer_val/1000, (fw_timer_val%1000)*1000000),
            HRTIMER_MODE_REL);
#else
    INIT_WORK(&aw22xxx->fw_work, aw22xxx_fw_work_routine);
    INIT_WORK(&aw22xxx->cfg_work, aw22xxx_cfg_work_routine);
    schedule_work(&aw22xxx->fw_work);
#endif
    return 0;
}

/******************************************************
 *
 * irq 
 *
 ******************************************************/
static void aw22xxx_interrupt_clear(struct aw22xxx *aw22xxx)
{
    unsigned char reg_val;

    pr_info("%s: enter\n", __func__);

    aw22xxx_i2c_read(aw22xxx, REG_INTST, &reg_val);
    pr_info("%s: reg INTST=0x%x\n", __func__, reg_val);
}

static void aw22xxx_interrupt_setup(struct aw22xxx *aw22xxx)
{
    pr_info("%s: enter\n", __func__);

    aw22xxx_interrupt_clear(aw22xxx);

    aw22xxx_i2c_write_bits(aw22xxx, REG_INTEN,
            BIT_INTEN_FUNCMPE_MASK, BIT_INTEN_FUNCMPE_ENABLE);
}

static irqreturn_t aw22xxx_irq(int irq, void *data)
{
    struct aw22xxx *aw22xxx = data;
    unsigned char reg_val;

    pr_info("%s: enter\n", __func__);

    aw22xxx_i2c_read(aw22xxx, REG_INTST, &reg_val);
    pr_info("%s: reg INTST=0x%x\n", __func__, reg_val);

    if(reg_val & BIT_INTST_FUNCMPE) {
        pr_info("%s: functions compelte!\n", __func__);
        aw22xxx_reg_page_cfg(aw22xxx, AW22XXX_REG_PAGE0);
        aw22xxx_mcu_reset(aw22xxx, true);
        aw22xxx_mcu_enable(aw22xxx, false);
        aw22xxx_chip_enable(aw22xxx, false);
        pr_info("%s: enter standby mode!\n", __func__);
    }
    pr_info("%s exit\n", __func__);

    return IRQ_HANDLED;
}

/*****************************************************
 *
 * device tree
 *
 *****************************************************/
static int aw22xxx_parse_dt(struct device *dev, struct aw22xxx *aw22xxx,
        struct device_node *np)
{
    aw22xxx->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
    if (aw22xxx->reset_gpio < 0) {
        dev_err(dev, "%s: no reset gpio provided, will not HW reset device\n", __func__);
        return -1;
    } else {
        dev_info(dev, "%s: reset gpio provided ok\n", __func__);
    }
    aw22xxx->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
    if (aw22xxx->irq_gpio < 0) {
        dev_err(dev, "%s: no irq gpio provided, will not suppport intterupt\n", __func__);
        return -1;
    } else {
        dev_info(dev, "%s: irq gpio provided ok\n", __func__);
    }

    return 0;
}

static int aw22xxx_gpio_to_irq(int gpio)
{
	return gpiod_to_irq(gpio_to_desc(gpio));
}

static int aw22xxx_hw_reset(struct aw22xxx *aw22xxx)
{
    pr_info("%s: enter\n", __func__);

    if (aw22xxx && gpio_is_valid(aw22xxx->reset_gpio)) {
        gpiod_set_raw_value_cansleep(gpio_to_desc(aw22xxx->reset_gpio), 0);
        msleep(1);
        gpiod_set_raw_value_cansleep(gpio_to_desc(aw22xxx->reset_gpio), 1);
        msleep(1);
    } else {
        dev_err(aw22xxx->dev, "%s:  failed\n", __func__);
    }
    return 0;
}

static int aw22xxx_hw_off(struct aw22xxx *aw22xxx)
{
    pr_info("%s: enter\n", __func__);

    if (aw22xxx && gpio_is_valid(aw22xxx->reset_gpio)) {
        gpiod_set_raw_value_cansleep(gpio_to_desc(aw22xxx->reset_gpio), 0);
        msleep(1);
    } else {
        dev_err(aw22xxx->dev, "%s:  failed\n", __func__);
    }
    return 0;
}

/*****************************************************
 *
 * check chip id
 *
 *****************************************************/
static int aw22xxx_read_chipid(struct aw22xxx *aw22xxx)
{
    int ret = -1;
    unsigned char cnt = 0;
    unsigned char reg_val = 0;
  
    aw22xxx_reg_page_cfg(aw22xxx, AW22XXX_REG_PAGE0);
    aw22xxx_sw_reset(aw22xxx);
  
    while(cnt < AW_READ_CHIPID_RETRIES) {
        ret = aw22xxx_i2c_read(aw22xxx, REG_SRST, &reg_val);
        if (ret < 0) {
            dev_err(aw22xxx->dev,
                "%s: failed to read register AW22XXX_REG_ID: %d\n",
                __func__, ret);
            return -EIO;
        }
        switch (reg_val) {
            case AW22XXX_SRSTR:
                pr_info("%s aw22xxx detected\n", __func__);
                //aw22xxx->flags |= AW22XXX_FLAG_SKIP_INTERRUPTS;
                aw22xxx_i2c_read(aw22xxx, REG_CHIPID, &reg_val);
                switch(reg_val) {
                    case AW22118_CHIPID:
                        aw22xxx->chip_type= AW22118;
                        pr_info("%s: chipid: aw22118\n", __func__);
                        break;
                    case AW22127_CHIPID:
                        aw22xxx->chip_type= AW22127;
                        pr_info("%s: chipid: aw22127\n", __func__);
                        break;
                    default:
                        pr_err("%s: unknown id=0x%02x\n", __func__, reg_val);
                        break;
                }
                return 0;
            default:
                pr_info("%s unsupported device revision (0x%x)\n",
                    __func__, reg_val );
                break;
        }
        cnt ++;

        msleep(AW_READ_CHIPID_RETRY_DELAY);
    }

    return -EINVAL;
}

/******************************************************
 *
 * sys group attribute: reg
 *
 ******************************************************/
static ssize_t aw22xxx_reg_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

    unsigned int databuf[2] = {0, 0};

    if(2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
        aw22xxx_i2c_write(aw22xxx, databuf[0], databuf[1]);
    }

    return count;
}

static ssize_t aw22xxx_reg_show(struct device *dev, struct device_attribute *attr,
                char *buf)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);
    ssize_t len = 0;
    unsigned int i = 0;
    unsigned char reg_val = 0;
    unsigned char reg_page = 0;
    aw22xxx_i2c_read(aw22xxx, REG_PAGE, &reg_page);
    for(i = 0; i < AW22XXX_REG_MAX; i ++) {
        if(!reg_page) {
            if(!(aw22xxx_reg_access[i]&REG_RD_ACCESS))
               continue;
        }
        aw22xxx_i2c_read(aw22xxx, i, &reg_val);
        len += snprintf(buf+len, PAGE_SIZE-len, "reg:0x%02x=0x%02x \n", i, reg_val);
    }
    return len;
}

static ssize_t aw22xxx_hwen_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

    unsigned int databuf[1] = {0};

    if(1 == sscanf(buf, "%x", &databuf[0])) {
        if(1 == databuf[0]) {
            aw22xxx_hw_reset(aw22xxx);
        } else {
            aw22xxx_hw_off(aw22xxx);
        }
    }

    return count;
}

static ssize_t aw22xxx_hwen_show(struct device *dev, struct device_attribute *attr,
                char *buf)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);
    ssize_t len = 0;
    len += snprintf(buf+len, PAGE_SIZE-len, "hwen=%d\n",
            gpiod_get_raw_value(gpio_to_desc(aw22xxx->reset_gpio)));

    return len;
}

static ssize_t aw22xxx_fw_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

    unsigned int databuf[1] = {0};

    if(1 == sscanf(buf, "%x", &databuf[0])) {
        aw22xxx->fw_update = databuf[0];
        if(1 == databuf[0]) {
            schedule_work(&aw22xxx->fw_work);
        }
    }

    return count;
}

static ssize_t aw22xxx_fw_show(struct device *dev, struct device_attribute *attr,
                char *buf)
{
    ssize_t len = 0;

    len += snprintf(buf+len, PAGE_SIZE-len, "firmware name = %s\n", aw22xxx_fw_name);

    return len;
}

static ssize_t aw22xxx_cfg_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

    unsigned int databuf[1] = {0};

    if(1 == sscanf(buf, "%d", &databuf[0])) {
        aw22xxx->cfg = databuf[0];
        if(aw22xxx->cfg) {
            schedule_work(&aw22xxx->cfg_work);
        }
    }

    return count;
}

static ssize_t aw22xxx_cfg_show(struct device *dev, struct device_attribute *attr,
                char *buf)
{
    ssize_t len = 0;
    unsigned int i;
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

    for(i=0; i<sizeof(aw22xxx_cfg_name)/AW22XXX_CFG_NAME_MAX; i++) {
        len += snprintf(buf+len, PAGE_SIZE-len, "cfg[%x] = %s\n", i, aw22xxx_cfg_name[i]);
    }
    len += snprintf(buf+len, PAGE_SIZE-len, "current cfg = %s\n", aw22xxx_cfg_name[aw22xxx->effect]);

    return len;
}

static ssize_t aw22xxx_effect_store(struct device* dev, struct device_attribute *attr,
                const char* buf, size_t len)
{
    unsigned int databuf[1];
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

    sscanf(buf,"%d",&databuf[0]);
    aw22xxx->effect = databuf[0];

    return len;
}

static ssize_t aw22xxx_effect_show(struct device* dev,struct device_attribute *attr, char* buf)
{
    ssize_t len = 0;
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

    len += snprintf(buf+len, PAGE_SIZE-len, "effect = 0x%02x\n", aw22xxx->effect);

    return len;
}

static ssize_t aw22xxx_imax_store(struct device* dev, struct device_attribute *attr,
                const char* buf, size_t len)
{
    unsigned int databuf[1];
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

    sscanf(buf,"%x",&databuf[0]);
    aw22xxx->imax = databuf[0];
    aw22xxx_imax_cfg(aw22xxx, aw22xxx_imax_code[aw22xxx->imax]);

    return len;
}

static ssize_t aw22xxx_imax_show(struct device* dev,struct device_attribute *attr, char* buf)
{
    ssize_t len = 0;
    unsigned int i;
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

    for(i=0; i<sizeof(aw22xxx_imax_name)/AW22XXX_IMAX_NAME_MAX; i++) {
        len += snprintf(buf+len, PAGE_SIZE-len, "imax[%x] = %s\n", i, aw22xxx_imax_name[i]);
    }
    len += snprintf(buf+len, PAGE_SIZE-len, "current id = 0x%02x, imax = %s\n",
        aw22xxx->imax, aw22xxx_imax_name[aw22xxx->imax]);

    return len;
}

static ssize_t aw22xxx_rgb_store(struct device* dev, struct device_attribute *attr,
                const char* buf, size_t len)
{
    unsigned int databuf[2];
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

    sscanf(buf,"%x %x",&databuf[0], &databuf[1]);
    aw22xxx->rgb[databuf[0]] = databuf[1];

    return len;
}

static ssize_t aw22xxx_rgb_show(struct device* dev,struct device_attribute *attr, char* buf)
{
    ssize_t len = 0;
    unsigned int i;
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

    for(i=0; i<AW22XXX_RGB_MAX; i++) {
        len += snprintf(buf+len, PAGE_SIZE-len, "rgb[%d] = 0x%06x\n", i, aw22xxx->rgb[i]);
    }
    return len;
}

static ssize_t aw22xxx_task0_store(struct device* dev, struct device_attribute *attr,
                const char* buf, size_t len)
{
    unsigned int databuf[1];
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

    sscanf(buf,"%x",&databuf[0]);
    aw22xxx->task0 = databuf[0];
    schedule_work(&aw22xxx->task_work);

    return len;
}

static ssize_t aw22xxx_task0_show(struct device* dev,struct device_attribute *attr, char* buf)
{
    ssize_t len = 0;
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

    len += snprintf(buf+len, PAGE_SIZE-len, "task0 = 0x%02x\n", aw22xxx->task0);

    return len;
}

static ssize_t aw22xxx_task1_store(struct device* dev, struct device_attribute *attr,
                const char* buf, size_t len)
{
    unsigned int databuf[1];
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

    sscanf(buf,"%x",&databuf[0]);
    aw22xxx->task1 = databuf[0];

    return len;
}

static ssize_t aw22xxx_task1_show(struct device* dev,struct device_attribute *attr, char* buf)
{
    ssize_t len = 0;
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

    len += snprintf(buf+len, PAGE_SIZE-len, "task1 = 0x%02x\n", aw22xxx->task1);

    return len;
}
static ssize_t aw22xxx_gain_store(struct device* dev, struct device_attribute *attr,
                const char* buf, size_t len)
{
    unsigned int databuf[1];
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

    sscanf(buf,"%x",&databuf[0]);

    mutex_lock(&aw22xxx->cfg_lock);

	aw22xxx_i2c_write(aw22xxx, 0xff, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x0c, 0x01);//disable AGC

	aw22xxx_i2c_write(aw22xxx, 0x05, 0xe1);
	aw22xxx_i2c_write(aw22xxx, 0x06, 0x3f);//max-gain
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x07);

	aw22xxx_i2c_write(aw22xxx, 0x05, 0xe2);//min-gain
	aw22xxx_i2c_write(aw22xxx, 0x06, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x07);

	aw22xxx_i2c_write(aw22xxx, 0x05, 0xe0);
	aw22xxx_i2c_write(aw22xxx, 0x06, databuf[0]);
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x07);

    mutex_unlock(&aw22xxx->cfg_lock);

    return len;
}

static ssize_t aw22xxx_gain_show(struct device* dev,struct device_attribute *attr, char* buf)
{
    ssize_t len = 0;
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);
	unsigned char reg_gain = 0;

	aw22xxx_i2c_read(aw22xxx, REG_GAIN, &reg_gain);

    len += snprintf(buf+len, PAGE_SIZE-len, "gain = 0x%02x\n", reg_gain);

    return len;
}

static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, aw22xxx_reg_show, aw22xxx_reg_store);
static DEVICE_ATTR(hwen, S_IWUSR | S_IRUGO, aw22xxx_hwen_show, aw22xxx_hwen_store);
static DEVICE_ATTR(fw, S_IWUSR | S_IRUGO, aw22xxx_fw_show, aw22xxx_fw_store);
static DEVICE_ATTR(cfg, S_IWUSR | S_IRUGO, aw22xxx_cfg_show, aw22xxx_cfg_store);
static DEVICE_ATTR(effect, S_IWUSR | S_IRUGO, aw22xxx_effect_show, aw22xxx_effect_store);
static DEVICE_ATTR(imax, S_IWUSR | S_IRUGO, aw22xxx_imax_show, aw22xxx_imax_store);
static DEVICE_ATTR(rgb, S_IWUSR | S_IRUGO, aw22xxx_rgb_show, aw22xxx_rgb_store);
static DEVICE_ATTR(task0, S_IWUSR | S_IRUGO, aw22xxx_task0_show, aw22xxx_task0_store);
static DEVICE_ATTR(task1, S_IWUSR | S_IRUGO, aw22xxx_task1_show, aw22xxx_task1_store);
static DEVICE_ATTR(gain, S_IWUSR | S_IRUGO, aw22xxx_gain_show, aw22xxx_gain_store);

static struct attribute *aw22xxx_attributes[] = {
    &dev_attr_reg.attr,
    &dev_attr_hwen.attr,
    &dev_attr_fw.attr,
    &dev_attr_cfg.attr,
    &dev_attr_effect.attr,
    &dev_attr_imax.attr,
    &dev_attr_rgb.attr,
    &dev_attr_task0.attr,
    &dev_attr_task1.attr,
    &dev_attr_gain.attr,
    NULL
};

static struct attribute_group aw22xxx_attribute_group = {
    .attrs = aw22xxx_attributes
};

/******************************************************
 *
 * led class dev
 *
 ******************************************************/
static void aw22xxx_default_music_gain(struct aw22xxx *aw22xxx)
{
    unsigned int i;

    for (i = 0; i < AW22XXX_MUSIC_GAIN_COUNT; i++)
        aw22xxx->music_gain[i] = aw_music_gain_table[i];
    aw22xxx->music_gain_count = AW22XXX_MUSIC_GAIN_COUNT;

    pr_info("%s: use default music gain table %d\n", __func__,
            aw22xxx->music_gain_count);
    for (i = 0; i < aw22xxx->music_gain_count; i++)
        pr_info("%s: default item %d, volume_level=%u, music_gain=0x%x\n",
                __func__, i, aw22xxx->music_gain[i].volume_level,
                aw22xxx->music_gain[i].music_gain);
}

static int aw22xxx_parse_music_gain_table(struct aw22xxx *aw22xxx,
                                          struct device_node *np)
{
    unsigned int table[AW22XXX_MUSIC_GAIN_COUNT * 2];
    int elems, count, i, ret;

    aw22xxx->music_gain = devm_kmalloc(aw22xxx->dev,
            sizeof(*aw22xxx->music_gain) * AW22XXX_MUSIC_GAIN_COUNT,
            GFP_KERNEL);
    if (!aw22xxx->music_gain) {
        dev_err(aw22xxx->dev,
                "Failed to parse music gain table, ret=%d\n", -ENOMEM);
        return -ENOMEM;
    }

    elems = of_property_count_elems_of_size(np, "aw22xxx,music_table",
                                            sizeof(u32));
    if (elems < 0) {
        pr_info("%s: no aw22xxx,music_table in dts, use default table\n",
                __func__);
        aw22xxx_default_music_gain(aw22xxx);
        return 0;
    }

    if (elems % 2) {
        dev_err(aw22xxx->dev,
                "%s: invalid aw22xxx,music_table length=%d, should be even\n",
                __func__, elems);
        aw22xxx_default_music_gain(aw22xxx);
        return 0;
    }

    count = elems / 2;
    if (count != AW22XXX_MUSIC_GAIN_COUNT) {
        dev_err(aw22xxx->dev,
                "%s: invalid aw22xxx,music_table item_num=%d, expected=%d\n",
                __func__, count, AW22XXX_MUSIC_GAIN_COUNT);
        aw22xxx_default_music_gain(aw22xxx);
        return 0;
    }

    ret = of_property_read_u32_array(np, "aw22xxx,music_table", table,
                                     elems);
    if (ret < 0) {
        dev_err(aw22xxx->dev,
                "%s: failed to read aw22xxx,music_table, use default table\n",
                __func__);
        aw22xxx_default_music_gain(aw22xxx);
        return 0;
    }

    for (i = 0; i < count; i++) {
        if (table[i * 2] >= AW22XXX_MUSIC_GAIN_COUNT) {
            dev_err(aw22xxx->dev,
                    "%s: invalid volume_level=%u at item %d, use default table\n",
                    __func__, table[i * 2], i);
            aw22xxx_default_music_gain(aw22xxx);
            return 0;
        }
        if (table[i * 2 + 1] >= 0x40) {
            dev_err(aw22xxx->dev,
                    "%s: invalid music_gain=0x%x at item %d, use default table\n",
                    __func__, table[i * 2 + 1], i);
            aw22xxx_default_music_gain(aw22xxx);
            return 0;
        }
        aw22xxx->music_gain[i].volume_level = table[i * 2];
        aw22xxx->music_gain[i].music_gain = table[i * 2 + 1];
    }
    aw22xxx->music_gain_count = count;

    pr_info("%s: use music gain table from dts %d\n", __func__, count);

    return 0;
}

static int aw22xxx_parse_multicolor(struct aw22xxx *aw22xxx,
                                    struct device_node *np)
{
    unsigned int count = 0;
    unsigned int i;
    int ret;

    ret = of_property_read_u32_array(np, "aw22xxx,multi_color_count",
                                     &count, 1);
    aw22xxx->multi_color_count = ret < 0 ? 0 : count;

    pr_info("%s: aw22xxx->multi_color_count: %d aw22xxx->imax:%d\n",
            __func__, aw22xxx->multi_color_count, aw22xxx->imax);

    if (!aw22xxx->multi_color_count)
        return 0;

    aw22xxx->multi_color_array = devm_kmalloc(aw22xxx->dev,
            sizeof(char *) * aw22xxx->multi_color_count, GFP_KERNEL);
    if (!aw22xxx->multi_color_array)
        return -ENOMEM;

    ret = of_property_read_string_array(np, "aw22xxx,multi_color_array",
                                        aw22xxx->multi_color_array,
                                        aw22xxx->multi_color_count);
    if (ret < 0) {
        pr_info("%s: not found multi_color_array, use default\n",
                __func__);
        for (i = 0; i < aw22xxx->multi_color_count; i++)
            aw22xxx->multi_color_array[i] =
                i < ARRAY_SIZE(aw22xxx_default_multicolor) ?
                aw22xxx_default_multicolor[i] :
                aw22xxx_default_multicolor[0];
    }

    for (i = 0; i < aw22xxx->multi_color_count; i++)
        pr_info("%s: multi_color_array[%d]: %s\n", __func__, i,
                aw22xxx->multi_color_array[i]);

    return 0;
}

static int aw22xxx_parse_led_cdev(struct aw22xxx *aw22xxx,
        struct device_node *np)
{
    struct device_node *temp;
    int ret = -1;

    pr_info("%s: enter\n", __func__);

    for_each_child_of_node(np, temp) {
        ret = of_property_read_string(temp, "aw22xxx,name",
            &aw22xxx->cdev.name);
        if (ret < 0) {
            dev_err(aw22xxx->dev,
                "Failure reading led name, ret = %d\n", ret);
            goto free_pdata;
        }
        ret = of_property_read_u32(temp, "aw22xxx,imax",
            &aw22xxx->imax);
        if (ret < 0) {
            dev_err(aw22xxx->dev,
                "Failure reading imax, ret = %d\n", ret);
            goto free_pdata;
        }

        aw22xxx->reset_imax =
            of_find_property(temp, "aw22xxx,reset_imax", NULL) != NULL;
        pr_info("%s: aw22xxx->reset_imax: %d aw22xxx->imax:%d\n",
                __func__, aw22xxx->reset_imax, aw22xxx->imax);

        ret = of_property_read_u32(temp, "aw22xxx,brightness",
            &aw22xxx->cdev.brightness);
        if (ret < 0) {
            dev_err(aw22xxx->dev,
                "Failure reading brightness, ret = %d\n", ret);
            goto free_pdata;
        }
        ret = of_property_read_u32(temp, "aw22xxx,max_brightness",
            &aw22xxx->cdev.max_brightness);
        if (ret < 0) {
            dev_err(aw22xxx->dev,
                "Failure reading max brightness, ret = %d\n", ret);
            goto free_pdata;
        }

        ret = aw22xxx_parse_music_gain_table(aw22xxx, temp);
        if (ret < 0)
            goto free_pdata;

        ret = aw22xxx_parse_multicolor(aw22xxx, temp);
        if (ret < 0)
            goto free_pdata;
    }

    INIT_WORK(&aw22xxx->brightness_work, aw22xxx_brightness_work);
    INIT_WORK(&aw22xxx->task_work, aw22xxx_task_work);

    aw22xxx->cdev.brightness_set = aw22xxx_set_brightness;
    ret = led_classdev_register_ext(aw22xxx->dev, &aw22xxx->cdev, NULL);
    if (ret) {
        dev_err(aw22xxx->dev,
            "unable to register led ret=%d\n", ret);
        goto free_pdata;
    }

    return 0;

free_pdata:
    return ret;
}

int aw22xxx_cover_gain(struct aw22xxx *aw22xxx, unsigned int volume_level)
{
    unsigned int i;

    if (!aw22xxx || volume_level > 0xf) {
        if (aw22xxx)
            dev_err(aw22xxx->dev, "%s(error volume level) %d\n",
                    __func__, volume_level);
        return -1;
    }

    for (i = 0; i < aw22xxx->music_gain_count; i++) {
        if (aw22xxx->music_gain[i].volume_level == volume_level)
            return aw22xxx->music_gain[i].music_gain;
    }

    return -1;
}
EXPORT_SYMBOL(aw22xxx_cover_gain);

static int aw22xxx_led_show(struct aw22xxx *aw22xxx, unsigned int effect)
{
    dev_err(aw22xxx->dev, "%s: effect = %d start\n", __func__, effect);

    aw22xxx->effect = (unsigned char)effect;
    aw22xxx->cfg = 1;
    schedule_work(&aw22xxx->cfg_work);

    dev_err(aw22xxx->dev, "%s: done\n", __func__);
    return 0;
}

static int aw22xxx_led_close(struct aw22xxx *aw22xxx)
{
    aw22xxx->effect = LIGHT_LEDS_OFF;
    aw22xxx->cfg = 1;
    schedule_work(&aw22xxx->cfg_work);

    msleep(20);

    dev_err(aw22xxx->dev, "%s: done\n", __func__);
    return 0;
}

#define AW22XXX_AUDIO_LED_EFFECTS_MIN 0x00
#define AW22XXX_AUDIO_LED_EFFECTS_MAX 0x3f
static int aw22xxx_audio_gain_led_effects(struct aw22xxx *aw22xxx,
                                          unsigned int gain)
{
    if ((gain > AW22XXX_AUDIO_LED_EFFECTS_MAX) ||
        (gain < AW22XXX_AUDIO_LED_EFFECTS_MIN)) {
        dev_err(aw22xxx->dev,
                "%s The gain is out of range(0x00~0x3f):gain:%d\n",
                __func__, gain);
        return -1;
    }

    dev_err(aw22xxx->dev, "%s gain:%d\n", __func__, gain);

    mutex_lock(&aw22xxx->cfg_lock);

    aw22xxx_i2c_write(aw22xxx, 0xff, 0x00);
    aw22xxx_i2c_write(aw22xxx, 0x0c, 0x01); /* disable AGC */

    aw22xxx_i2c_write(aw22xxx, 0x05, 0xe1);
    aw22xxx_i2c_write(aw22xxx, 0x06, 0x3f); /* max-gain */
    aw22xxx_i2c_write(aw22xxx, 0x04, 0x07);

    aw22xxx_i2c_write(aw22xxx, 0x05, 0xe2); /* min-gain */
    aw22xxx_i2c_write(aw22xxx, 0x06, 0x00);
    aw22xxx_i2c_write(aw22xxx, 0x04, 0x07);

    aw22xxx_i2c_write(aw22xxx, 0x05, 0xe0);
    aw22xxx_i2c_write(aw22xxx, 0x06, gain);
    aw22xxx_i2c_write(aw22xxx, 0x04, 0x07);

    aw22xxx->audio_gain = gain;

    mutex_unlock(&aw22xxx->cfg_lock);

    dev_err(aw22xxx->dev, "%s: successful\n", __func__);

    return 0;
}

static unsigned int aw22xxx_multicolor_shift(struct aw22xxx *aw22xxx)
{
    if (!aw22xxx->multi_color_count)
        return 0;
    if (aw22xxx->multi_color_type == 1)
        return AW22XXX_MULTI_COLOR_SHIFT;
    if (aw22xxx->multi_color_type == 2)
        return 2 * AW22XXX_MULTI_COLOR_SHIFT;
    return 0;
}

static int tran_led_cmd_cfg_2(struct aw22xxx *aw22xxx, unsigned int type,
                              unsigned int num2, unsigned int num3)
{
    unsigned int shift = aw22xxx_multicolor_shift(aw22xxx);
    unsigned int effect;
    int gain;

    pr_info("%s: aw22xxx->multi_color_type= %d multi_color_shift=%d\n",
            __func__, aw22xxx->multi_color_type, shift);

    switch (type) {
    case 0x00: /* close */
        return aw22xxx_led_close(aw22xxx);
    case 0x01: /* scheme 1 */
        effect = shift + LIGHT_SCHEME1_1 + num2;
        break;
    case 0x02:
        effect = shift + LIGHT_SCHEME2_1 + num2;
        break;
    case 0x03:
        effect = shift + LIGHT_SCHEME3_1 + num2;
        break;
    case 0x04:
        effect = shift + LIGHT_SCHEME4_1 + num2;
        break;
    case 0x0A: /* audio */
        dev_info(aw22xxx->dev, "%s audio num:%d\n", __func__, num2);
        if (num2 > 0x03) {
            dev_err(aw22xxx->dev,
                    "%s(error audio effect select) %d\n",
                    __func__, __LINE__);
            return 0;
        }
        effect = shift + LIGHT_AUDIO1_1 +
                 num2 * AW22XXX_SCHEME_STRIDE + num3;
        break;
    case 0x0B: /* factory */
        dev_info(aw22xxx->dev, "%s factory num:%d\n", __func__, num2);
        if (num2 > 0x03) {
            dev_err(aw22xxx->dev,
                    "%s(error factory effect select) %d\n",
                    __func__, __LINE__);
            return 0;
        }
        effect = LIGHT_FACTORY_RED + num2;
        break;
    case 0x0C: /* audio gain */
        if (num2 > 0x0f) {
            dev_err(aw22xxx->dev,
                    "%s(error volume level) %d\n", __func__, num2);
            return 0;
        }
        gain = aw22xxx_cover_gain(aw22xxx, num2);
        if (gain < 0) {
            dev_err(aw22xxx->dev,
                    "%s(error audio gain) %d\n", __func__, num2);
            return 0;
        }
        aw22xxx_audio_gain_led_effects(aw22xxx, gain);
        return 0;
    case 0x0D: /* performance */
        dev_info(aw22xxx->dev, "%s performance num:%d\n", __func__, num2);
        if (num2 > 0x02) {
            dev_err(aw22xxx->dev,
                    "%s(error performance effect select) %d\n",
                    __func__, __LINE__);
            return 0;
        }
        effect = LIGHT_PERFORMANCE_1 + num2;
        break;
    default:
        dev_err(aw22xxx->dev, "%s(no para) %d\n", __func__, __LINE__);
        return 0;
    }

    return aw22xxx_led_show(aw22xxx, effect);
}

static int aw22xxx_handle_tran_led_cmd(struct aw22xxx *aw22xxx,
                                       const char *cmd)
{
    unsigned int num[6];

    if (sscanf(cmd, "%x %x %x %x %x %x",
               &num[0], &num[1], &num[2], &num[3], &num[4], &num[5]) == 6) {
        dev_err(aw22xxx->dev,
                "%s 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
                __func__, num[0], num[1], num[2], num[3], num[4], num[5]);

        if (num[0] == 0x00) {
            tran_led_cmd_cfg_2(aw22xxx, num[1], num[2], num[3]);
        } else {
            dev_err(aw22xxx->dev, "%s(no para) %d\n",
                    __func__, __LINE__);
        }
    }

    return 0;
}

/*
 * tran_led_cmd is created on the i2c device, so driver_data is aw22xxx.
 * The same parser backs the TRAN_SET_CMD ioctl through the ops callback.
 */
static ssize_t store_tran_led_cmd(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf, size_t count)
{
    struct aw22xxx *aw22xxx = dev_get_drvdata(dev);

    if (!count)
        return 0;
    if (count > 0x27)
        return -EINVAL;

    aw22xxx_handle_tran_led_cmd(aw22xxx, buf);

    return count;
}

static ssize_t show_tran_led_cmd(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
    struct aw22xxx *aw22xxx = dev_get_drvdata(dev);

    return sprintf(buf, "%d\n", aw22xxx->chip_type);
}

/*
 * tran_multicolor is also on the i2c device.  The name array comes from the
 * DT property aw22xxx,multi_color_array (B85/E23/S30 on X6871).
 */
static ssize_t aw22xxx_multicolor_show(struct device *dev,
                                       struct device_attribute *attr,
                                       char *buf)
{
    struct aw22xxx *aw22xxx = dev_get_drvdata(dev);

    if (!aw22xxx)
        return -EINVAL;

    return sprintf(buf, "%d\n", aw22xxx->multi_color_type);
}

static ssize_t aw22xxx_multicolor_store(struct device *dev,
                                        struct device_attribute *attr,
                                        const char *buf, size_t count)
{
    struct aw22xxx *aw22xxx = dev_get_drvdata(dev);
    unsigned int i;

    if (!aw22xxx)
        return -EINVAL;

    for (i = 0; i < aw22xxx->multi_color_count; i++)
        pr_info("%s: multi_color_array[%d]: %s\n", __func__, i,
                aw22xxx->multi_color_array[i]);

    if (count && aw22xxx->multi_color_count &&
        aw22xxx->multi_color_array && buf[0]) {
        for (i = 0; i < aw22xxx->multi_color_count; i++) {
            if (!strncmp(aw22xxx->multi_color_array[i], buf, 3)) {
                aw22xxx->multi_color_type = i;
                pr_info("%s: aw22xxx->multi_color_type= %d\n",
                        __func__, i);
                return count;
            }
        }
    }

    pr_info("%s: default aw22xxx->multi_color_type= %d\n",
            __func__, aw22xxx->multi_color_type);

    return count;
}

static struct aw22xxx *aw22xxx_from_tran_led(struct tran_led_device *dev)
{
    if (!dev)
        return NULL;
    return dev_get_drvdata(&dev->dev);
}

static int aw22xxx_get_chip_type(struct tran_led_device *dev, int *type)
{
    struct aw22xxx *aw22xxx = aw22xxx_from_tran_led(dev);

    if (!aw22xxx || !type)
        return -EINVAL;

    if (tlc_get_debug_level() >= 3)
        pr_info("%s enter\n", __func__);

    *type = aw22xxx->chip_type;

    return 0;
}

static int aw22xxx_get_tran_led_cmd(struct tran_led_device *dev, char *buf,
                                    int len)
{
    struct aw22xxx *aw22xxx = aw22xxx_from_tran_led(dev);

    if (!aw22xxx)
        return -EINVAL;
    if (len != AW22XXX_CMD_SIZE) {
        pr_err("Invalid command data size\n");
        return -EINVAL;
    }

    memcpy(buf, aw22xxx->cmd, AW22XXX_CMD_SIZE);

    return 0;
}

static int aw22xxx_set_tran_led_cmd(struct tran_led_device *dev,
                                    const char *buf, int len)
{
    struct aw22xxx *aw22xxx = aw22xxx_from_tran_led(dev);
    const unsigned int *cmd = (const unsigned int *)buf;

    if (!aw22xxx)
        return -EINVAL;
    if (len != AW22XXX_CMD_SIZE) {
        pr_err("Invalid command data size\n");
        return -EINVAL;
    }
    if (cmd[0] != 1) {
        pr_err("Invalid command data\n");
        return -EINVAL;
    }

    memcpy(aw22xxx->cmd, buf, AW22XXX_CMD_SIZE);

    return tran_led_cmd_cfg_2(aw22xxx, cmd[2], cmd[3], cmd[4]);
}

static int aw22xxx_ops_store_tran_led_cmd(struct tran_led_device *dev,
                                          const char *cmd)
{
    struct aw22xxx *aw22xxx = aw22xxx_from_tran_led(dev);
    int ret;

    if (!aw22xxx || !cmd)
        return -EINVAL;

    ret = aw22xxx_handle_tran_led_cmd(aw22xxx, cmd);

    return ret < 0 ? ret : 0;
}

static int aw22xxx_ops_show_tran_led_cmd(struct tran_led_device *dev,
                                         char *buf)
{
    struct aw22xxx *aw22xxx = aw22xxx_from_tran_led(dev);
    char tmp[12] = {0};
    int len;

    if (!aw22xxx || !buf)
        return -EINVAL;

    len = snprintf(tmp, sizeof(tmp), "%d", aw22xxx->chip_type);
    if (len != 1)
        return -EIO;

    buf[0] = tmp[0];

    return 0;
}

static int aw22xxx_tran_multicolor_store(struct tran_led_device *dev,
                                         const char *cmd)
{
    struct aw22xxx *aw22xxx = aw22xxx_from_tran_led(dev);
    int ret;

    if (!aw22xxx || !cmd)
        return -EINVAL;

    ret = aw22xxx_multicolor_store(aw22xxx->dev, NULL, cmd, strlen(cmd));

    return ret < 0 ? ret : 0;
}

static int aw22xxx_tran_multicolor_show(struct tran_led_device *dev,
                                        char *buf)
{
    struct aw22xxx *aw22xxx = aw22xxx_from_tran_led(dev);

    if (!aw22xxx || !buf)
        return -EINVAL;

    return snprintf(buf, PAGE_SIZE, "%d\n", aw22xxx->multi_color_type);
}

static const struct tran_led_ops leds_aw22xxx_ops = {
    .get_chip_type = aw22xxx_get_chip_type,
    .get_cmd = aw22xxx_get_tran_led_cmd,
    .set_cmd = aw22xxx_set_tran_led_cmd,
    .store_tran_led_cmd = aw22xxx_ops_store_tran_led_cmd,
    .show_tran_led_cmd = aw22xxx_ops_show_tran_led_cmd,
    .store_tran_multicolor = aw22xxx_tran_multicolor_store,
    .show_tran_multicolor = aw22xxx_tran_multicolor_show,
};
/******************************************************
 *
 * i2c driver
 *
 ******************************************************/
static DEVICE_ATTR(tran_led_cmd, 0664, show_tran_led_cmd, store_tran_led_cmd);
static DEVICE_ATTR(tran_multicolor, 0664, aw22xxx_multicolor_show,
                   aw22xxx_multicolor_store);

static int aw22xxx_i2c_probe(struct i2c_client *i2c)
{
    struct aw22xxx *aw22xxx;
    struct device_node *np = i2c->dev.of_node;
    struct kobject *kobj = NULL;
    int irq_requested = 0;
    int led_registered = 0;
    int group_created = 0;
    int cmd_file = 0;
    int multicolor_file = 0;
    int irq;
    int ret;

    pr_info("%s: enter\n", __func__);

    if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
        dev_err(&i2c->dev, "check_functionality failed\n");
        return -EIO;
    }

    aw22xxx = devm_kzalloc(&i2c->dev, sizeof(struct aw22xxx), GFP_KERNEL);
    if (aw22xxx == NULL)
        return -ENOMEM;

    aw22xxx->dev = &i2c->dev;
    aw22xxx->i2c = i2c;

    i2c_set_clientdata(i2c, aw22xxx);

    mutex_init(&aw22xxx->cfg_lock);

    /* aw22xxx rst & int */
    if (np) {
        ret = aw22xxx_parse_dt(&i2c->dev, aw22xxx, np);
        if (ret) {
            dev_err(&i2c->dev,
                    "%s: failed to parse device tree node\n", __func__);
            goto err_cleanup;
        }
    } else {
        aw22xxx->reset_gpio = -1;
        aw22xxx->irq_gpio = -1;
    }

    if (gpio_is_valid(aw22xxx->reset_gpio)) {
        ret = devm_gpio_request_one(&i2c->dev, aw22xxx->reset_gpio,
            GPIOF_OUT_INIT_LOW, "aw22xxx_rst");
        if (ret) {
            dev_err(&i2c->dev, "%s: rst request failed\n", __func__);
            goto err_cleanup;
        }
    }

    if (gpio_is_valid(aw22xxx->irq_gpio)) {
        ret = devm_gpio_request_one(&i2c->dev, aw22xxx->irq_gpio,
				    GPIOF_IN, "aw22xxx_int");
        if (ret) {
            dev_err(&i2c->dev, "%s: int request failed\n", __func__);
            goto err_cleanup;
        }
    }

    /* hardware reset */
    aw22xxx_hw_reset(aw22xxx);

    /* aw22xxx chip id */
    ret = aw22xxx_read_chipid(aw22xxx);
    if (ret < 0) {
        dev_err(&i2c->dev, "%s: aw22xxx_read_chipid failed ret=%d\n",
                __func__, ret);
        goto err_cleanup;
    }

    /* aw22xxx irq */
    if (gpio_is_valid(aw22xxx->irq_gpio) &&
        !(aw22xxx->flags & AW22XXX_FLAG_SKIP_INTERRUPTS)) {
        /* register irq handler */
        aw22xxx_interrupt_setup(aw22xxx);
        irq = aw22xxx_gpio_to_irq(aw22xxx->irq_gpio);
        ret = devm_request_threaded_irq(&i2c->dev, irq,
                    NULL, aw22xxx_irq,
                    IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                    "aw22xxx", aw22xxx);
        if (ret != 0) {
            dev_err(&i2c->dev, "%s: failed to request IRQ %d: %d\n",
                    __func__, irq, ret);
            goto err_cleanup;
        }
        irq_requested = 1;
    } else {
        dev_info(&i2c->dev, "%s skipping IRQ registration\n", __func__);
        /* disable feature support if gpio was invalid */
        aw22xxx->flags |= AW22XXX_FLAG_SKIP_INTERRUPTS;
    }

    dev_set_drvdata(&i2c->dev, aw22xxx);

    ret = aw22xxx_parse_led_cdev(aw22xxx, np);
    if (ret < 0) {
        dev_err(&i2c->dev, "%s error creating led class dev\n", __func__);
        goto err_cleanup;
    }
    led_registered = 1;

    ret = sysfs_create_group(&aw22xxx->cdev.dev->kobj,
            &aw22xxx_attribute_group);
    if (ret) {
        dev_err(&i2c->dev, "led sysfs ret: %d\n", ret);
        goto err_cleanup;
    }
    group_created = 1;

    ret = device_create_file(&i2c->dev, &dev_attr_tran_led_cmd);
    if (ret) {
        dev_err(&i2c->dev, "tran_led_cmd sysfs ret: %d\n", ret);
        goto err_cleanup;
    }
    cmd_file = 1;

    ret = device_create_file(&i2c->dev, &dev_attr_tran_multicolor);
    if (ret) {
        dev_err(&i2c->dev, "tran_multicolor sysfs ret: %d\n", ret);
        goto err_cleanup;
    }
    multicolor_file = 1;

    kobj = kobject_create_and_add("aw22xxx_led", NULL);
    if (!kobj) {
        dev_err(&i2c->dev, "%s: kobject_create_and_add fail\n", __func__);
        ret = -ENOMEM;
        goto err_cleanup;
    }
    aw22xxx->led_kobj = kobj;

    aw22xxx->tran_name = "primary_led";
    aw22xxx->led_dev = tran_led_device_register(aw22xxx->tran_name,
            aw22xxx->dev, aw22xxx, &leds_aw22xxx_ops, NULL);
    if (IS_ERR(aw22xxx->led_dev)) {
        ret = PTR_ERR(aw22xxx->led_dev);
        aw22xxx->led_dev = NULL;
        if (tlc_get_debug_level() >= TLC_INFO_LEVEL)
            pr_notice("[tlc]%s : register tran led failed!\n", __func__);
        goto err_cleanup;
    }

    ret = sysfs_create_link(kobj, &i2c->dev.kobj, "led");
    if (ret < 0) {
        dev_err(&i2c->dev, "%s : sysfs_create_link failed\n", __func__);
        goto err_cleanup;
    }

    aw22xxx_fw_init(aw22xxx);
    aw22xxx->multi_color_type = 0;

    pr_info("%s probe completed successfully!\n", __func__);

    return 0;

err_cleanup:
    if (aw22xxx->led_dev)
        tran_led_device_unregister(aw22xxx->led_dev);
    if (kobj)
        kobject_put(kobj);
    if (cmd_file)
        device_remove_file(&i2c->dev, &dev_attr_tran_led_cmd);
    if (multicolor_file)
        device_remove_file(&i2c->dev, &dev_attr_tran_multicolor);
    if (group_created)
        sysfs_remove_group(&aw22xxx->cdev.dev->kobj,
                &aw22xxx_attribute_group);
    if (led_registered)
        led_classdev_unregister(&aw22xxx->cdev);
    if (irq_requested)
        devm_free_irq(&i2c->dev,
                aw22xxx_gpio_to_irq(aw22xxx->irq_gpio), aw22xxx);
    devm_kfree(&i2c->dev, aw22xxx);

    return ret;
}

static void aw22xxx_i2c_remove(struct i2c_client *i2c)
{
    struct aw22xxx *aw22xxx = i2c_get_clientdata(i2c);

    pr_info("%s: enter\n", __func__);

    device_remove_file(&i2c->dev, &dev_attr_tran_led_cmd);
    device_remove_file(&i2c->dev, &dev_attr_tran_multicolor);
    sysfs_remove_group(&aw22xxx->cdev.dev->kobj,
            &aw22xxx_attribute_group);
    led_classdev_unregister(&aw22xxx->cdev);

    if (aw22xxx->led_dev) {
        tran_led_device_unregister(aw22xxx->led_dev);
        aw22xxx->led_dev = NULL;
    }
    if (aw22xxx->led_kobj) {
        kobject_put(aw22xxx->led_kobj);
        aw22xxx->led_kobj = NULL;
    }

    if (gpio_is_valid(aw22xxx->irq_gpio) &&
        !(aw22xxx->flags & AW22XXX_FLAG_SKIP_INTERRUPTS))
        devm_free_irq(&i2c->dev,
                aw22xxx_gpio_to_irq(aw22xxx->irq_gpio), aw22xxx);

    hrtimer_cancel(&aw22xxx->fw_timer);
    cancel_work_sync(&aw22xxx->fw_work);
    cancel_work_sync(&aw22xxx->cfg_work);
    cancel_work_sync(&aw22xxx->brightness_work);
    cancel_work_sync(&aw22xxx->task_work);

    devm_kfree(&i2c->dev, aw22xxx);
}

static const struct i2c_device_id aw22xxx_i2c_id[] = {
    { AW22XXX_I2C_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, aw22xxx_i2c_id);

static struct of_device_id aw22xxx_dt_match[] = {
    { .compatible = "awinic,aw22xxx_led" },
    { },
};

static struct i2c_driver aw22xxx_i2c_driver = {
    .driver = {
        .name = AW22XXX_I2C_NAME,
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(aw22xxx_dt_match),
    },
    .probe = aw22xxx_i2c_probe,
    .remove = aw22xxx_i2c_remove,
    .id_table = aw22xxx_i2c_id,
};

static int __init aw22xxx_i2c_init(void)
{
    int ret = 0;

    pr_info("aw22xxx driver version %s\n", AW22XXX_DRIVER_VERSION);

    ret = i2c_add_driver(&aw22xxx_i2c_driver);
    if(ret){
        pr_err("fail to add aw22xxx device into i2c\n");
        return ret;
    }

    return 0;
}
late_initcall(aw22xxx_i2c_init);

static void __exit aw22xxx_i2c_exit(void)
{
    i2c_del_driver(&aw22xxx_i2c_driver);
}
module_exit(aw22xxx_i2c_exit);

MODULE_DESCRIPTION("AW22XXX LED Driver");
MODULE_LICENSE("GPL v2");
