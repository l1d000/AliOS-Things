/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#include <k_api.h>
#include <aos/aos.h>
#include "soc_init.h"
#include "isd9160.h"

#define VERSION_REC_PB          "v1.01"
#define VERSION_VR              "v2.01"
#define AUDIO_FILE_SDCARD       "audio.data"
#define FIRMWARE_FILE_SDCARD    "isd9160_fw.bin"

#define VRCMD_WAKEUP_LED        GPIO_ALS_LED
#define VRCMD_INDICATOR_LED     GPIO_GS_LED

typedef struct {
    int last_time;
    int interval;
} VRIndicator;

static int key_flag = 0;
static int stop_flag = 0;

static VRIndicator g_vri_data;

static const char *g_vrcmd_list[] = {
    "Xiao Te Xiao Te",
    "Photograph",
    "Scanning QR",
    "Playback music",
    "Pause music",
    "Previous music",
    "Next music",
    "Turn up volume",
    "Turn down volume",
    "Current temperature",
    "Current humidity",
    "Turn on light",
    "Turn off light",
};

static void isd9160_loop(void *arg)
{
    int ret = 0;

    isd9160_loop_once();
    if (key_flag == 1) {
        stop_flag = 0;
        printf("handle_record begin, press the same key again to stop it\n");
        ret = handle_record(AUDIO_FILE_SDCARD, &stop_flag);
        printf("handle_record return %d\n", ret);
        key_flag = 0;
    } else if (key_flag == 2) {
        printf("handle_playback begin\n");
        ret = handle_playback(AUDIO_FILE_SDCARD);
        printf("handle_playback return %d\n", ret);
        key_flag = 0;
    } else if (key_flag == 3) {
        printf("handle_upgrade begin\n");
        ret = handle_upgrade(FIRMWARE_FILE_SDCARD);
        printf("handle_upgrade return %d\n", ret);
        key_flag = 0;
    }
    aos_post_delayed_action(1000, isd9160_loop, NULL);
}

int handing_shake()
{
    static sys_time_t last_time = 0;
    sys_time_t        now_time  = 0;
    int               ret       = 0;

    now_time = krhino_sys_time_get();
    if (now_time - last_time < 200) {
        ret = 1;
    }
    last_time = now_time;

    return ret;
}

void key1_handle(void *arg)
{
    if (handing_shake())
        return;
    if (key_flag != 0 && stop_flag == 0) {
        stop_flag = 1;
    }
    key_flag = 1;
}

void key2_handle(void *arg)
{
    if (handing_shake())
        return;
    key_flag = 2;
}

void key3_handle(void *arg)
{
    if (handing_shake())
        return;
    key_flag = 3;
}

static void wakeup_delayed_action(void *arg)
{
    hal_gpio_output_high(&brd_gpio_table[VRCMD_WAKEUP_LED]);
}

static void indicator_delayed_action(void *arg)
{
    hal_gpio_output_toggle(&brd_gpio_table[VRCMD_INDICATOR_LED]);
    if (--g_vri_data.last_time > 0) {
        aos_post_delayed_action(g_vri_data.interval, indicator_delayed_action, NULL);
    } else {
        aos_cancel_delayed_action(g_vri_data.interval, indicator_delayed_action, NULL);
    }
}

void isd9160_vr_handle(const char *sw_ver, int cmd_id)
{
    int vrcmd_num = sizeof(g_vrcmd_list) / sizeof(const char *);
    int ret = 0;

    if (cmd_id < 0 || cmd_id >= vrcmd_num) {
        printf("Received unknow VRCMD %d.", cmd_id);
        return;
    }
    if (cmd_id == 0) {
        hal_gpio_output_low(&brd_gpio_table[VRCMD_WAKEUP_LED]);
        ret = aos_post_delayed_action(10000, wakeup_delayed_action, NULL);
    } else {
        g_vri_data.last_time = cmd_id * 2 - 1;
        g_vri_data.interval = 2000 / g_vri_data.last_time;
        aos_cancel_delayed_action(10000, wakeup_delayed_action, NULL);
        aos_post_delayed_action(2000, wakeup_delayed_action, NULL);
        hal_gpio_output_low(&brd_gpio_table[VRCMD_INDICATOR_LED]);
        aos_post_delayed_action(g_vri_data.interval, indicator_delayed_action, NULL);
    }
    printf("Received VRCMD: %s\n", g_vrcmd_list[cmd_id]);
}

void isd9160_bootup(const char *sw_ver)
{
    int ret = 0;

    if (!strcmp(sw_ver, VERSION_REC_PB)) {
        ret |= hal_gpio_enable_irq(&brd_gpio_table[GPIO_KEY_1],
                                IRQ_TRIGGER_RISING_EDGE, key1_handle, NULL);
        ret |= hal_gpio_enable_irq(&brd_gpio_table[GPIO_KEY_2],
                                IRQ_TRIGGER_RISING_EDGE, key2_handle, NULL);
        if (ret != 0) {
            printf("hal_gpio_enable_irq key return failed.\n");
        }
        printf("Ready for record and playback.\n");
    } else if (!strcmp(sw_ver, VERSION_VR)) {
        ret |= hal_gpio_disable_irq(&brd_gpio_table[GPIO_KEY_1]);
        ret |= hal_gpio_disable_irq(&brd_gpio_table[GPIO_KEY_2]);
        if (ret != 0) {
            printf("hal_gpio_enable_irq key return failed.\n");
        }
        register_vrcmd_callback(isd9160_vr_handle);
        printf("Ready for voice recognition.\n");
    } else {
        printf("Unknow version %s\n", sw_ver);
    }
}

int application_start(int argc, char *argv[])
{
    int ret = 0;

    LOG("application started.");
    ret = fatfs_register();
    if (ret != 0) {
        KIDS_A10_PRT("fatfs_register return failed.\n");
    }
    isd9160_i2c_init();
    audio_init();
    ret = hal_gpio_enable_irq(&brd_gpio_table[GPIO_KEY_3],
                                IRQ_TRIGGER_RISING_EDGE, key3_handle, NULL);
    if (ret != 0) {
        printf("hal_gpio_enable_irq key return failed.\n");
    }
    register_swver_callback(isd9160_bootup);
    aos_post_delayed_action(1000, isd9160_loop, NULL);
    aos_loop_run();

    return 0;
}
