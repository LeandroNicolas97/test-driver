#include <zephyr/drivers/watchdog.h>
#include <zephyr/sys/printk.h>
#include "watchdog.h"

#define WDT_NODE       DT_INST(0, atmel_sam0_watchdog)
#define WDT_LABEL      DT_LABEL(WDT_NODE)
#define WDT_MAX_WINDOW 16000U

const struct device *wdt_dev;
static struct wdt_timeout_cfg cfg_wdt;

int watchdog_init(void)
{
    uint8_t err;

    wdt_dev = DEVICE_DT_GET(WDT_NODE);

    if (!wdt_dev) {
        printk("Cannot get WDT device\n");
        return -1;
    }
    cfg_wdt.flags = WDT_FLAG_RESET_SOC;
    cfg_wdt.callback = NULL;
    cfg_wdt.window.min = 0U;
    cfg_wdt.window.max = WDT_MAX_WINDOW;
    err = wdt_install_timeout(wdt_dev, &cfg_wdt);
    if (err < 0) {
        printk("Watchdog install error\n");
        return -1;
    }
    err = wdt_setup(wdt_dev, 0);
    if (err < 0) {
        printk("Watchdog setup error\n");
    }
    return 1;
}

void watchdog_reset(void)
{
    /* printk("Watchdog reset\n"); */
    wdt_feed(wdt_dev, 0);
}

void watchdog_disable(void)
{
    wdt_disable(wdt_dev);
}
