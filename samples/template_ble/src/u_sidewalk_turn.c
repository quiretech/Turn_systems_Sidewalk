#if defined(CONFIG_TURN_APP)

#include <sid_api.h>
#include <sid_error.h>
#include <sid_pal_assert_ifc.h>
#include <app_ble_config.h>

#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include <settings_utils.h>
#include <dk_buttons_and_leds.h>
#include <sidewalk_version.h>
#include <nordic_dfu.h>
#if defined(CONFIG_BOOTLOADER_MCUBOOT)
#include <zephyr/dfu/mcuboot.h>
#endif
#if defined(CONFIG_GPIO)
#include <state_notifier_gpio_backend.h>
#endif
#if defined(CONFIG_LOG)
#include <state_notifier_log_backend.h>
#endif

#include <pal_init.h>
#include <buttons.h>
#include <application_thread.h>
#include <sidewalk_callbacks.h>

#if defined(CONFIG_NORDIC_QSPI_NOR)
#include <zephyr/device.h>
#include <zephyr/pm/device.h>


#define EXTERNAL_FLASH DT_CHOSEN(nordic_pm_ext_flash)
#endif

#endif //#if defined(CONFIG_TURN_APP)
