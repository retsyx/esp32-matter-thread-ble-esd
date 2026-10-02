#include "sdkconfig.h"

#if CONFIG_APP_POWER_DIAGNOSTICS

#include "power_diagnostics.h"

#include <cstdio>
#include <esp_attr.h>
#include <esp_log.h>
#include <esp_pm.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <platform/CHIPDeviceConfig.h>

#if CONFIG_BT_ENABLED
#include <esp_bt.h>
#endif

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <esp_ieee802154.h>
#include <esp_openthread.h>
#include <esp_openthread_lock.h>
#include <openthread/link.h>
#include <openthread/thread.h>
#endif

static const char *TAG = "power_diag";
static gpio_num_t s_motion_pin;
static portMUX_TYPE s_sleep_stats_lock = portMUX_INITIALIZER_UNLOCKED;
static uint64_t s_sleep_call_us;
static uint64_t s_longest_sleep_call_us;
static uint32_t s_sleep_calls;
static int64_t s_monitor_start_us;

static esp_err_t IRAM_ATTR record_sleep_duration(int64_t duration_us, void *)
{
    // Called from the idle task's critical section. Never log or block here.
    if (duration_us > 0) {
        portENTER_CRITICAL_ISR(&s_sleep_stats_lock);
        s_sleep_call_us += duration_us;
        ++s_sleep_calls;
        if (static_cast<uint64_t>(duration_us) > s_longest_sleep_call_us) {
            s_longest_sleep_call_us = duration_us;
        }
        portEXIT_CRITICAL_ISR(&s_sleep_stats_lock);
    }
    return ESP_OK;
}

static void stop_sleep_measurement()
{
    esp_pm_sleep_cbs_register_config_t callbacks = {};
    callbacks.exit_cb = record_sleep_duration;
    esp_pm_light_sleep_unregister_cbs(&callbacks);
}

static void dump_sleep_duration()
{
    const int64_t elapsed_us = esp_timer_get_time() - s_monitor_start_us;
    portENTER_CRITICAL(&s_sleep_stats_lock);
    const uint64_t sleep_call_us = s_sleep_call_us;
    const uint64_t longest_us = s_longest_sleep_call_us;
    const uint32_t calls = s_sleep_calls;
    portEXIT_CRITICAL(&s_sleep_stats_lock);
    const uint64_t permille = elapsed_us > 0 ? sleep_call_us * 1000 / elapsed_us : 0;
    // IDF measures the entire sleep call, including entry/exit and rejected attempts.
    ESP_LOGI(TAG, "Sleep call duration: %llu/%lld us (%llu.%llu%%), calls=%lu longest_us=%llu",
             static_cast<unsigned long long>(sleep_call_us), static_cast<long long>(elapsed_us),
             static_cast<unsigned long long>(permille / 10),
             static_cast<unsigned long long>(permille % 10),
             static_cast<unsigned long>(calls), static_cast<unsigned long long>(longest_us));
}

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
static const char *radio_state_name(esp_ieee802154_state_t state)
{
    switch (state) {
    case ESP_IEEE802154_RADIO_DISABLE: return "disabled";
    case ESP_IEEE802154_RADIO_IDLE: return "idle";
    case ESP_IEEE802154_RADIO_SLEEP: return "sleep";
    case ESP_IEEE802154_RADIO_RECEIVE: return "receive";
    case ESP_IEEE802154_RADIO_TRANSMIT: return "transmit";
    default: return "unknown";
    }
}

static void dump_thread_state()
{
    if (!esp_openthread_lock_acquire(pdMS_TO_TICKS(100))) {
        ESP_LOGW(TAG, "Could not acquire OpenThread lock for snapshot");
        return;
    }
    otInstance *instance = esp_openthread_get_instance();
    if (instance == nullptr) {
        esp_openthread_lock_release();
        ESP_LOGW(TAG, "OpenThread instance unavailable");
        return;
    }
    const otDeviceRole role = otThreadGetDeviceRole(instance);
    const otLinkModeConfig mode = otThreadGetLinkMode(instance);
    const uint32_t poll_ms = otLinkGetPollPeriod(instance);
    const esp_ieee802154_state_t radio = esp_ieee802154_get_state();
    esp_openthread_lock_release();

    // Print outside the OpenThread lock so serial output cannot stall the stack.
    ESP_LOGI(TAG, "Thread role=%s rx_on_idle=%d full_device=%d poll_ms=%lu",
             otThreadDeviceRoleToString(role), mode.mRxOnWhenIdle, mode.mDeviceType,
             static_cast<unsigned long>(poll_ms));
    ESP_LOGI(TAG, "802.15.4 radio=%s", radio_state_name(radio));
}
#endif

static void diagnostic_task(void *)
{
    esp_pm_lock_handle_t console_lock = nullptr;
    if (esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "power_diag", &console_lock) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create diagnostic console lock");
        stop_sleep_measurement();
        vTaskDelete(nullptr);
        return;
    }
    for (unsigned sample = 1; sample <= 3; ++sample) {
        // This task blocks between samples; it does not poll during idle.
        vTaskDelay(pdMS_TO_TICKS(60000));
        // Light sleep disconnects native USB. Allow the host to reconnect for output.
        esp_pm_lock_acquire(console_lock);
        vTaskDelay(pdMS_TO_TICKS(2000));
        ESP_LOGI(TAG, "Power snapshot %u/3, uptime=%lld us", sample, esp_timer_get_time());

        esp_pm_config_t config = {};
        if (esp_pm_get_configuration(&config) == ESP_OK) {
            ESP_LOGI(TAG, "PM light_sleep=%d min_mhz=%d max_mhz=%d motion_gpio_level=%d",
                     config.light_sleep_enable, config.min_freq_mhz, config.max_freq_mhz,
                     gpio_get_level(s_motion_pin));
        }
#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
        dump_thread_state();
#endif
#if CONFIG_BT_ENABLED
        ESP_LOGI(TAG, "BLE controller enabled=%d",
                 esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED);
#endif
        esp_pm_dump_locks(stdout);
        dump_sleep_duration();
        esp_timer_dump(stdout);
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_pm_lock_release(console_lock);
    }
    ESP_LOGI(TAG, "Power diagnostics complete; stopping diagnostic task");
    stop_sleep_measurement();
    esp_pm_lock_delete(console_lock);
    vTaskDelete(nullptr);
}

void start_power_diagnostics(gpio_num_t motion_pin)
{
    s_motion_pin = motion_pin;
    s_monitor_start_us = esp_timer_get_time();
    esp_pm_sleep_cbs_register_config_t callbacks = {};
    callbacks.exit_cb = record_sleep_duration;
    if (esp_pm_light_sleep_register_cbs(&callbacks) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register sleep duration callback");
        return;
    }
    if (xTaskCreate(diagnostic_task, "power_diag", 4096, nullptr, 1, nullptr) != pdPASS) {
        stop_sleep_measurement();
        ESP_LOGE(TAG, "Failed to create power diagnostic task");
    }
}

#endif // CONFIG_APP_POWER_DIAGNOSTICS
