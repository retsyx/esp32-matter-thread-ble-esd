/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>
#include <setup_payload/OnboardingCodesUtil.h>
#include <bsp/esp-bsp.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_matter.h>
#include <esp_matter_ota.h>
#include <esp_openthread_netif_glue.h>
#include <esp_pm.h>
#include <esp_timer.h>
#include <esp_vfs_eventfd.h>
#include <nvs_flash.h>
#include <driver/i2c.h>
#include <app_openthread_config.h>
#include <app_reset.h>
#include <common_macros.h>

#include <button_gpio.h>

#include "sparkfun_bma400.h"

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

#define ACC_INTERRUPT_PIN 5
#define I2C_MASTER_SDA_IO 6
#define I2C_MASTER_SCL_IO 7

#define I2C_MASTER_FREQ_HZ 100000   /*!< I2C master clock frequency */

typedef struct {
    uint16_t sensor_endpoint_id;
    uint16_t battery_endpoint_id;
} endpoint_data_t;

static const char *TAG = "app_main";

static endpoint_data_t endpoint_data;
static esp_timer_handle_t battery_timer_handle;

static esp_err_t init_i2c(i2c_port_t port = I2C_NUM_0)
{
    ESP_LOGI(TAG, "Initializing I2C on port %d", port);

    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_MASTER_FREQ_HZ,
        },
    };

    esp_err_t err = i2c_param_config(port, &i2c_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure I2C driver, err: %d", err);
        return err;
    }

    err = i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install driver, err: %d", err);
        return err;
    }

    return ESP_OK;
}

static esp_err_t deinit_i2c(i2c_port_t port = I2C_NUM_0) {
    ESP_LOGI(TAG, "Denitializing I2C on port %d", port);
    return i2c_driver_delete(port);
}

static esp_err_t factory_reset_button_register()
{
    button_handle_t push_button;
    esp_err_t err = bsp_iot_button_create(&push_button, NULL, BSP_BUTTON_NUM);
    VerifyOrReturnError(err == ESP_OK, err);
    return app_reset_button_register(push_button);
}

#define BATTERY_ADC_PIN ADC_CHANNEL_1

static adc_oneshot_unit_handle_t adc1_handle;

static esp_err_t init_battery_adc()
{
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, BATTERY_ADC_PIN, &config));

    ESP_LOGI(TAG, "Battery ADC inited");

    return ESP_OK;
}

static esp_err_t deinit_battery_adc()
{
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));

    ESP_LOGI(TAG, "Battery ADC deinited");

    return ESP_OK;
}

static int get_battery_percentage()
{
    // See https://docs.espressif.com/projects/esp-idf/en/release-v6.0/esp32/api-reference/peripherals/adc/index.html#adc-attenuation
    const int vref = 1100;
    const int k1 = 4; // for 12dB attenuation
    const int dmax = 4095;
    int dout;
    // init/deinit ADC for every read because there is a bug where ADC reads return bogus data
    // after light sleep otherwise.
    init_battery_adc();
    esp_err_t err = adc_oneshot_read(adc1_handle, BATTERY_ADC_PIN, &dout);
    deinit_battery_adc();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error %d reading battery ADC", err);
        return 0;
    }
    int mv = dout * k1 * vref / dmax;

    ESP_LOGI(TAG, "Battery ADC readout %d mV (battery level %d mV)", mv, 2*mv);

    // Max voltage is defined as 4.2V = 4200mV
    // Min voltage is defined as 3.5V = 3500mV
    // We are measuring voltage on the midpoint of a voltage divider, so divide by 2.
    const int min_mv = 3500 / 2;
    const int max_mv = 4200 / 2;

    if (mv < min_mv) {
        mv = min_mv;
    } else if (mv > max_mv) {
        mv = max_mv;
    }

    int p = 100 * (mv - min_mv) / (max_mv - min_mv);

    ESP_LOGI(TAG, "Battery percentage %d%%", p);

    return p;
}

static void battery_cb(void *arg)
{
    endpoint_data_t e = *(endpoint_data_t *)arg;

    // Read battery level
    int p = get_battery_percentage();
    bool is_low = false;
    if (p < 20) {
        is_low = true;
    }
    auto call = [e, p, is_low]() {
        ESP_LOGI(TAG, "Setting battery level: %d is_low: %d", p, is_low);
        // Update battery level and low state
        attribute_t * attribute = attribute::get(e.battery_endpoint_id,
                                   PowerSource::Id,
                                   PowerSource::Attributes::BatPercentRemaining::Id);
        esp_matter_attr_val_t val = esp_matter_invalid(NULL);
        attribute::get_val(attribute, &val);
        val.val.i = p * 2; // Matter range is 0-200
        attribute::update(e.battery_endpoint_id, PowerSource::Id, PowerSource::Attributes::BatPercentRemaining::Id, &val);

        attribute = attribute::get(e.battery_endpoint_id,
                                   PowerSource::Id,
                                   PowerSource::Attributes::BatReplacementNeeded::Id);
        attribute::get_val(attribute, &val);
        val.val.b = is_low;
        attribute::update(e.battery_endpoint_id, PowerSource::Id, PowerSource::Attributes::BatReplacementNeeded::Id, &val);
    };

    // schedule the attribute update so that we can report it from matter thread
    chip::DeviceLayer::SystemLayer().ScheduleLambda([call] { call(); });

    // Restart the timer
    uint64_t period_us = 24UL * 3600UL * 1000000UL;
    esp_err_t err = esp_timer_start_once(battery_timer_handle, period_us);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start timer, err: %d", err);
    }
}

#define CHECK(A) do { int x; if ((x = (A))) ESP_LOGI(TAG, "Error %d "#A, x); } while (false)

static esp_err_t init_acc()
{
    init_i2c();

    BMA400 accelerometer;

    while (accelerometer.beginI2C() != BMA400_OK) {
        ESP_LOGE(TAG, "BMA400 not connected, check wiring and I2C address!");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "BMA400 initialized");

#if 0
    accelerometer.getSensorData();
    ESP_LOGI(TAG, "acc %.2f %.2f %.2f",
        accelerometer.data.accelX,
        accelerometer.data.accelY,
        accelerometer.data.accelZ);
#endif

    // Configure wakeup interrupt
    bma400_wakeup_conf wakeupConfig = {
        .wakeup_ref_update = BMA400_UPDATE_ONE_TIME,
        .sample_count = BMA400_SAMPLE_COUNT_8,
        .wakeup_axes_en = BMA400_AXIS_XYZ_EN,
        .int_wkup_threshold = 4,
        .int_wkup_ref_x = 255,
        .int_wkup_ref_y = 255,
        .int_wkup_ref_z = 255,
        .int_chan = BMA400_INT_CHANNEL_1
    };
    CHECK(accelerometer.setWakeupInterrupt(&wakeupConfig));

    // Configure the INT1 pin to push/pull mode, active high
    CHECK(accelerometer.setInterruptPinMode(BMA400_INT_CHANNEL_1, BMA400_INT_PUSH_PULL_ACTIVE_1));

    // Enable auto wakeup interrupt condition
    CHECK(accelerometer.enableInterrupt(BMA400_AUTO_WAKEUP_EN, true));

    // Set up automatic transition to low power mode
    bma400_auto_lp_conf autoLPConfig = {
        .auto_low_power_trigger = BMA400_AUTO_LP_TIME_RESET_EN,
        .auto_lp_timeout_threshold = 400  // 1 second timeout
    };
    CHECK(accelerometer.setAutoLowPower(&autoLPConfig));

    // Configure generic interrupt to reset auto low power timer
    bma400_gen_int_conf config = {
        .gen_int_thres = 5,
        .gen_int_dur = 1,
        .axes_sel = BMA400_AXIS_XYZ_EN,
        .data_src = BMA400_DATA_SRC_ACCEL_FILT_2,
        .criterion_sel = BMA400_ACTIVITY_INT,
        .evaluate_axes = BMA400_ANY_AXES_INT,
        .ref_update = BMA400_UPDATE_EVERY_TIME,
        .hysteresis = BMA400_HYST_48_MG,
        .int_thres_ref_x = 0,
        .int_thres_ref_y = 0,
        .int_thres_ref_z = 512,
        .int_chan = BMA400_UNMAP_INT_PIN
    };
    CHECK(accelerometer.setGeneric2Interrupt(&config));

    // Enable generic 2 interrupt condition
    CHECK(accelerometer.enableInterrupt(BMA400_GEN2_INT_EN, true));

    CHECK(accelerometer.setMode(BMA400_MODE_LOW_POWER));

    deinit_i2c();

    return ESP_OK;
}

static void acc_cb(void *arg, void *data)
{
    endpoint_data_t e = *(endpoint_data_t *)data;

    // We got an interrupt from the accelerometer, generate an update
    ESP_LOGI(TAG, "Motion detected!");

    auto call = [e](bool value) {
        ESP_LOGI(TAG, "Setting occupancy value: %d", value);
        attribute_t * attribute = attribute::get(e.sensor_endpoint_id,
                                                 OccupancySensing::Id,
                                                 OccupancySensing::Attributes::Occupancy::Id);

        esp_matter_attr_val_t val = esp_matter_invalid(NULL);
        attribute::get_val(attribute, &val);
        val.val.b = value;
        attribute::update(e.sensor_endpoint_id, OccupancySensing::Id, OccupancySensing::Attributes::Occupancy::Id, &val);
    };

    // schedule the attribute update so that we can report it from matter thread
    chip::DeviceLayer::SystemLayer().ScheduleLambda([call] { call(true); });

    vTaskDelay(pdMS_TO_TICKS(5000));

    chip::DeviceLayer::SystemLayer().ScheduleLambda([call] { call(false); });

    init_acc();
}

static esp_err_t init_battery_timer(endpoint_data_t * endpoint_data)
{
    esp_timer_create_args_t args = {
        .callback = battery_cb,
        .arg = endpoint_data,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "battery",
        .skip_unhandled_events = true,
    };
    return esp_timer_create(&args, &battery_timer_handle);
}

static esp_err_t init_gpio_button(endpoint_data_t * endpoint_data)
{
    button_handle_t btn_handle;
    // Setup the GPIO 'button' to wake us up when the accelerometer pulls the pin high
    button_config_t btn_conf = {0};

    button_gpio_config_t gpio_conf {
        .gpio_num = ACC_INTERRUPT_PIN,
        .active_level = 1,
        .enable_power_save = true,
        .disable_pull = true,
    };

    esp_err_t err = iot_button_new_gpio_device(&btn_conf, &gpio_conf, &btn_handle);
    VerifyOrReturnError(err == ESP_OK, err);

    err = iot_button_register_cb(btn_handle, BUTTON_PRESS_DOWN, NULL, acc_cb, endpoint_data);
    VerifyOrReturnError(err == ESP_OK, err);

    ESP_LOGI(TAG, "Created GPIO button");
    return err;
}

static void open_commissioning_window_if_necessary()
{
    VerifyOrReturn(chip::Server::GetInstance().GetFabricTable().FabricCount() == 0);

    chip::CommissioningWindowManager & commissionMgr = chip::Server::GetInstance().GetCommissioningWindowManager();
    VerifyOrReturn(commissionMgr.IsCommissioningWindowOpen() == false);

    // After removing last fabric, this example does not remove the Wi-Fi credentials
    // and still has IP connectivity so, only advertising on DNS-SD.
    CHIP_ERROR err = commissionMgr.OpenBasicCommissioningWindow(chip::System::Clock::Seconds16(300),
                                    chip::CommissioningWindowAdvertisement::kAllSupported);
    if (err != CHIP_NO_ERROR)
    {
        ESP_LOGE(TAG, "Failed to open commissioning window, err:%" CHIP_ERROR_FORMAT, err.Format());
    }
}

static void enable_sleep()
{
    esp_pm_config_t pm_config = {
        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .light_sleep_enable = true
    };
    esp_pm_configure(&pm_config);
}

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(TAG, "Commissioning window opened");
        PrintOnboardingCodes(chip::RendezvousInformationFlags(chip::RendezvousInformationFlag::kOnNetwork));
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(TAG, "Commissioning window closed");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        enable_sleep();
        break;

    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGI(TAG, "Commissioning failed, fail safe timer expired");
        break;

    case chip::DeviceLayer::DeviceEventType::kThreadConnectivityChange:
        ESP_LOGI(TAG, "Thread connectivity changed %d", event->ThreadConnectivityChange.Result);
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
        ESP_LOGI(TAG, "Fabric removed successfully");
        open_commissioning_window_if_necessary();
        break;

    case chip::DeviceLayer::DeviceEventType::kBLEDeinitialized:
        ESP_LOGI(TAG, "BLE deinitialized and memory reclaimed");
        break;

    case chip::DeviceLayer::DeviceEventType::kSecureSessionEstablished:
        ESP_LOGI(TAG, "Established secure session");
        break;

    default:
        ESP_LOGI(TAG, "Unhandled event %d", event->Type);
        break;
    }
}

// This callback is invoked when clients interact with the Identify Cluster.
// In the callback implementation, an endpoint can identify itself. (e.g., by flashing an LED or light).
static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                       uint8_t effect_variant, void *priv_data)
{
    ESP_LOGI(TAG, "Identification callback: type: %u, effect: %u, variant: %u", type, effect_id, effect_variant);
    return ESP_OK;
}

// This callback is called for every attribute update. The callback implementation shall
// handle the desired attributes and return an appropriate error code. If the attribute
// is not of your interest, please do not return an error code and strictly return ESP_OK.
static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    // Since this is just a sensor and we don't expect any writes on our temperature sensor,
    // so, return success.
    return ESP_OK;
}

extern "C" void app_main()
{
    esp_err_t err;

    /* Initialize the ESP NVS layer */
    nvs_flash_init();

    /* Initialize push button on the dev-kit to reset the device */
    err = factory_reset_button_register();
    ABORT_APP_ON_FAILURE(ESP_OK == err, ESP_LOGE(TAG, "Failed to initialize reset button, err: %d", err));

    /* Create a Matter node and add the mandatory Root Node device type on endpoint 0 */
    node::config_t root_config;
    node_t *root = node::create(&root_config, app_attribute_update_cb, app_identification_cb);
    ABORT_APP_ON_FAILURE(root != nullptr, ESP_LOGE(TAG, "Failed to create Matter root node"));

    // Add the occupancy sensor
    occupancy_sensor::config_t occupancy_sensor_config;
    occupancy_sensor_config.occupancy_sensing.feature_flags = cluster::occupancy_sensing::feature::passive_infrared::get_id();
    occupancy_sensor_config.occupancy_sensing.occupancy_sensor_type =
        chip::to_underlying(OccupancySensing::OccupancySensorTypeEnum::kPir);
    occupancy_sensor_config.occupancy_sensing.occupancy_sensor_type_bitmap =
        chip::to_underlying(OccupancySensing::OccupancySensorTypeBitmap::kPir);

    endpoint_t * occupancy_sensor_ep = occupancy_sensor::create(root, &occupancy_sensor_config, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(occupancy_sensor_ep != nullptr, ESP_LOGE(TAG, "Failed to create occupancy_sensor endpoint"));
    endpoint_data.sensor_endpoint_id = endpoint::get_id(occupancy_sensor_ep);

    // Add the battery reporting device
    power_source::config_t power_source_config;
    power_source_config.power_source.feature_flags = cluster::power_source::feature::battery::get_id();
    endpoint_t *power_source_ep = power_source::create(root, &power_source_config, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(power_source_ep != nullptr, ESP_LOGE(TAG, "Failed to create power_source endpoint"));
    esp_matter::cluster_t *cluster = esp_matter::cluster::get(power_source_ep, chip::app::Clusters::PowerSource::Id);
    cluster::power_source::attribute::create_bat_percent_remaining(cluster, 200, 0, 200);

    endpoint_data.battery_endpoint_id = endpoint::get_id(power_source_ep);

    err = init_gpio_button(&endpoint_data);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to initialize GPIO button"));

    // init accelerometer
    err = init_acc();
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to initialize accelerometer"));

    err = init_battery_timer(&endpoint_data);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to initialize battery timer"));

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    /* Set OpenThread platform config */
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&config);
#endif

    /* Matter start */
    err = esp_matter::start(app_event_cb);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to start Matter, err:%d", err));

    battery_cb(&endpoint_data);

    if (chip::Server::GetInstance().GetFabricTable().FabricCount() != 0) {
        enable_sleep();
    }
}
