/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <esp_matter.h>
#include <esp_matter_cluster.h>
#include <esp_matter_endpoint.h>
#include <esp_matter_console.h>
#include <esp_matter_ota.h>

#include <common_macros.h>
#include <app_priv.h>
#include <app_reset.h>
#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#endif

#include <app/server/CommissioningWindowManager.h>
#include <app/clusters/diagnostic-logs-server/diagnostic-logs-server.h>
#include <diagnostic-logs-provider-delegate-impl.h>
#include <app/server/Server.h>

static const char *TAG = "app_main";
uint16_t blinds_endpoint_id = 0;
uint16_t blinds_top_endpoint_id = 0;

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

constexpr auto k_timeout_seconds = 300;

#if CONFIG_ENABLE_ENCRYPTED_OTA
extern const char decryption_key_start[] asm("_binary_esp_image_encryption_key_pem_start");
extern const char decryption_key_end[] asm("_binary_esp_image_encryption_key_pem_end");

static const char *s_decryption_key = decryption_key_start;
static const uint16_t s_decryption_key_len = decryption_key_end - decryption_key_start;
#endif // CONFIG_ENABLE_ENCRYPTED_OTA

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
        ESP_LOGI(TAG, "Interface IP Address changed");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        break;

    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGI(TAG, "Commissioning failed, fail safe timer expired");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
        ESP_LOGI(TAG, "Commissioning session started");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
        ESP_LOGI(TAG, "Commissioning session stopped");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(TAG, "Commissioning window opened");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(TAG, "Commissioning window closed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
        {
            ESP_LOGI(TAG, "Fabric removed successfully");
            if (chip::Server::GetInstance().GetFabricTable().FabricCount() == 0)
            {
                chip::CommissioningWindowManager & commissionMgr = chip::Server::GetInstance().GetCommissioningWindowManager();
                constexpr auto kTimeoutSeconds = chip::System::Clock::Seconds16(k_timeout_seconds);
                if (!commissionMgr.IsCommissioningWindowOpen())
                {
                    /* After removing last fabric, this example does not remove the Wi-Fi credentials
                     * and still has IP connectivity so, only advertising on DNS-SD.
                     */
                    CHIP_ERROR err = commissionMgr.OpenBasicCommissioningWindow(kTimeoutSeconds,
                                                    chip::CommissioningWindowAdvertisement::kDnssdOnly);
                    if (err != CHIP_NO_ERROR)
                    {
                        ESP_LOGE(TAG, "Failed to open commissioning window, err:%" CHIP_ERROR_FORMAT, err.Format());
                    }
                }
            }
        break;
        }

    case chip::DeviceLayer::DeviceEventType::kFabricWillBeRemoved:
        ESP_LOGI(TAG, "Fabric will be removed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricUpdated:
        ESP_LOGI(TAG, "Fabric is updated");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricCommitted:
        ESP_LOGI(TAG, "Fabric is committed");
        break;

    case chip::DeviceLayer::DeviceEventType::kBLEDeinitialized:
        ESP_LOGI(TAG, "BLE deinitialized and memory reclaimed");
        break;

    default:
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
    esp_err_t err = ESP_OK;

    //ELIA: Changing to POST_UPDATE simply made the callback to be executed after the attribute is updated
    if (type == PRE_UPDATE) {
        /* Driver update */
        app_driver_handle_t driver_handle = (app_driver_handle_t)priv_data;
        err = app_driver_attribute_update(driver_handle, endpoint_id, cluster_id, attribute_id, val);
    }

    return err;
}

extern "C" void app_main()
{
    //esp_err_t err = ESP_OK;

    /* Initialize the ESP NVS layer */
    nvs_flash_init();

    esp_matter::node::config_t node_config;
    esp_matter::node_t *node = esp_matter::node::create(&node_config, app_attribute_update_cb, NULL);

    // --- Endpoint for Bottom Blinds ---
    esp_matter::endpoint::window_covering_device::config_t bottom_window_config;
    esp_matter::endpoint_t *bottom_endpoint = esp_matter::endpoint::window_covering_device::create(node, &bottom_window_config, esp_matter::endpoint_flags::ENDPOINT_FLAG_NONE, NULL);
    blinds_endpoint_id = esp_matter::endpoint::get_id(bottom_endpoint);
    ESP_LOGI(TAG, "Bottom Blinds created with endpoint_id %d", blinds_endpoint_id);

    esp_matter::cluster_t *bottom_cluster = esp_matter::cluster::get(bottom_endpoint, chip::app::Clusters::WindowCovering::Id);
    esp_matter::cluster::window_covering::feature::lift::config_t bottom_lift;
    esp_matter::cluster::window_covering::feature::position_aware_lift::config_t bottom_position_aware_lift;
    nullable<uint8_t> bottom_percentage = nullable<uint8_t>(0);
    nullable<uint16_t> bottom_percentage_100ths = nullable<uint16_t>(0);
    bottom_position_aware_lift.current_position_lift_percentage = bottom_percentage;
    bottom_position_aware_lift.target_position_lift_percent_100ths = bottom_percentage_100ths;
    bottom_position_aware_lift.current_position_lift_percent_100ths = bottom_percentage_100ths;
    esp_matter::cluster::window_covering::feature::absolute_position::config_t bottom_absolute_position;
    esp_matter::cluster::window_covering::feature::lift::add(bottom_cluster, &bottom_lift);
    esp_matter::cluster::window_covering::feature::position_aware_lift::add(bottom_cluster, &bottom_position_aware_lift);
    esp_matter::cluster::window_covering::feature::absolute_position::add(bottom_cluster, &bottom_absolute_position);

    
    // --- Endpoint for Top Lift ---
    esp_matter::endpoint::window_covering_device::config_t top_window_config;
    esp_matter::endpoint_t *top_endpoint = esp_matter::endpoint::window_covering_device::create(node, &top_window_config, esp_matter::endpoint_flags::ENDPOINT_FLAG_NONE, NULL);
    blinds_top_endpoint_id = esp_matter::endpoint::get_id(top_endpoint);
    ESP_LOGI(TAG, "Top Lift created with endpoint_id %d", blinds_top_endpoint_id);

    esp_matter::cluster_t *top_cluster = esp_matter::cluster::get(top_endpoint, chip::app::Clusters::WindowCovering::Id);
    esp_matter::cluster::window_covering::feature::lift::config_t blinds_top;
    esp_matter::cluster::window_covering::feature::position_aware_lift::config_t top_position_aware_lift;
    nullable<uint8_t> top_percentage = nullable<uint8_t>(0);
    nullable<uint16_t> top_percentage_100ths = nullable<uint16_t>(0);
    top_position_aware_lift.current_position_lift_percentage = top_percentage;
    top_position_aware_lift.target_position_lift_percent_100ths = top_percentage_100ths;
    top_position_aware_lift.current_position_lift_percent_100ths = top_percentage_100ths;
    esp_matter::cluster::window_covering::feature::absolute_position::config_t top_absolute_position;
    esp_matter::cluster::window_covering::feature::lift::add(top_cluster, &blinds_top);
    esp_matter::cluster::window_covering::feature::position_aware_lift::add(top_cluster, &top_position_aware_lift);
    esp_matter::cluster::window_covering::feature::absolute_position::add(top_cluster, &top_absolute_position);

    // add diagnostic logs cluster on root endpoint
    esp_matter::cluster::diagnostic_logs::config_t diag_logs_config;
    esp_matter::endpoint_t *root_ep = esp_matter::endpoint::get(node, 0); // get the root node ep
    esp_matter::cluster::diagnostic_logs::create(root_ep, &diag_logs_config, CLUSTER_FLAG_SERVER);

    esp_matter::start(app_event_cb);

    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait a bit
    
    app_driver_blinds_init();
    app_driver_handle_t button_handle = app_driver_button_init();
    app_driver_handle_t button_reset_handle = app_driver_reset_button_init();

    //app_driver_blinds_top_init(blinds_top_endpoint_id); // Initialize the driver for the top lift


}
