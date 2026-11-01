#include <stdio.h>
#include <string.h>
#include "mdns.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "esp_http_client.h"
#include "esp_ota_ops.h"
#include "esp_timer.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include <stdlib.h>

#define TOUCH_SENSOR_PIN   GPIO_NUM_4
#define LED_CONTROL_PIN  GPIO_NUM_3
#define FAN_CONTROL_PIN  GPIO_NUM_2
#define WIFI_SSID          "Shivam"
#define WIFI_PASS          "12345678"
// ============================================================
// TB6612FNG MOTOR DRIVER
// ============================================================

#define MOTOR_PWMA   GPIO_NUM_1
#define MOTOR_AIN2   GPIO_NUM_10
#define MOTOR_AIN1   GPIO_NUM_9
#define MOTOR_STBY   GPIO_NUM_8

#define MOTOR_BIN1   GPIO_NUM_7
#define MOTOR_BIN2   GPIO_NUM_6
#define MOTOR_PWMB   GPIO_NUM_5

// Incoming motor command range: -255 to +255
// Actual hardware PWM is always reduced to 50%.
#define MOTOR_PWM_SCALE 1.25

#define MOTOR_PWM_FREQ       1000
#define MOTOR_PWM_RESOLUTION LEDC_TIMER_8_BIT

#define MOTOR_LEFT_CHANNEL   LEDC_CHANNEL_0
#define MOTOR_RIGHT_CHANNEL  LEDC_CHANNEL_1
#define MOTOR_PWM_TIMER      LEDC_TIMER_0

#define DEPLOYED_VERSION_URL "http://Shivams-MacBook-Air-2.local:8000/version.txt"
#define DEPLOYED_FIRMWARE_URL "http://Shivams-MacBook-Air-2.local:8000/sample_project.bin"
static bool get_installed_version(char *version_buffer, size_t buffer_size);
static bool save_installed_version(const char *version);
static int ble_gap_event(struct ble_gap_event *event, void *arg);
static bool get_remote_firmware_version(char *version_buffer, size_t buffer_size);
static bool ota_update_required(char *remote_version,size_t buffer_size);
static bool save_pending_version(const char *version);
static bool get_pending_version(char *version_buffer, size_t buffer_size);
static bool clear_pending_version(void);
static void launch_espressif_prebuilt_ota(const char *new_version);
static void ble_host_task(void *param);
static void ble_on_sync(void);
static void motor_control(int left_command, int right_command);
static const char *TAG = "EON_ROVER_CORE";
 // BLE Nordic UART Service (NUS)
static const ble_uuid128_t gatt_svr_svc_uuid =
    BLE_UUID128_INIT(
        0x9e, 0xca, 0xdc, 0x24,
        0x0e, 0xe5,
        0xa9, 0xe0,
        0xf3, 0x93,
        0xa3, 0xb5,
        0x01, 0x00, 0x40, 0x6e
    );

static const ble_uuid128_t gatt_svr_rx_uuid =
    BLE_UUID128_INIT(
        0x9e, 0xca, 0xdc, 0x24,
        0x0e, 0xe5,
        0xa9, 0xe0,
        0xf3, 0x93,
        0xa3, 0xb5,
        0x02, 0x00, 0x40, 0x6e
    );

static const ble_uuid128_t gatt_svr_tx_uuid =
    BLE_UUID128_INIT(
        0x9e, 0xca, 0xdc, 0x24,
        0x0e, 0xe5,
        0xa9, 0xe0,
        0xf3, 0x93,
        0xa3, 0xb5,
        0x03, 0x00, 0x40, 0x6e
    );
static void process_ble_motor_command(const char *data)
{
    int left = 0;
    int right = 0;
    int light = 0;
    int fan = 0;
    char direction = 'N';

    int parsed = sscanf(
        data,
        "<L:%d,R:%d,D:%c,LGT:%d,FAN:%d>",
        &left,
        &right,
        &direction,
        &light,
        &fan
    );

    if (parsed != 5) {
        ESP_LOGW(TAG, "⚠️ Invalid BLE packet: %s", data);
        return;
    }

    ESP_LOGI(
        TAG,
        "🎮 BLE COMMAND: L=%d R=%d D=%c LGT=%d FAN=%d",
        left,
        right,
        direction,
        light,
        fan
    );

    // Motor control
    motor_control(left, right);

    // LED control
    // Active-low LED and fan control
    gpio_set_level(GPIO_NUM_3, light ? 0 : 1);
    gpio_set_level(GPIO_NUM_2, fan ? 0 : 1);
    ESP_LOGI(
        TAG,
        "💡 LED=%s | 🌀 FAN=%s",
        light ? "ON" : "OFF",
        fan ? "ON" : "OFF"
    );
}
static int gatt_svr_chr_access(uint16_t conn_handle,uint16_t attr_handle,struct ble_gatt_access_ctxt *ctxt,void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        char data[128];

        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len >= sizeof(data)) {
            len = sizeof(data) - 1;
        }

        int rc = ble_hs_mbuf_to_flat(ctxt->om, data, len, NULL);
        if (rc != 0) {
            return BLE_ATT_ERR_UNLIKELY;
        }

        data[len] = '\0';

        ESP_LOGI(TAG, "📱 BLE RX: %s", data);
        process_ble_motor_command(data);

        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &gatt_svr_rx_uuid.u,
                .access_cb = gatt_svr_chr_access,
                .flags = BLE_GATT_CHR_F_WRITE |
                         BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid = &gatt_svr_tx_uuid.u,
                .access_cb = gatt_svr_chr_access,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            {
                0,
            }
        },
    },
    {
        0,
    },
};
static void gatt_svr_init(void)
{
    ble_svc_gap_init();
    ble_svc_gatt_init();

    ble_gatts_count_cfg(gatt_svr_svcs);
    ble_gatts_add_svcs(gatt_svr_svcs);
}
static void ble_on_sync(void)
{
    uint8_t addr_type;
    int rc;

    rc = ble_hs_id_infer_auto(0, &addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE identity setup failed: %d", rc);
        return;
    }

    ESP_LOGI(TAG, "🔵 BLE host synchronized.");
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    const char *device_name = "EON-CUBIC-ROVER";

    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "❌ BLE advertising fields failed: %d", rc);
        return;
    }

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(
        addr_type,
        NULL,
        BLE_HS_FOREVER,
        &adv_params,
        ble_gap_event,
        NULL
    );

    if (rc != 0) {
        ESP_LOGE(TAG, "❌ BLE advertising failed: %d", rc);
        return;
    }

    ESP_LOGI(TAG, "📡 Advertising as: %s", device_name);
}
static void ble_host_task(void *param)
{
    ESP_LOGI(TAG, "🔵 BLE host task started.");

    nimble_port_run();

    nimble_port_freertos_deinit();
}
static void ble_init(void)
{
    ESP_LOGI(TAG, "🔵 Initializing BLE...");

    nimble_port_init();

    ble_svc_gap_init();
    ble_svc_gatt_init();

    ble_hs_cfg.sync_cb = ble_on_sync;

    gatt_svr_init();

    nimble_port_freertos_init(ble_host_task);

    ESP_LOGI(TAG, "🔵 BLE initialization complete.");
}
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            ESP_LOGI(TAG,
                     "🔗 BLE CONNECTED: conn_handle=%d",
                     event->connect.conn_handle);
        } else {
            ESP_LOGE(TAG,
                     "❌ BLE CONNECTION FAILED: status=%d",
                     event->connect.status);
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG,
                 "🔴 BLE DISCONNECTED: conn_handle=%d reason=%d",
                 event->disconnect.conn.conn_handle,
                 event->disconnect.reason);

        /* Restart advertising so the phone can reconnect. */
        {
            struct ble_gap_adv_params adv_params;
            memset(&adv_params, 0, sizeof(adv_params));

            adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
            adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

            int rc = ble_gap_adv_start(
                BLE_OWN_ADDR_PUBLIC,
                NULL,
                BLE_HS_FOREVER,
                &adv_params,
                ble_gap_event,
                NULL
            );

            if (rc != 0) {
                ESP_LOGE(TAG,
                         "❌ Failed to restart advertising: %d",
                         rc);
            } else {
                ESP_LOGI(TAG, "📡 Advertising restarted.");
            }
        }
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "📡 BLE advertising complete.");
        break;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG,
                 "📐 BLE MTU updated: conn_handle=%d mtu=%d",
                 event->mtu.conn_handle,
                 event->mtu.value);
        break;

    default:
        break;
    }

    return 0;
}
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define FIRMWARE_VERSION "V1"


static bool wifi_is_running = false;

void suspend_wifi_radio_stack(void) {
    if (wifi_is_running) {
        ESP_LOGW(TAG, "🔋 Touch LOW triggered: Suspending RF Radios... Disabling Wi-Fi driver.");
        esp_wifi_disconnect();
        esp_wifi_stop();
        wifi_is_running = false;
        ESP_LOGI(TAG, "🌙 Radio stack shut down. System entered Low Power Standby Protocol Mode.");
    }
}

void resume_wifi_radio_stack(void) {
    if (!wifi_is_running) {
        ESP_LOGI(TAG, "🌐 Touch HIGH triggered: Reactivating Wi-Fi Station Infrastructure...");
        esp_wifi_start();
        wifi_is_running = true;
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        wifi_is_running = true;
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        if (wifi_is_running) {
            esp_wifi_connect();
            ESP_LOGW(TAG, "🔄 Re-connecting to network mesh...");
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "📡 Linked! Local Allocation Node IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void initialize_network_drivers(void) {
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    esp_wifi_set_max_tx_power(56); // MacBook low-power safety config
}

void launch_espressif_prebuilt_ota(const char *new_version)
{
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "🚀 LOCAL HTTP OTA TEST STARTING");
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "Firmware URL: %s", DEPLOYED_FIRMWARE_URL);

    esp_http_client_config_t config = {
        .url = DEPLOYED_FIRMWARE_URL,
        .timeout_ms = 15000,
        .keep_alive_enable = true,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(TAG, "❌ Failed to initialize HTTP client.");
        return;
    }

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to connect to OTA server: %s",
                 esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return;
    }

    int content_length = esp_http_client_fetch_headers(client);
    int status_code = esp_http_client_get_status_code(client);

    ESP_LOGI(TAG, "HTTP status: %d", status_code);
    ESP_LOGI(TAG, "Firmware size: %d bytes", content_length);

    if (status_code != 200) {
        ESP_LOGE(TAG, "❌ OTA server returned HTTP %d.", status_code);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return;
    }

    if (content_length <= 0) {
        ESP_LOGE(TAG, "❌ Invalid firmware Content-Length.");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return;
    }

    const esp_partition_t *update_partition =
        esp_ota_get_next_update_partition(NULL);

    if (update_partition == NULL) {
        ESP_LOGE(TAG, "❌ No OTA update partition available.");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return;
    }

    ESP_LOGI(TAG, "🎯 OTA target partition: %s", update_partition->label);

    esp_ota_handle_t ota_handle = 0;

    err = esp_ota_begin(
        update_partition,
        (size_t)content_length,
        &ota_handle
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ esp_ota_begin failed: %s",
                 esp_err_to_name(err));
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return;
    }

    static uint8_t ota_write_buffer[4096];

    int read_bytes = 0;
    int total_bytes = 0;

    while ((read_bytes = esp_http_client_read(
                client,
                (char *)ota_write_buffer,
                sizeof(ota_write_buffer))) > 0) {

        err = esp_ota_write(
            ota_handle,
            ota_write_buffer,
            read_bytes
        );

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "❌ esp_ota_write failed: %s",
                     esp_err_to_name(err));

            esp_ota_abort(ota_handle);
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            return;
        }

        total_bytes += read_bytes;

        ESP_LOGI(
            TAG,
            "📥 OTA progress: %d / %d bytes",
            total_bytes,
            content_length
        );
    }

    if (total_bytes != content_length) {
        ESP_LOGE(
            TAG,
            "❌ Incomplete OTA download: received %d / %d bytes.",
            total_bytes,
            content_length
        );

        esp_ota_abort(ota_handle);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return;
    }

    err = esp_ota_end(ota_handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ Firmware validation failed: %s",
                 esp_err_to_name(err));

        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return;
    }

    ESP_LOGI(TAG, "✅ Firmware image validated successfully.");

    err = esp_ota_set_boot_partition(update_partition);

    if (err != ESP_OK) {

        ESP_LOGE(TAG,
                "❌ Failed to select new boot partition: %s",
                esp_err_to_name(err));

        esp_http_client_close(client);
        esp_http_client_cleanup(client);

        return;
    }

    if (!save_pending_version(new_version)) {

        ESP_LOGE(TAG,
                "❌ Failed to save pending firmware version.");

        esp_http_client_close(client);
        esp_http_client_cleanup(client);

        return;
    }

    ESP_LOGI(TAG,
            "📝 Pending OTA version recorded: %s",
            new_version);

    ESP_LOGI(TAG,
            "🟢 OTA UPDATE SUCCESSFUL");

    ESP_LOGI(TAG,
            "🎯 Next boot partition: %s",
            update_partition->label);

    ESP_LOGI(TAG,
            "🔄 Restarting in 2 seconds...");

    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    vTaskDelay(pdMS_TO_TICKS(2000));

    esp_restart();
}
static void led_control_init(void)
{
    gpio_reset_pin(LED_CONTROL_PIN);

    gpio_set_direction(
        LED_CONTROL_PIN,
        GPIO_MODE_OUTPUT
    );

    gpio_set_level(
        LED_CONTROL_PIN,
        0
    );

    ESP_LOGI(
        TAG,
        "💡 LED control initialized on GPIO %d.",
        LED_CONTROL_PIN
    );
}


static void set_led_state(bool on)
{
    gpio_set_level(
        LED_CONTROL_PIN,
        on ? 1 : 0
    );
}   
static bool wait_for_wifi_connection(void)
{
    ESP_LOGI(TAG, "📡 Waiting for Wi-Fi connection before OTA...");

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(15000)
    );

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "✅ Wi-Fi connection confirmed. OTA can proceed.");
        return true;
    }

    ESP_LOGE(TAG, "❌ Wi-Fi connection timeout. OTA will not start.");
    return false;
}
static bool get_remote_firmware_version(char *version_buffer, size_t buffer_size)
{
    esp_http_client_config_t config = {
        .url = DEPLOYED_VERSION_URL,
        .timeout_ms = 5000,
        .keep_alive_enable = true,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    if (client == NULL) {
        ESP_LOGE(TAG, "❌ Failed to initialize version HTTP client.");
        return false;
    }

    esp_err_t err = esp_http_client_open(client, 0);

    if (err != ESP_OK) {
        ESP_LOGE(TAG,
                 "❌ Failed to connect to version server: %s",
                 esp_err_to_name(err));

        esp_http_client_cleanup(client);
        return false;
    }

    int content_length = esp_http_client_fetch_headers(client);
    int status_code = esp_http_client_get_status_code(client);

    if (status_code != 200) {
        ESP_LOGE(TAG,
                 "❌ Version server returned HTTP %d.",
                 status_code);

        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return false;
    }

    if (content_length <= 0 ||
        content_length >= (int)buffer_size) {

        ESP_LOGE(TAG,
                 "❌ Invalid version file size: %d",
                 content_length);

        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return false;
    }

    int total_read = 0;

    while (total_read < content_length) {

        int read_bytes = esp_http_client_read(
            client,
            version_buffer + total_read,
            content_length - total_read
        );

        if (read_bytes <= 0) {
            break;
        }

        total_read += read_bytes;
    }

    version_buffer[total_read] = '\0';

    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if (total_read != content_length) {
        ESP_LOGE(TAG,
                 "❌ Incomplete version download: %d / %d bytes",
                 total_read,
                 content_length);

        return false;
    }

    // Remove newline / carriage return from version.txt
    for (int i = 0; i < total_read; i++) {

        if (version_buffer[i] == '\r' ||
            version_buffer[i] == '\n' ||
            version_buffer[i] == ' ') {

            version_buffer[i] = '\0';
            break;
        }
    }

    ESP_LOGI(TAG,
             "🌐 Remote firmware version: %s",
             version_buffer);

    return true;
}
static bool ota_update_required(char *remote_version,size_t buffer_size)
{
    char installed_version[32] = {0};

    ESP_LOGI(TAG, "🔎 Checking firmware version...");

    // Read version stored in NVS
    get_installed_version(
        installed_version,
        sizeof(installed_version)
    );

    // Get version available on server
    if (!get_remote_firmware_version(
            remote_version,
            buffer_size)) {

        ESP_LOGW(TAG,
                 "⚠️ Could not determine remote firmware version.");

        return false;
    }

    ESP_LOGI(TAG,
             "📦 Installed firmware version: %s",
             installed_version);

    ESP_LOGI(TAG,
             "🌐 Available firmware version: %s",
             remote_version);

    if (strcmp(installed_version, remote_version) == 0) {

        ESP_LOGI(TAG,
                 "✅ Firmware is already up to date.");

        return false;
    }

    ESP_LOGI(TAG,
             "🆕 Firmware update available: %s → %s",
             installed_version,
             remote_version);

    return true;
}
static bool get_installed_version(char *version_buffer, size_t buffer_size)
{
    nvs_handle_t nvs_handle;

    esp_err_t err = nvs_open("ota", NVS_READONLY, &nvs_handle);

    if (err != ESP_OK) {
        ESP_LOGW(TAG,
                 "⚠️ OTA version not stored yet. Using unknown version.");

        strncpy(version_buffer, "UNKNOWN", buffer_size);
        version_buffer[buffer_size - 1] = '\0';

        return false;
    }

    size_t required_size = buffer_size;

    err = nvs_get_str(
        nvs_handle,
        "version",
        version_buffer,
        &required_size
    );

    nvs_close(nvs_handle);

    if (err != ESP_OK) {

        ESP_LOGW(TAG,
                 "⚠️ No installed firmware version found.");

        strncpy(version_buffer, "UNKNOWN", buffer_size);
        version_buffer[buffer_size - 1] = '\0';

        return false;
    }

    ESP_LOGI(TAG,
             "📦 Installed firmware version: %s",
             version_buffer);

    return true;
}
static bool save_installed_version(const char *version)
{
    nvs_handle_t nvs_handle;

    esp_err_t err = nvs_open(
        "ota",
        NVS_READWRITE,
        &nvs_handle
    );

    if (err != ESP_OK) {

        ESP_LOGE(TAG,
                 "❌ Failed to open OTA NVS namespace: %s",
                 esp_err_to_name(err));

        return false;
    }

    err = nvs_set_str(
        nvs_handle,
        "version",
        version
    );

    if (err != ESP_OK) {

        ESP_LOGE(TAG,
                 "❌ Failed to store firmware version: %s",
                 esp_err_to_name(err));

        nvs_close(nvs_handle);
        return false;
    }

    err = nvs_commit(nvs_handle);

    nvs_close(nvs_handle);

    if (err != ESP_OK) {

        ESP_LOGE(TAG,
                 "❌ Failed to commit firmware version: %s",
                 esp_err_to_name(err));

        return false;
    }

    ESP_LOGI(TAG,
             "💾 Installed firmware version saved: %s",
             version);

    return true;
}
static bool save_pending_version(const char *version)
{
    nvs_handle_t nvs_handle;

    esp_err_t err = nvs_open(
        "ota",
        NVS_READWRITE,
        &nvs_handle
    );

    if (err != ESP_OK) {

        ESP_LOGE(TAG,
                 "❌ Failed to open OTA NVS namespace: %s",
                 esp_err_to_name(err));

        return false;
    }

    err = nvs_set_str(
        nvs_handle,
        "pending_version",
        version
    );

    if (err != ESP_OK) {

        ESP_LOGE(TAG,
                 "❌ Failed to store pending firmware version: %s",
                 esp_err_to_name(err));

        nvs_close(nvs_handle);
        return false;
    }

    err = nvs_commit(nvs_handle);

    nvs_close(nvs_handle);

    if (err != ESP_OK) {

        ESP_LOGE(TAG,
                 "❌ Failed to commit pending firmware version: %s",
                 esp_err_to_name(err));

        return false;
    }

    ESP_LOGI(TAG,
             "📝 Pending firmware version saved: %s",
             version);

    return true;
}
static bool get_pending_version(char *version_buffer,size_t buffer_size)
{
    nvs_handle_t nvs_handle;

    esp_err_t err = nvs_open(
        "ota",
        NVS_READONLY,
        &nvs_handle
    );

    if (err != ESP_OK) {

        return false;
    }

    size_t required_size = buffer_size;

    err = nvs_get_str(
        nvs_handle,
        "pending_version",
        version_buffer,
        &required_size
    );

    nvs_close(nvs_handle);

    if (err != ESP_OK) {

        version_buffer[0] = '\0';
        return false;
    }

    return true;
}
static bool clear_pending_version(void)
{
    nvs_handle_t nvs_handle;

    esp_err_t err = nvs_open(
        "ota",
        NVS_READWRITE,
        &nvs_handle
    );

    if (err != ESP_OK) {

        ESP_LOGE(TAG,
                 "❌ Failed to open OTA NVS namespace: %s",
                 esp_err_to_name(err));

        return false;
    }

    err = nvs_erase_key(
        nvs_handle,
        "pending_version"
    );

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(nvs_handle);
        return true;
    }

    if (err != ESP_OK) {

        ESP_LOGE(TAG,
                 "❌ Failed to clear pending firmware version: %s",
                 esp_err_to_name(err));

        nvs_close(nvs_handle);
        return false;
    }

    err = nvs_commit(nvs_handle);

    nvs_close(nvs_handle);

    if (err != ESP_OK) {

        ESP_LOGE(TAG,
                 "❌ Failed to commit pending-version removal: %s",
                 esp_err_to_name(err));

        return false;
    }

    ESP_LOGI(TAG,
             "🧹 Pending firmware version cleared.");

    return true;
}
static void initialize_installed_version(void)
{
    char installed_version[32] = {0};

    if (!get_installed_version(
            installed_version,
            sizeof(installed_version))) {

        ESP_LOGI(TAG,
                 "🆕 No installed firmware version found.");

        ESP_LOGI(TAG,
                 "💾 Initializing installed version to UNKNOWN.");

        save_installed_version("UNKNOWN");

        return;
    }

    ESP_LOGI(TAG,
             "📦 Confirmed installed firmware version: %s",
             installed_version);
}
static void test_mac_hostname_resolution(void)
{
    ESP_LOGI(TAG, "🔎 Testing mDNS resolution for Mac...");
    ESP_ERROR_CHECK(mdns_init());
    esp_ip4_addr_t mac_ip = {0};

    esp_err_t err = mdns_query_a(
        "Shivams-MacBook-Air-2",
        5000,
        &mac_ip
    );

    if (err == ESP_OK) {
        ESP_LOGI(
            TAG,
            "✅ mDNS SUCCESS: Shivams-MacBook-Air-2.local -> " IPSTR,
            IP2STR(&mac_ip)
        );
    } else {
        ESP_LOGE(
            TAG,
            "❌ mDNS FAILED: %s",
            esp_err_to_name(err)
        );
    }
}
static void motor_control_init(void)
{
    ESP_LOGI(TAG, "⚙️ Initializing TB6612FNG motor driver...");

    // Direction pins
    gpio_reset_pin(MOTOR_AIN1);
    gpio_reset_pin(MOTOR_AIN2);
    gpio_reset_pin(MOTOR_BIN1);
    gpio_reset_pin(MOTOR_BIN2);
    gpio_reset_pin(MOTOR_STBY);

    gpio_set_direction(MOTOR_AIN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_AIN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_BIN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_BIN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_STBY, GPIO_MODE_OUTPUT);

    // SAFE START:
    // Standby disabled + both motors stopped.
    gpio_set_level(MOTOR_STBY, 0);

    gpio_set_level(MOTOR_AIN1, 0);
    gpio_set_level(MOTOR_AIN2, 0);
    gpio_set_level(MOTOR_BIN1, 0);
    gpio_set_level(MOTOR_BIN2, 0);

    // PWM timer
    ledc_timer_config_t motor_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = MOTOR_PWM_TIMER,
        .duty_resolution  = MOTOR_PWM_RESOLUTION,
        .freq_hz          = MOTOR_PWM_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };

    ESP_ERROR_CHECK(ledc_timer_config(&motor_timer));

    // Left motor PWM - PWMA
    ledc_channel_config_t left_pwm = {
        .gpio_num   = MOTOR_PWMA,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = MOTOR_LEFT_CHANNEL,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = MOTOR_PWM_TIMER,
        .duty       = 0,
        .hpoint     = 0
    };

    ESP_ERROR_CHECK(ledc_channel_config(&left_pwm));

    // Right motor PWM - PWMB
    ledc_channel_config_t right_pwm = {
        .gpio_num   = MOTOR_PWMB,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = MOTOR_RIGHT_CHANNEL,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = MOTOR_PWM_TIMER,
        .duty       = 0,
        .hpoint     = 0
    };

    ESP_ERROR_CHECK(ledc_channel_config(&right_pwm));

    ESP_LOGI(TAG, "✅ TB6612FNG initialized.");
    ESP_LOGI(TAG, "🛡️ Motor PWM safety scaling: 50%%");
}
static void motor_control(int left_command, int right_command)
{
    // Keep incoming commands inside the expected range.
    if (left_command > 255) {
        left_command = 255;
    }

    if (left_command < -255) {
        left_command = -255;
    }

    if (right_command > 255) {
        right_command = 255;
    }

    if (right_command < -255) {
        right_command = -255;
    }

    // --------------------------------------------------------
    // Direction
    // --------------------------------------------------------

    // Motor A - corrected direction
    if (left_command > 0) {
        gpio_set_level(MOTOR_AIN1, 0);
        gpio_set_level(MOTOR_AIN2, 1);
    }
    else if (left_command < 0) {
        gpio_set_level(MOTOR_AIN1, 1);
        gpio_set_level(MOTOR_AIN2, 0);
    }
    else {
        gpio_set_level(MOTOR_AIN1, 0);
        gpio_set_level(MOTOR_AIN2, 0);
    }

    if (right_command > 0) {
        // Right motor forward
        gpio_set_level(MOTOR_BIN1, 1);
        gpio_set_level(MOTOR_BIN2, 0);
    }
    else if (right_command < 0) {
        // Right motor reverse
        gpio_set_level(MOTOR_BIN1, 0);
        gpio_set_level(MOTOR_BIN2, 1);
    }
    else {
        // Right motor stop
        gpio_set_level(MOTOR_BIN1, 0);
        gpio_set_level(MOTOR_BIN2, 0);
    }

    // --------------------------------------------------------
    // 50% PWM scaling
    //
    // 255 -> 127
    // 200 -> 100
    // 127 -> 63
    // 100 -> 50
    // --------------------------------------------------------

    uint32_t left_pwm =
        (uint32_t)(abs(left_command) / MOTOR_PWM_SCALE);

    uint32_t right_pwm =
        (uint32_t)(abs(right_command) / MOTOR_PWM_SCALE);

    // Apply PWM
    ESP_ERROR_CHECK(
        ledc_set_duty(
            LEDC_LOW_SPEED_MODE,
            MOTOR_LEFT_CHANNEL,
            left_pwm
        )
    );

    ESP_ERROR_CHECK(
        ledc_update_duty(
            LEDC_LOW_SPEED_MODE,
            MOTOR_LEFT_CHANNEL
        )
    );

    ESP_ERROR_CHECK(
        ledc_set_duty(
            LEDC_LOW_SPEED_MODE,
            MOTOR_RIGHT_CHANNEL,
            right_pwm
        )
    );

    ESP_ERROR_CHECK(
        ledc_update_duty(
            LEDC_LOW_SPEED_MODE,
            MOTOR_RIGHT_CHANNEL
        )
    );

    // Enable driver only when there is something to drive.
    if (left_command != 0 || right_command != 0) {
        gpio_set_level(MOTOR_STBY, 1);
    }
    else {
        gpio_set_level(MOTOR_STBY, 0);
    }

    ESP_LOGI(
        TAG,
        "🚗 Motor: L=%d → PWM=%lu | R=%d → PWM=%lu",
        left_command,
        (unsigned long)left_pwm,
        right_command,
        (unsigned long)right_pwm
    );
}

static void motor_hardware_test(void)
{
    ESP_LOGW(TAG, "=== EON MOTOR TEST: FULL PWM ===");

    motor_control(0, 0);
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "TEST 1: MOTOR A FORWARD");
    motor_control(255, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    motor_control(0, 0);
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "TEST 2: MOTOR B FORWARD");
    motor_control(0, 255);
    vTaskDelay(pdMS_TO_TICKS(1000));
    motor_control(0, 0);
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "TEST 3: BOTH MOTORS FORWARD");
    motor_control(255, 255);
    vTaskDelay(pdMS_TO_TICKS(1000));
    motor_control(0, 0);

    ESP_LOGW(TAG, "=== MOTOR TEST FINISHED ===");
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    initialize_installed_version();

    gpio_reset_pin(TOUCH_SENSOR_PIN);
    gpio_set_direction(TOUCH_SENSOR_PIN, GPIO_MODE_INPUT);
    gpio_reset_pin(GPIO_NUM_3);
    gpio_reset_pin(GPIO_NUM_2);

    gpio_set_direction(GPIO_NUM_3, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);

    gpio_set_level(GPIO_NUM_3, 1);
    gpio_set_level(GPIO_NUM_2, 1);
    led_control_init();
    motor_control_init();

    ESP_LOGI(TAG, "================================================");
    ESP_LOGI(TAG, "🤖 EON Cubic Rover - Raw Stream Low-Level Active");
    ESP_LOGI(TAG, "================================================");
    const esp_partition_t *running_partition =
    esp_ota_get_running_partition();

    esp_ota_img_states_t ota_state;

    if (esp_ota_get_state_partition(running_partition, &ota_state) == ESP_OK) {

        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {

            ESP_LOGI(TAG,
                    "✅ OTA firmware started successfully.");

            ESP_LOGI(TAG,
                    "✅ Confirming current firmware as VALID.");

            esp_err_t confirm_err =
                esp_ota_mark_app_valid_cancel_rollback();

            if (confirm_err == ESP_OK) {

                ESP_LOGI(TAG,
                        "🟢 Firmware confirmed. Rollback protection cleared.");

                char pending_version[32] = {0};

                if (get_pending_version(
                        pending_version,
                        sizeof(pending_version))) {

                    ESP_LOGI(TAG,
                            "📝 Confirmed OTA version: %s",
                            pending_version);

                    if (save_installed_version(pending_version)) {

                        ESP_LOGI(TAG,
                                "💾 Installed firmware version updated to: %s",
                                pending_version);

                        if (clear_pending_version()) {

                            ESP_LOGI(TAG,
                                    "🧹 OTA pending state cleared.");

                        } else {

                            ESP_LOGW(TAG,
                                    "⚠️ Firmware version saved, but pending state could not be cleared.");
                        }

                    } else {

                        ESP_LOGE(TAG,
                                "❌ Firmware confirmed, but installed version could not be saved.");
                    }

                } else {

                    ESP_LOGW(TAG,
                            "⚠️ Firmware confirmed, but no pending OTA version was found.");
                }
            }
             else {

                ESP_LOGE(TAG,
                        "❌ Failed to confirm firmware: %s",
                        esp_err_to_name(confirm_err));
            }
        }
    }
    
   // launch_espressif_prebuilt_ota();

        // ==========================================
    // EON ROVER - TEMPORARY DIRECT BLE MODE
    // ==========================================

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "       EON ROVER - BLE MODE");
    ESP_LOGI(TAG, "========================================");

    ESP_LOGI(TAG, "Starting BLE directly...");

    ble_init();

    ESP_LOGI(TAG, "BLE initialization requested.");
    ESP_LOGI(TAG, "Device name: EON-CUBIC-ROVER");
    ESP_LOGI(TAG, "Waiting for Android controller...");

    // Keep app_main alive while NimBLE runs.
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}