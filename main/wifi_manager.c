#include "wifi_manager.h"

#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "app_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "mdns.h"
#include "nvs_flash.h"
#include "wifi_credentials.h"

static const char *TAG = "wifi";
static const EventBits_t GOT_IP_BIT = BIT0;
static EventGroupHandle_t connection_events;

static const char *auth_mode_name(wifi_auth_mode_t mode)
{
    switch (mode) {
        case WIFI_AUTH_OPEN: return "open";
        case WIFI_AUTH_WEP: return "WEP";
        case WIFI_AUTH_WPA_PSK: return "WPA-PSK";
        case WIFI_AUTH_WPA2_PSK: return "WPA2-PSK";
        case WIFI_AUTH_WPA_WPA2_PSK: return "WPA/WPA2-PSK";
        case WIFI_AUTH_WPA3_PSK: return "WPA3-SAE";
        case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2/WPA3";
        case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2-Enterprise";
        default: return "other";
    }
}

static const char *disconnect_reason_name(uint8_t reason)
{
    switch (reason) {
        case WIFI_REASON_UNSPECIFIED: return "unspecified by AP";
        case WIFI_REASON_AUTH_FAIL: return "authentication failed";
        case WIFI_REASON_ASSOC_FAIL: return "association failed";
        case WIFI_REASON_HANDSHAKE_TIMEOUT: return "four-way handshake timeout";
        case WIFI_REASON_CONNECTION_FAIL: return "connection failed";
        case WIFI_REASON_NO_AP_FOUND: return "no AP with that name found";
        case WIFI_REASON_NO_AP_FOUND_W_COMPATIBLE_SECURITY:
            return "incompatible AP security";
        case WIFI_REASON_NO_AP_FOUND_IN_AUTHMODE_THRESHOLD:
            return "AP below auth-mode threshold";
        default: return "see the ESP-IDF Wi-Fi reason table";
    }
}

/*
 * Publish espgimbal.local. Running on a bench supply there is no serial console
 * to read an address off, and a DHCP lease is not guaranteed to come back the
 * same after a power cut. A failure here is logged and ignored: the board is
 * still reachable by IP, so it must not stop the stream coming up.
 */
static void start_mdns_responder(void)
{
    esp_err_t error = mdns_init();
    if (error != ESP_OK) {
        ESP_LOGW(TAG, "mDNS unavailable (%s); use the IP address above",
                 esp_err_to_name(error));
        return;
    }

    if (mdns_hostname_set(MDNS_HOSTNAME) != ESP_OK ||
        mdns_instance_name_set("ESP32-S3 hand-tracking gimbal") != ESP_OK) {
        ESP_LOGW(TAG, "Could not set the mDNS hostname");
        return;
    }

    mdns_service_add(NULL, "_gimbalcam", "_tcp", STREAM_TCP_PORT, NULL, 0);
    mdns_service_add(NULL, "_gimbalctl", "_tcp", CONTROL_TCP_PORT, NULL, 0);

    ESP_LOGI(TAG, "Also reachable as %s.local", MDNS_HOSTNAME);
}

static void handle_wifi_event(void *context,
                              esp_event_base_t event_base,
                              int32_t event_id,
                              void *event_data)
{
    (void)context;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        const wifi_event_sta_connected_t *connected = event_data;
        ESP_LOGI(TAG,
                 "Handshake completed with BSSID " MACSTR " on channel %u using %s",
                 MAC2STR(connected->bssid),
                 connected->channel,
                 auth_mode_name(connected->authmode));
    } else if (event_base == WIFI_EVENT &&
               event_id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *disconnect = event_data;

        xEventGroupClearBits(connection_events, GOT_IP_BIT);
        ESP_LOGW(TAG,
                 "Disconnected from BSSID " MACSTR ", RSSI %d dBm: "
                 "reason %u (%s); reconnecting...",
                 MAC2STR(disconnect->bssid),
                 disconnect->rssi,
                 disconnect->reason,
                 disconnect_reason_name(disconnect->reason));
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *got_ip = event_data;
        ESP_LOGI(TAG, "Connected. Board IP address: " IPSTR,
                 IP2STR(&got_ip->ip_info.ip));
        ESP_LOGI(TAG, "Start the desktop tracker with that address");
        start_mdns_responder();
        xEventGroupSetBits(connection_events, GOT_IP_BIT);
    }
}

esp_err_t wifi_manager_connect(void)
{
    if (strcmp(WIFI_NETWORK_NAME, "YOUR_2_4_GHZ_WIFI_NAME") == 0) {
        ESP_LOGE(TAG, "Set WIFI_NETWORK_NAME and WIFI_NETWORK_PASSWORD in "
                      "main/include/wifi_credentials.h");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t error = nvs_flash_init();
    if (error == ESP_ERR_NVS_NO_FREE_PAGES ||
        error == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        error = nvs_flash_init();
    }
    if (error != ESP_OK) {
        return error;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    connection_events = xEventGroupCreate();
    if (connection_events == NULL) {
        return ESP_ERR_NO_MEM;
    }

    const wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_config));

    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &handle_wifi_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &handle_wifi_event, NULL));

    wifi_config_t station_config = {0};
    strlcpy((char *)station_config.sta.ssid,
            WIFI_NETWORK_NAME,
            sizeof(station_config.sta.ssid));
    strlcpy((char *)station_config.sta.password,
            WIFI_NETWORK_PASSWORD,
            sizeof(station_config.sta.password));
    station_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    /* Scan every channel and take the strongest matching BSSID, which matters
     * on a mesh where the nearest node changes as the gimbal is moved around. */
    station_config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    station_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    station_config.sta.pmf_cfg.capable = true;
    station_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &station_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Power saving adds irregular pauses that are very visible in video, and
     * it would also delay the tracking commands coming back the other way. */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    /* Never log the passphrase itself; lengths are enough to catch a typo. */
    ESP_LOGI(TAG,
             "Credential check: SSID length %u, passphrase length %u "
             "(passphrase contents intentionally hidden)",
             (unsigned)strlen(WIFI_NETWORK_NAME),
             (unsigned)strlen(WIFI_NETWORK_PASSWORD));

    ESP_LOGI(TAG, "Joining %s using the strongest compatible BSSID...",
             WIFI_NETWORK_NAME);
    ESP_ERROR_CHECK(esp_wifi_connect());
    xEventGroupWaitBits(connection_events,
                        GOT_IP_BIT,
                        pdFALSE,
                        pdTRUE,
                        portMAX_DELAY);
    return ESP_OK;
}
