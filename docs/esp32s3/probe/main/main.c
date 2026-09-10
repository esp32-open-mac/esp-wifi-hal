#include <inttypes.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "ping/ping_sock.h"
#include "network.h" /* Generated in private artifact directory, never committed. */

#define GOT_IP BIT0
#define LOST_WIFI BIT1
#define PING_DONE BIT2
static EventGroupHandle_t events;
static esp_netif_ip_info_t network;
static const char *TAG = "s3_hal_e2e";
static unsigned replies;
static void event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        network = ((ip_event_got_ip_t *)data)->ip_info;
        ESP_LOGI(TAG, "stage=got_ip ip=" IPSTR " gateway=" IPSTR,
                 IP2STR(&network.ip), IP2STR(&network.gw));
        xEventGroupSetBits(events, GOT_IP);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "stage=disconnected reason=%u", ((wifi_event_sta_disconnected_t *)data)->reason);
        xEventGroupSetBits(events, LOST_WIFI);
    }
}
static void ping_ok(esp_ping_handle_t handle, void *arg)
{ (void)handle; (void)arg; ++replies; }
static void ping_end(esp_ping_handle_t handle, void *arg)
{ (void)handle; (void)arg; xEventGroupSetBits(events, PING_DONE); }
static void ping_gateway(void)
{
    esp_ping_config_t config = ESP_PING_DEFAULT_CONFIG();
    IP_ADDR4(&config.target_addr, ip4_addr1(&network.gw), ip4_addr2(&network.gw),
             ip4_addr3(&network.gw), ip4_addr4(&network.gw));
    config.count = 20; config.interval_ms = 100; config.timeout_ms = 1000; config.data_size = 512;
    esp_ping_callbacks_t callbacks = {.on_ping_success=ping_ok, .on_ping_end=ping_end};
    esp_ping_handle_t handle; replies=0;
    xEventGroupClearBits(events, PING_DONE);
    ESP_ERROR_CHECK(esp_ping_new_session(&config, &callbacks, &handle));
    ESP_ERROR_CHECK(esp_ping_start(handle));
    EventBits_t bits = xEventGroupWaitBits(events, PING_DONE, pdTRUE, pdFALSE, pdMS_TO_TICKS(30000));
    ESP_ERROR_CHECK(esp_ping_stop(handle));
    ESP_ERROR_CHECK(esp_ping_delete_session(handle));
    ESP_LOGI(TAG, "stage=traffic replies=%u sent=20 bytes=512 finished=%u", replies, !!(bits & PING_DONE));
    if (!(bits & PING_DONE) || replies < 18) {
        ESP_LOGE(TAG, "stage=fail test=traffic"); abort();
    }
}
void app_main(void)
{
#ifdef S3_REVIEWED_HAL
    const char *variant = "reviewed_hal";
#else
    const char *variant = "stock_hal";
#endif
    ESP_LOGI(TAG, "stage=boot variant=%s idf=%s", variant, esp_get_idf_version());
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    events=xEventGroupCreate(); assert(events);
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, event, NULL));
    for (unsigned round=1; round<=3; ++round) {
        wifi_init_config_t init=WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&init));
        ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        wifi_config_t config={0};
        memcpy(config.sta.ssid, S3_TEST_SSID, sizeof(S3_TEST_SSID)-1);
        memcpy(config.sta.password, S3_TEST_PASSWORD, sizeof(S3_TEST_PASSWORD)-1);
        config.sta.threshold.authmode=WIFI_AUTH_WPA2_PSK;
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &config));
        ESP_ERROR_CHECK(esp_wifi_start());
        ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
        wifi_scan_config_t scan={.show_hidden=true, .scan_type=WIFI_SCAN_TYPE_PASSIVE, .scan_time.passive=120};
        ESP_ERROR_CHECK(esp_wifi_scan_start(&scan, true));
        uint16_t found=0; ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&found));
        ESP_ERROR_CHECK(esp_wifi_clear_ap_list());
        ESP_LOGI(TAG, "stage=scan round=%u found=%u", round, found);
        xEventGroupClearBits(events, GOT_IP|LOST_WIFI);
        ESP_ERROR_CHECK(esp_wifi_connect());
        EventBits_t bits=xEventGroupWaitBits(events, GOT_IP|LOST_WIFI, pdFALSE, pdFALSE, pdMS_TO_TICKS(30000));
        if (!(bits & GOT_IP)) { ESP_LOGE(TAG,"stage=fail test=connect round=%u bits=%u",round,(unsigned)bits); abort(); }
        wifi_ap_record_t ap; ESP_ERROR_CHECK(esp_wifi_sta_get_ap_info(&ap));
        ESP_LOGI(TAG,"stage=associated round=%u auth=%u channel=%u rssi=%d",round,ap.authmode,ap.primary,ap.rssi);
        ping_gateway();
        int64_t tsf1=esp_wifi_get_tsf_time(WIFI_IF_STA);
        vTaskDelay(pdMS_TO_TICKS(100));
        int64_t tsf2=esp_wifi_get_tsf_time(WIFI_IF_STA);
        ESP_LOGI(TAG,"stage=tsf value=%" PRId64 " delta=%" PRId64,tsf2,tsf2-tsf1);
        if(tsf1<=0 || tsf2<=tsf1) { ESP_LOGE(TAG,"stage=fail test=tsf"); abort(); }
        ESP_ERROR_CHECK(esp_wifi_disconnect());
        ESP_ERROR_CHECK(esp_wifi_stop());
        ESP_ERROR_CHECK(esp_wifi_deinit());
        ESP_LOGI(TAG,"stage=cycle_complete round=%u free_heap=%" PRIu32,round,esp_get_free_heap_size());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG,"stage=complete variant=%s cycles=3",variant);
}
