#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_crt_bundle.h"
#include "esp_app_format.h"
#include "ota_http.h"

static const char *TAG = "OTA_HTTP";

static SemaphoreHandle_t s_wifi_ready = NULL;
static ota_http_config_t s_cfg;

/* ── API publica ──────────────────────────────────────────────────────────── */

void ota_http_notify_connected(void)
{
    if (s_wifi_ready) {
        xSemaphoreGive(s_wifi_ready);
    }
}

/* ── Chequeo de version ───────────────────────────────────────────────────── */

static bool fetch_remote_version(char *out, size_t out_len)
{
    char resp[256] = {0};
    bool ok = false;

    esp_http_client_config_t cfg = {
        .url               = s_cfg.version_url,
        .timeout_ms        = 10000,
        .crt_bundle_attach = esp_crt_bundle_attach, /* soporta HTTP y HTTPS */
        .max_redirection_count = 5,
    };
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    if (!c) return false;

    if (esp_http_client_open(c, 0) == ESP_OK) {
        esp_http_client_fetch_headers(c);
        int n = esp_http_client_read(c, resp, sizeof(resp) - 1);
        esp_http_client_close(c);

        if (n > 0) {
            char *p = strstr(resp, "\"version\"");
            if (p) { p = strchr(p, ':'); }
            if (p) { p = strchr(p, '"'); }
            if (p) { p++; sscanf(p, "%31[^\"]", out); ok = true; }
        }
    }

    esp_http_client_cleanup(c);
    return ok;
}

/* ── Descarga y escritura en flash (soporta HTTPS + redirecciones) ─────────── */

static esp_err_t download_and_flash(void)
{
    esp_http_client_config_t http_cfg = {
        .url                   = s_cfg.firmware_url,
        .timeout_ms            = 60000,
        .crt_bundle_attach     = esp_crt_bundle_attach,
        .max_redirection_count = 5,
        .keep_alive_enable     = true,
        .buffer_size           = 8192,
        .buffer_size_tx        = 4096,
    };
    esp_https_ota_config_t ota_cfg = {
        .http_config = &http_cfg,
    };

    ESP_LOGI(TAG, "Descargando: %s", s_cfg.firmware_url);
    esp_err_t err = esp_https_ota(&ota_cfg);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "OTA completo. Reiniciando...");
        esp_restart();
    }
    return err;
}

/* ── Task principal ───────────────────────────────────────────────────────── */

static void ota_task(void *arg)
{
    xSemaphoreTake(s_wifi_ready, portMAX_DELAY);

    const esp_app_desc_t *app = esp_app_get_description();
    ESP_LOGI(TAG, "Version actual: %s", app->version);

    while (1) {
        ESP_LOGI(TAG, "Consultando servidor...");

        char remote_ver[32] = {0};

        if (!fetch_remote_version(remote_ver, sizeof(remote_ver))) {
            ESP_LOGW(TAG, "Sin respuesta del servidor. Reintentando en %ds",
                     s_cfg.check_interval_sec);
        } else {
            ESP_LOGI(TAG, "Remota: v%s  |  Local: v%s", remote_ver, app->version);

            if (strcmp(remote_ver, app->version) != 0) {
                ESP_LOGI(TAG, "Nueva version disponible. Actualizando...");
                esp_err_t err = download_and_flash();
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Fallo la actualizacion: %s", esp_err_to_name(err));
                }
            } else {
                ESP_LOGI(TAG, "Firmware al dia.");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(s_cfg.check_interval_sec * 1000));
    }
}

/* ── ota_http_start ───────────────────────────────────────────────────────── */

esp_err_t ota_http_start(const ota_http_config_t *config)
{
    if (!config || !config->version_url || !config->firmware_url) {
        return ESP_ERR_INVALID_ARG;
    }

    s_cfg = *config;

    s_wifi_ready = xSemaphoreCreateBinary();
    if (!s_wifi_ready) {
        return ESP_ERR_NO_MEM;
    }

    BaseType_t ok = xTaskCreate(ota_task, "ota_task", 16384, NULL, 5, NULL);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Componente OTA iniciado. Esperando conexion WiFi...");
    return ESP_OK;
}
