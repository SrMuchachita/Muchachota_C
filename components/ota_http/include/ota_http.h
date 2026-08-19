#pragma once

#include "esp_err.h"

/**
 * Estados del proceso OTA, reportados via el callback on_status.
 */
typedef enum {
    OTA_HTTP_STATUS_CHECKING       = 0, /* Consultando servidor de version           */
    OTA_HTTP_STATUS_NO_RESPONSE    = 1, /* Sin respuesta del servidor                */
    OTA_HTTP_STATUS_UP_TO_DATE     = 2, /* Firmware al dia                           */
    OTA_HTTP_STATUS_UPDATING       = 3, /* Nueva version encontrada, descargando     */
    OTA_HTTP_STATUS_SUCCESS        = 4, /* Descarga OK, reiniciando                  */
    OTA_HTTP_STATUS_FAILED         = 5, /* Fallo la descarga/actualizacion           */
} ota_http_status_t;

/**
 * Configuracion del componente OTA HTTP/HTTPS.
 * Funciona con servidor local (HTTP) y GitHub (HTTPS).
 */
typedef struct {
    const char *version_url;        /* URL completa del version.json          */
    const char *firmware_url;       /* URL completa del firmware.bin          */
    int         check_interval_sec; /* Segundos entre chequeos, ej: 60        */
    void (*on_status)(ota_http_status_t status); /* Opcional: notifica cambios de estado (ej: para mostrar en HMI) */
} ota_http_config_t;

/**
 * Inicia el task OTA en segundo plano.
 * Llama esto en app_main() despues de inicializar NVS y WiFi.
 */
esp_err_t ota_http_start(const ota_http_config_t *config);

/**
 * Notifica al task OTA que el WiFi ya tiene IP.
 * Llamalo en tu manejador de eventos WiFi cuando obtengas IP.
 */
void ota_http_notify_connected(void);
