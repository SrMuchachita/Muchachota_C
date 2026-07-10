#pragma once

#include "esp_err.h"

/**
 * Configuracion del componente OTA HTTP/HTTPS.
 * Funciona con servidor local (HTTP) y GitHub (HTTPS).
 */
typedef struct {
    const char *version_url;        /* URL completa del version.json          */
    const char *firmware_url;       /* URL completa del firmware.bin          */
    int         check_interval_sec; /* Segundos entre chequeos, ej: 60        */
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
