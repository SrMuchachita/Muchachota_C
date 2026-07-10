# ota_http — Componente OTA reutilizable para ESP32

Permite que cualquier proyecto ESP-IDF se actualice automaticamente
descargando el firmware desde un servidor HTTP local o desde GitHub.
Soporta HTTP y HTTPS (con certificados). Probado con ESP32-S3, ESP-IDF 6.x.

---

## Estructura del componente

```
ota_http/
├── CMakeLists.txt       componente ESP-IDF
├── include/
│   └── ota_http.h       API publica (2 funciones)
├── ota_http.c           logica interna
├── templates/
│   ├── partitions.csv   tabla de particiones lista para copiar
│   └── sdkconfig.defaults  configuracion base lista para copiar
└── README.md            esta guia
```

---

## Migracion a otro proyecto — paso a paso

### Paso 1 — Copiar la carpeta del componente

```
ota_http/  →  tu_proyecto/components/ota_http/
```

Tu proyecto debe quedar asi:

```
tu_proyecto/
├── CMakeLists.txt
├── partitions.csv          <- viene de templates/
├── sdkconfig.defaults      <- viene de templates/
├── version.json            <- archivo para GitHub
├── components/
│   └── ota_http/           <- carpeta copiada
└── main/
    ├── CMakeLists.txt
    └── main.c
```

---

### Paso 2 — Copiar los archivos de templates a la raiz del proyecto

Copia estos dos archivos desde `components/ota_http/templates/`
a la raiz de tu proyecto:

```
templates/partitions.csv      →  tu_proyecto/partitions.csv
templates/sdkconfig.defaults  →  tu_proyecto/sdkconfig.defaults
```

Si tu proyecto ya tiene `sdkconfig.defaults`, agrega estas lineas
al final del tuyo en vez de reemplazarlo:

```
CONFIG_PARTITION_TABLE_CUSTOM=y
CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions.csv"
CONFIG_PARTITION_TABLE_OFFSET=0x8000
CONFIG_ESPTOOLPY_FLASHSIZE_4MB=y
CONFIG_MBEDTLS_CERTIFICATE_BUNDLE=y
CONFIG_MBEDTLS_CERTIFICATE_BUNDLE_DEFAULT_FULL=y
```

> IMPORTANTE: si ya tienes un sdkconfig generado, borralo para
> que se regenere con la nueva configuracion:
>
>   del sdkconfig
>   rmdir /s /q build

---

### Paso 3 — Agregar VERSION al CMakeLists.txt raiz

```cmake
cmake_minimum_required(VERSION 3.22)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(tu_proyecto VERSION 1.0.0)
```

El numero de version es lo que el ESP32 compara contra el servidor.
Si son iguales, no actualiza. Siempre sube el numero al publicar.

---

### Paso 4 — Actualizar main/CMakeLists.txt

Agrega los REQUIRES que necesita el componente:

```cmake
idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES
                        nvs_flash
                        esp_wifi
                        esp_netif
                        esp_event
                        ota_http)
```

Si ya tienes otros REQUIRES, solo agrega los que falten.

---

### Paso 5 — Modificar main.c

#### 5.1 — Includes

```c
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "ota_http.h"
```

#### 5.2 — Defines

```c
#define WIFI_SSID        "TuRedWiFi"
#define WIFI_PASS        "TuPassword"

/* Opcion A — GitHub (internet, desde cualquier lugar) */
#define OTA_VERSION_URL  "https://raw.githubusercontent.com/TU_USUARIO/TU_REPO/main/version.json"
#define OTA_FIRMWARE_URL "https://github.com/TU_USUARIO/TU_REPO/releases/latest/download/firmware.bin"

/* Opcion B — Servidor local (sin internet, red local) */
/* #define OTA_VERSION_URL  "http://192.168.1.100:8080/version.json"  */
/* #define OTA_FIRMWARE_URL "http://192.168.1.100:8080/firmware.bin"  */

#define OTA_CHECK  60   /* segundos entre chequeos */
```

#### 5.3 — Manejador WiFi

Si tu proyecto NO tiene WiFi todavia, agrega este bloque completo:

```c
static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI("wifi", "IP: " IPSTR, IP2STR(&e->ip_info.ip));
        ota_http_notify_connected();   /* <-- avisa al componente OTA */
    }
}

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, NULL));
    wifi_config_t wifi_cfg = {
        .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
}
```

Si tu proyecto YA tiene WiFi, solo agrega esta linea dentro de
tu handler cuando obtienes la IP:

```c
ota_http_notify_connected();
```

#### 5.4 — app_main()

```c
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init();

    ota_http_config_t ota_cfg = {
        .version_url        = OTA_VERSION_URL,
        .firmware_url       = OTA_FIRMWARE_URL,
        .check_interval_sec = OTA_CHECK,
    };
    ota_http_start(&ota_cfg);

    /* ... tu codigo original sigue aqui sin cambios ... */
}
```

---

### Paso 6 — Compilar y flashear por primera vez

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

> Nota: si ya habia un sdkconfig de una configuracion anterior,
> borralo antes de compilar (ver Paso 2).

---

## Como publicar una actualizacion por GitHub

```
1. Modifica tu codigo

2. Sube el numero en CMakeLists.txt raiz:
      project(tu_proyecto VERSION 1.1.0)

3. Compila:
      idf.py build

4. En GitHub, crea un nuevo Release:
      - Ve a tu repo → Releases → "Draft a new release"
      - Tag: v1.1.0
      - Sube el archivo: build\tu_proyecto.bin
      - Renombralo a "firmware.bin" al subirlo
      - Publica el release

5. Actualiza version.json en la raiz del repo:
      {"version":"1.1.0"}

6. Haz commit y push de version.json:
      git add version.json
      git commit -m "version 1.1.0"
      git push

7. Espera el intervalo configurado (ej: 60 segundos)
   El ESP32 detecta el cambio y se actualiza solo
```

### Como subir el proyecto a GitHub por primera vez

```bash
git init
git add .
git commit -m "version inicial"
git remote add origin https://github.com/TU_USUARIO/TU_REPO.git
git push -u origin main
```

> Si el repo ya existe en GitHub con archivos (README, licencia),
> haz esto en vez del push de arriba:
>   git pull origin main --allow-unrelated-histories
>   git push -u origin main

---

## API — solo necesitas 2 funciones

```c
/* En app_main() — inicia el task OTA en segundo plano */
esp_err_t ota_http_start(const ota_http_config_t *config);

/* En el WiFi event handler — llama cuando obtienes IP */
void ota_http_notify_connected(void);
```

### Estructura de configuracion

```c
ota_http_config_t cfg = {
    .version_url        = "https://raw.githubusercontent.com/.../version.json",
    .firmware_url       = "https://github.com/.../releases/latest/download/firmware.bin",
    .check_interval_sec = 60,
};
```

---

## Errores comunes y soluciones

| Error en monitor | Causa | Solucion |
|---|---|---|
| `No hay particion OTA disponible` | Falta partitions.csv o sdkconfig sin OTA | Copiar templates/ y borrar sdkconfig + build/ |
| `Sin respuesta del servidor` | URL incorrecta o sin internet | Verificar URL y conexion WiFi |
| `Imagen invalida` | El .bin es para otro chip | Compilar con `idf.py set-target esp32s3` |
| No actualiza aunque version es diferente | version.json y VERSION no coinciden | Verificar que sean identicos: `1.1.0` == `1.1.0` |
| WiFi no conecta | SSID o password incorrectos | Revisar defines WIFI_SSID y WIFI_PASS |
| Error TLS / certificado | Bundle no habilitado | Agregar `CONFIG_MBEDTLS_CERTIFICATE_BUNDLE=y` al sdkconfig.defaults y borrar build/ |
