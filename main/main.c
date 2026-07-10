//ANSI C
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <inttypes.h>

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

// ESP-IDF Drivers
#include "driver/gptimer.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "nvs.h"

// APIs de NimBLE Host 
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

// NimBLE GATT APIs 
#include "host/ble_gatt.h"
#include "services/gatt/ble_svc_gatt.h"

// APIs GAP de NimBLE 
#include "services/gap/ble_svc_gap.h"

// Librerias externas
#include "esp_modbus_master.h"
#include "mbcontroller.h"
#include "ina260.h"

/* ============================= Recursos para Bluetooth =============================== */
#define DEVICE_NAME                      "RD90Pro"
#define BLE_GAP_APPEARANCE_GENERIC_TAG   0x0200
#define BLE_GAP_LE_ROLE_PERIPHERAL       0x00


//*****************************************RECURSOS DE MODULO MAIN************************************************/
#define TASK_SIZE            (6 * 1024)
#define TAG                  "RD90Con"


//*****************************************DEFINICIONES EQUIPO*********************************************************/
#define MB_SLAVE_ADDR        (0x0A)
#define MB_MASTER_BAUD       (115200)


//*****************************************DEFINICIONES GPIO*********************************************************/
// OUTPUT
#define DOUT0   41
#define DOUT1   40
#define DOUT2   39
#define DOUT3   38
#define DOUT_MASK_CONFIG     ((1ULL << DOUT0)|(1ULL << DOUT1)|(1ULL << DOUT2)|(1ULL << DOUT3))

// INPUT NORMAL PULSADORES
#define J1_DIN1      9
#define J2_DIN0      3

// INPUT ENCODER
#define ENC_A_DIN0   12
#define ENC_B_DIN1   13
#define DIN_MASK_CONFIG   ((1ULL << J1_DIN1)|(1ULL << J2_DIN0)|(1ULL << ENC_A_DIN0)|(1ULL << ENC_B_DIN1))

#define PULSES_PER_METER  1000


//*****************************************PINOUT ADC JOYSTIK******************************************************/
#define J2_X_AI0        ADC_CHANNEL_3  // GPIO4
#define J2_Y_AI1        ADC_CHANNEL_4  // GPIO5
#define J1_X_AI2        ADC_CHANNEL_5  // GPIO6
#define J1_Y_AI3        ADC_CHANNEL_6  // GPIO7
#define P1_V1_AI4       ADC_CHANNEL_7  // GPIO8

//*****************************************PINOUT MOTORES DC ******************************************************/
#define MOTOR_COUNT        (4)
#define MOTOR_MAX_PWM      1000


//*****************************************PINOUT SERVOS******************************************************/
//#define SERVO_CH0_PIN     12
//#define SERVO_CH1_PIN     11


//*****************************************PINOUT PWM LED******************************************************/
#define LED_PIN          13
#define LED_FREQ_HZ      5000


//*****************************************DEFINICIONES I2C*********************************************************/
#define INA_I2C_PORT                    I2C_NUM_0 
#define INA_I2C_SDA_PIN                 (21)
#define INA_I2C_SCL_PIN            		(14)
#define INA_DEV_ADDR     		        (0b1000000)
#define INA_DEV_FREQ_Hz                 100000


//*****************************************DEFINICIONES NVS*****************************************************/
#define NVS_NAMESPACE       "rd90_cfg"
#define NVS_KEY_CENTER_SRV1 "ctr_srv1"
#define NVS_KEY_CENTER_SRV2 "ctr_srv2"
#define NVS_KEY_CENTER_SRV3 "ctr_srv3"
#define NVS_KEY_srv1_min    "srv1_min"
#define NVS_KEY_srv1_max    "srv1_max"
#define NVS_KEY_srv2_min    "srv2_min"
#define NVS_KEY_srv2_max    "srv2_max"
#define NVS_KEY_srv3_min    "srv3_min"
#define NVS_KEY_srv3_max    "srv3_max"

//*****************************************DEFINICIONES MODBUS*****************************************************/
#define MB_UART_PORT           UART_NUM_1
#define MB_UART_TXD           (1)
#define MB_UART_RXD           (2)
#define MB_UART_DIR           (42)
#define UART_BUFFER_SIZE      (512)

#define HMI_UART_PORT           UART_NUM_2
#define HMI_UART_TXD           (18)
#define HMI_UART_RXD           (17)
#define HMI_UART_BAUD          (115200)
#define HMI_HEADER_1           0x5A
#define HMI_HEADER_2           0xA5


//******************************* ESTRUCTURAS *****************************************/
typedef enum {
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_REVERSE,
    MOTOR_RIGHT,
    MOTOR_LEFT,
} motor_dir_t;

typedef enum {
    HMI_REG_ONLINE_INDICATOR    = 0x00,
    HMI_REG_BLUETOOTH_INDICATOR = 0x01,
    HMI_REG_ANGLE_HEAD_CHANGED  = 0x02,
    HMI_REG_ANGLE_NECK_CHANGED  = 0x03,
    HMI_REG_ROBOT_LED_CHANGED   = 0x04,
    HMI_REG_GIRO_AUTOMATICO     = 0x05,
    HMI_REG_START_DEMO          = 0x06,
    HMI_REG_STOP_DEMO           = 0x07,
    HMI_REG_ENCODER             = 0x08,
    HMI_REG_CONSOLE_VOLTAGE     = 0x09,
    HMI_REG_ROBOT_VOLTAGE       = 0x0A,
    HMI_REG_ANGLE_X             = 0x0B,
    HMI_REG_ANGLE_Y             = 0x0C,
    HMI_REG_BLUETOOTH_PASSWORD  = 0x0D,
    HMI_REG_ROBOT_SERIAL        = 0x0E,  // sufijo numérico → pantalla muestra "RD90R-XXXXXX"
    HMI_REG_CENTER              = 0x0F,  // bits[31:16]=cuello, bits[15:0]=cabeza
    HMI_REG_SRV1_LIMITS         = 0x10,  // bits[31:16]=max,   bits[15:0]=min
    HMI_REG_SRV2_LIMITS         = 0x11,  // bits[31:16]=max,   bits[15:0]=min
    HMI_REG_JOY1                = 0x12,  // bits[31:16]=joy1_x, bits[15:0]=joy1_y (0-4095)
    HMI_REG_JOY2                = 0x13,  // bits[31:16]=joy2_x, bits[15:0]=joy2_y (0-4095)
    HMI_REG_BUTTONS             = 0x14,  // bit1=J1_DIN1, bit0=J2_DIN0 (0=presionado, 1=suelto)
    HMI_REG_PING                = 0x15,  // HMI → consola cada 3s, value=1
    HMI_REG_PONG                = 0x16,  // consola → HMI en respuesta al PING, value=1
    HMI_REG_P1                  = 0x17,  // P1 joystick ADC crudo (0-4095)
    HMI_REG_CENTER_SRV3         = 0x18,  // HMI → consola: centro srv3 (0-270)
    HMI_REG_SRV3_LIMITS         = 0x19,  // HMI → consola: bits[31:16]=max, bits[15:0]=min
    HMI_REG_MOTOR               = 0x1A,  // bits[31:16]=cmd, bits[15:0]=vel (0-1000)
    HMI_REG_SRV1_ANGLE          = 0x1B,  // ángulo cabeza 0-180°
    HMI_REG_SRV2_ANGLE          = 0x1C,  // ángulo cuello 0-180°
    HMI_REG_SRV3_ANGLE          = 0x1D,  // ángulo servo3 0-270°
    HMI_CMD_MAX
} hmi_reg_t;

typedef struct {
    uint8_t reg;
    int32_t value;
} hmi_tx_frame_t;

typedef struct {
    uint8_t buffer[10];
    uint8_t len;
} hmi_rx_frame_t;

typedef struct {
    uint16_t motor_cmd;
    uint16_t motor_vel;
    uint16_t srv1_angle;
    uint16_t srv2_angle;
    uint16_t led_pwm;
    uint16_t srv3_angle;
} mb_master_cmd_t;

static mb_master_cmd_t mb_cmd = {0};

typedef struct {
    uint16_t subConnHandle;
    bool notifyEnabled;
} notify_state_t;

// NUEVO: Fases del giro automático
typedef enum {
    GIRO_FASE_1 = 0,  // srv1=min, srv2 sube
    GIRO_FASE_2,      // salto instantáneo
    GIRO_FASE_3,      // srv1=max, srv2 sube + pausa
    GIRO_FASE_4,      // retorno a 90,90
    GIRO_IDLE
} giro_auto_fase_t;

//****************************************PROTOTIPOS DE FUNCIONES APP*********************************************/
void vGpioInit(void);
void vI2cInit(void);
void vAdcInit(void);
void vUartInit(void);
void vMbMasterInit(void);
void vHardwareInit (void);


//**************************************PROTOTIPOS DE FUNCIONES DE UTIL*********************************************/
void hmi_send_data(uint8_t reg, int32_t value);
void wait_for_sensor (ina260_t *dev);
uint16_t adc_multi_sampling_channel (adc_oneshot_unit_handle_t handle, adc_channel_t chan);
void hmi_process_buffer(uint8_t *buffer, uint16_t len);
void hmi_handle_reg(uint8_t reg, int32_t value);
void save_center_to_nvs(void);
void load_center_from_nvs(void);
void save_limits_to_nvs(void);
void load_limits_from_nvs(void);


//**************************************PROTOTIPOS DE FUNCIONES BLE *********************************************/
void vBleServiceInit (void);
int  gapInit (void);
void startAdvertisingInit (void);
void startAdvertising (void);
void nimbleHostConfigInit (void);
int  gapEventHandler (struct ble_gap_event *event, void *arg);

void onStackReset (int reason);
void onStackSync (void);
void ble_store_config_init (void);

int  gattSvcInit (void);
void gattSvrSubscribeCb (struct ble_gap_event *event);
int tiltChrAccess (uint16_t, uint16_t, struct ble_gatt_access_ctxt *, void *);
int battChrAccess (uint16_t, uint16_t, struct ble_gatt_access_ctxt *, void *);
int tempChrAccess (uint16_t, uint16_t, struct ble_gatt_access_ctxt *, void *);
int encChrAccess  (uint16_t, uint16_t, struct ble_gatt_access_ctxt *, void *);

//**************************************PROTOTIPOS DE FUNCIONES TAREAS*******************************************/
void vTaskEncoder        (void *pvParameters);
void vTaskPowerMonitor   (void *pvParameters);
void vTaskUartHmiEvents  (void *pvParameters);
void vTaskHmiRxProcess   (void *pvParameters);
void vTaskHmiTransmit    (void *pvParameters);
void vTaskModbusControl  (void *pvParameters);
void vTaskJoystickControl(void *pvParameters);
void vTaskModbusWatchdog (void *pvParameters);
void vTaskNimbleHost     (void *pvParameters);   
void vTaskNimbleNotify   (void *pvParameters);
void vTaskGiroAutomatico (void *pvParameters);  // NUEVO


//************************************** RECURSOS DE FREERTOS ****************************************************/
static QueueHandle_t xQueueHmiTx     = NULL;
static QueueHandle_t xQueueHmiRx     = NULL;
static QueueHandle_t xQueueUartEvent = NULL;


//************************************** RECURSOS DE BLE ****************************************************/
static const ble_uuid128_t robotSvcUuid = BLE_UUID128_INIT(
    0x01,0x0E,0x31,0xE1,0x29,0x2D,0xD5,0xAD,
    0xFE,0x45,0xD0,0xF3,0xF6,0x85,0xC8,0x4C);

static const ble_uuid128_t tiltChrUuid = BLE_UUID128_INIT(
    0x01,0xC0,0xF1,0xA1,0x12,0x12,0xEF,0xDE,
    0x15,0x23,0x78,0x5F,0xEA,0xBC,0x00,0x00);

static const ble_uuid128_t battChrUuid = BLE_UUID128_INIT(
    0x02,0xC0,0xF1,0xA1,0x12,0x12,0xEF,0xDE,
    0x15,0x23,0x78,0x5F,0xEA,0xBC,0x00,0x00);

static const ble_uuid128_t tempChrUuid = BLE_UUID128_INIT(
    0x03,0xC0,0xF1,0xA1,0x12,0x12,0xEF,0xDE,
    0x15,0x23,0x78,0x5F,0xEA,0xBC,0x00,0x00);

static const ble_uuid128_t encChrUuid = BLE_UUID128_INIT(
    0x04,0xC0,0xF1,0xA1,0x12,0x12,0xEF,0xDE,
    0x15,0x23,0x78,0x5F,0xEA,0xBC,0x00,0x00);

uint16_t tiltChrValHandle;
uint16_t battChrValHandle;
uint16_t tempChrValHandle;
uint16_t encChrValHandle;

notify_state_t tiltNotify  = {0};
notify_state_t battNotify  = {0};
notify_state_t tempNotify  = {0};
notify_state_t encNotify   = {0};

int16_t appTilt[2]    = {0, 0};
int16_t appBattery[2] = {0, 0};
int16_t appTemp       = 0;
int32_t appEncoder    = 0;

static const struct ble_gatt_svc_def gattSrvServices[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &robotSvcUuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &tiltChrValHandle,
                .uuid       = &tiltChrUuid.u,
                .access_cb  = tiltChrAccess,
            },
            {
                .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &battChrValHandle,
                .uuid       = &battChrUuid.u,
                .access_cb  = battChrAccess,
            },
            {
                .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &tempChrValHandle,
                .uuid       = &tempChrUuid.u,
                .access_cb  = tempChrAccess,
            },
            {
                .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &encChrValHandle,
                .uuid       = &encChrUuid.u,
                .access_cb  = encChrAccess,
            },
            {0}
        }
    },
    {0}
};

uint8_t ownAddrType = 0;
uint8_t addrVal[6]  = {0};
static uint16_t bleConnHandle = BLE_HS_CONN_HANDLE_NONE;


//************************************** VARIABLES GLOBALES ****************************************************/
static void  *mb_master_handle = NULL;
static portMUX_TYPE paramLock  = portMUX_INITIALIZER_UNLOCKED;

volatile int64_t last_modbus_activity_time = 0;
volatile int32_t encoder_count = 0;
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

static volatile bool mb_led_pwm_updated   = false;
static volatile bool mb_servo_updated_joy = false;
static volatile bool mb_servo_updated_hmi = false;
static volatile bool mb_srv3_updated      = false;
static volatile bool hmi_encoder_reset    = false;
static volatile bool hmi_motor_override   = false;
static volatile bool robot_online_flag    = false;

// NUEVO: Variables para giro automático
static volatile bool giro_auto_active = false;
static volatile bool giro_auto_single_run = false;  // NUEVO: Para ejecutar solo una vez
static giro_auto_fase_t giro_fase = GIRO_IDLE;

i2c_master_bus_handle_t i2c1_handle   = NULL;
ina260_t dev_ina260_handle            = {0};

adc_oneshot_unit_handle_t adc1_handle = NULL;

int16_t target_srv1;
int16_t target_srv2;
int16_t srv1_angle , prev_srv1;
int16_t srv2_angle , prev_srv2;
int16_t srv3_angle , prev_srv3;

static int16_t center_srv1 = 90;
static int16_t center_srv2 = 90;
static int16_t center_srv3 = 135;

static int16_t srv1_min = 0;
static int16_t srv1_max = 180;
static int16_t srv2_min = 0;
static int16_t srv2_max = 180;
static int16_t srv3_min = 0;
static int16_t srv3_max = 180;//maximo 270

//************************************** FUNCIONES DE INTERRUPCION *********************************************/
void IRAM_ATTR encoder_a_isr_handler (void *arg)
{
    int a = gpio_get_level(ENC_A_DIN0);
    int b = gpio_get_level(ENC_B_DIN1);

    portENTER_CRITICAL_ISR(&encoderMux);
    if (a == b) {
        encoder_count++;
    } else {
        encoder_count--;
    }
    portEXIT_CRITICAL_ISR(&encoderMux);
}



/*****************************************************************************************************/
//**************************************APP MAIN******************************************************/
/*****************************************************************************************************/
void app_main(void)
{
    vHardwareInit();
    vBleServiceInit();

    xQueueHmiTx = xQueueCreate(50, sizeof(hmi_tx_frame_t));
    if(xQueueHmiTx == NULL) {
        ESP_LOGE(TAG, "Error al crear xQueueHmiTx");
		return;
    }

    xQueueHmiRx = xQueueCreate(20, sizeof(hmi_rx_frame_t));
    if(xQueueHmiRx == NULL) {
        ESP_LOGE(TAG, "Error al crear xQueueHmiRx");
		return;
    }

    xTaskCreate(vTaskNimbleHost,      "NimBLE",     TASK_SIZE, NULL, 6, NULL);
    xTaskCreate(vTaskNimbleNotify,    "BleNotify",  TASK_SIZE, NULL, 3, NULL);
    xTaskCreate(vTaskHmiTransmit,     "HMI Tx",     TASK_SIZE, NULL, 3, NULL);
    xTaskCreate(vTaskUartHmiEvents,   "HMI Event",  TASK_SIZE, NULL, 4, NULL);
    xTaskCreate(vTaskHmiRxProcess,    "HMI Rx",     TASK_SIZE, NULL, 4, NULL);
    xTaskCreate(vTaskJoystickControl, "Joystick",   TASK_SIZE, NULL, 5, NULL);
    xTaskCreate(vTaskPowerMonitor,    "PowerMon",   TASK_SIZE, NULL, 4, NULL);
    xTaskCreate(vTaskEncoder,         "Encoder",    TASK_SIZE, NULL, 4, NULL);
    xTaskCreate(vTaskModbusControl,   "ModbusCtrl", TASK_SIZE, NULL, 6, NULL);
    xTaskCreate(vTaskModbusWatchdog,  "MbWatchdog", TASK_SIZE, NULL, 7, NULL);
    xTaskCreate(vTaskGiroAutomatico,  "GiroAuto",   TASK_SIZE, NULL, 5, NULL);  // NUEVO

    hmi_send_data(HMI_REG_PONG, 1);
}


/* ============================================================
 * TAREAS FREERTOS
 * ============================================================ */

void vTaskNimbleHost (void *pvParameters) 
{
    nimble_port_run();  
}


void vTaskNimbleNotify (void *pvParameters)
{
    struct os_mbuf *notifyBuf;

    while (1)
    {
        if (tiltNotify.notifyEnabled) {
            notifyBuf = ble_hs_mbuf_from_flat(appTilt, sizeof(appTilt));
            if (notifyBuf) {
                ble_gatts_notify_custom(tiltNotify.subConnHandle, tiltChrValHandle, notifyBuf);
            }
        }

        if (battNotify.notifyEnabled) {
            notifyBuf = ble_hs_mbuf_from_flat(appBattery, sizeof(appBattery));
            if (notifyBuf) {
                ble_gatts_notify_custom(battNotify.subConnHandle, battChrValHandle, notifyBuf);
            }
        }

        if (tempNotify.notifyEnabled) {
            notifyBuf = ble_hs_mbuf_from_flat(&appTemp, sizeof(appTemp));
            if (notifyBuf) {
                ble_gatts_notify_custom(tempNotify.subConnHandle, tempChrValHandle, notifyBuf);
            }
        }

        if (encNotify.notifyEnabled) {
            notifyBuf = ble_hs_mbuf_from_flat(&appEncoder, sizeof(appEncoder));
            if (notifyBuf) {
                ble_gatts_notify_custom(encNotify.subConnHandle, encChrValHandle, notifyBuf);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


// NUEVA TAREA: Giro Automático
void vTaskGiroAutomatico(void *pvParameters)
{
    // Parámetros configurables
    const int16_t SRV_STEP = 1;
    const uint16_t PAUSA_MS = 500;
    const uint8_t DELAY_MS = 50;

    int16_t srv1_current = 90;
    int16_t srv2_current = 90;


    while (1)
    {
        if (!giro_auto_active) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        switch (giro_fase)
        {
            case GIRO_FASE_1:
                //srv1_current = srv1_min;
                static bool init = false;
                if (!init) {
                    srv1_current = srv1_min;
                    srv2_current = srv2_min; 
                    init = true;

                    ESP_LOGI(TAG, "Init Fase 1");
                }

                if (srv2_current < srv2_max) {
                    srv2_current += SRV_STEP;
                } else {
                    giro_fase = GIRO_FASE_2;
                    init = false;
                    ESP_LOGI(TAG, "Giro Auto: Fase 1 → Fase 2");
                }
                break;

            case GIRO_FASE_2:
                srv1_current = srv1_max;
                srv2_current = srv2_min;
                
                giro_fase = GIRO_FASE_3;
                ESP_LOGI(TAG, "Giro Auto: Fase 2 → Fase 3");
                break;

            case GIRO_FASE_3:
                srv1_current = srv1_max;

                if (srv2_current < srv2_max) {
                    srv2_current += SRV_STEP;
                } else {
                    ESP_LOGI(TAG, "Giro Auto: Pausa de %d ms", PAUSA_MS);
                    vTaskDelay(pdMS_TO_TICKS(PAUSA_MS));
                    
                    giro_fase = GIRO_FASE_4;
                    ESP_LOGI(TAG, "Giro Auto: Fase 3 → Fase 4");
                }
                break;

            case GIRO_FASE_4:
                {
                    bool srv1_done = false;
                    bool srv2_done = false;

                    if (srv1_current < 90) {
                        srv1_current += SRV_STEP;
                    } else if (srv1_current > 90) {
                        srv1_current -= SRV_STEP;
                    } else {
                        srv1_done = true;
                    }

                    if (srv2_current < 90) {
                        srv2_current += SRV_STEP;
                    } else if (srv2_current > 90) {
                        srv2_current -= SRV_STEP;
                    } else {
                        srv2_done = true;
                    }

                    if (srv1_done && srv2_done) {
                        // CICLO COMPLETO - DESACTIVAR AUTOMÁTICAMENTE
                        giro_auto_active = false;
                        giro_fase = GIRO_IDLE;
                        ESP_LOGI(TAG, "Giro Auto: COMPLETADO - Esperando nueva activación");
                        srv1_angle  = center_srv1;
                        srv2_angle  = center_srv2;
                        prev_srv1   = center_srv1;
                        prev_srv2   = center_srv2;
                        target_srv1 = center_srv1;
                        target_srv2 = center_srv2;

                        // Notificar al HMI que terminó (opcional)
                        hmi_send_data(HMI_REG_GIRO_AUTOMATICO, (int32_t)0);
                    }
                }
                break;

            case GIRO_IDLE:
            default:
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
        }

        portENTER_CRITICAL(&paramLock);
        mb_cmd.srv1_angle = srv1_current;
        mb_cmd.srv2_angle = srv2_current;
        mb_servo_updated_hmi = true;
        portEXIT_CRITICAL(&paramLock);

        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    }
}


void vTaskJoystickControl (void *pvParameters)
{
    uint16_t joy1_x, joy1_y, joy2_x, joy2_y;

    motor_dir_t cmd = MOTOR_STOP, prev_cmd = MOTOR_STOP;
    uint16_t vel = 0, prev_vel = 0;

    srv1_angle = center_srv1; prev_srv1 = 0;
    srv2_angle = center_srv2; prev_srv2 = 0;

    target_srv1 = center_srv1;
    target_srv2 = center_srv2;

    const uint16_t SERVO_Y_INVERTED = 1;
    const uint16_t ADC_MAX       = 4095;
    const uint8_t  UPD_TIME      = 50;
    const uint8_t  SERVO_STEP    = 1;
    const uint16_t DEAD_MARGIN   = 600;
    const uint16_t S3_DEAD_MARGIN = 1000;
    const uint8_t  CAL_SAMPLES   = 20;

    // Calibración en reposo: medir centro real de cada eje
    uint32_t sum_j1x = 0, sum_j1y = 0, sum_j2x = 0, sum_j2y = 0, sum_p1 = 0;
    for (uint8_t i = 0; i < CAL_SAMPLES; i++) {
        sum_j1x += adc_multi_sampling_channel(adc1_handle, J1_X_AI2);
        sum_j1y += adc_multi_sampling_channel(adc1_handle, J1_Y_AI3);
        sum_j2x += adc_multi_sampling_channel(adc1_handle, J2_X_AI0);
        sum_j2y += adc_multi_sampling_channel(adc1_handle, J2_Y_AI1);
        sum_p1  += adc_multi_sampling_channel(adc1_handle, P1_V1_AI4);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    uint16_t j1x_center = (uint16_t)(sum_j1x / CAL_SAMPLES);
    uint16_t j1y_center = (uint16_t)(sum_j1y / CAL_SAMPLES);
    uint16_t j2x_center = (uint16_t)(sum_j2x / CAL_SAMPLES);
    uint16_t j2y_center = (uint16_t)(sum_j2y / CAL_SAMPLES);
    uint16_t p1_center  = (uint16_t)(sum_p1  / CAL_SAMPLES);

    uint16_t j1x_min = (j1x_center > DEAD_MARGIN) ? j1x_center - DEAD_MARGIN : 0;
    uint16_t j1x_max = j1x_center + DEAD_MARGIN;
    uint16_t j1y_min = (j1y_center > DEAD_MARGIN) ? j1y_center - DEAD_MARGIN : 0;
    uint16_t j1y_max = j1y_center + DEAD_MARGIN;
    uint16_t j2x_min = (j2x_center > DEAD_MARGIN) ? j2x_center - DEAD_MARGIN : 0;
    uint16_t j2x_max = j2x_center + DEAD_MARGIN;
    uint16_t j2y_min = (j2y_center > DEAD_MARGIN) ? j2y_center - DEAD_MARGIN : 0;
    uint16_t j2y_max = j2y_center + DEAD_MARGIN;
    uint16_t p1_min  = (p1_center > S3_DEAD_MARGIN) ? p1_center - S3_DEAD_MARGIN : 0;
    uint16_t p1_max  = p1_center + S3_DEAD_MARGIN;

    srv3_angle = center_srv3; prev_srv3 = -1;
    int16_t target_srv3 = center_srv3;

    ESP_LOGI(TAG, "JOY CAL: J1X=%d J1Y=%d J2X=%d J2Y=%d P1=%d", j1x_center, j1y_center, j2x_center, j2y_center, p1_center);

    uint8_t hmi_joy_counter = 0;
    uint8_t prev_buttons    = 0xFF;

    while (1) {
        // Si giro automático está activo, no procesar joystick
        if (giro_auto_active) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        joy1_x = adc_multi_sampling_channel(adc1_handle, J1_X_AI2);
        joy1_y = adc_multi_sampling_channel(adc1_handle, J1_Y_AI3);
        
        joy2_x = adc_multi_sampling_channel(adc1_handle, J2_X_AI0);
        joy2_y = adc_multi_sampling_channel(adc1_handle, J2_Y_AI1);
        uint16_t p1_val = adc_multi_sampling_channel(adc1_handle, P1_V1_AI4);

        // JOYSTICK 1 → CONTROL ROBOT
        if (joy1_y > j1y_max) {
            cmd = MOTOR_FORWARD;
            vel = (uint16_t)(((uint32_t)(joy1_y - j1y_max) * MOTOR_MAX_PWM) / (ADC_MAX - j1y_max));
        }
        else if (joy1_y < j1y_min) {
            cmd = MOTOR_REVERSE;
            vel = (uint16_t)(((uint32_t)(j1y_min - joy1_y) * MOTOR_MAX_PWM) / j1y_min);
        }
        else if (joy1_x > j1x_max) {
            cmd = MOTOR_RIGHT;
            vel = (uint16_t)(((uint32_t)(joy1_x - j1x_max) * MOTOR_MAX_PWM) / (ADC_MAX - j1x_max));
        }
        else if (joy1_x < j1x_min) {
            cmd = MOTOR_LEFT;
            vel = (uint16_t)(((uint32_t)(j1x_min - joy1_x) * MOTOR_MAX_PWM) / j1x_min);
        }
        else {
            cmd = MOTOR_STOP;
            vel = 0;
        }

        // JOYSTICK 2 → SERVOS (TARGET)
        if (joy2_y > j2y_max) {
            if (SERVO_Y_INVERTED) {
                if (target_srv1 > 0) target_srv1 -= SERVO_STEP;
            } else {
                if (target_srv1 < 180) target_srv1 += SERVO_STEP;
            }
        }
        else if (joy2_y < j2y_min) {
            if (SERVO_Y_INVERTED) {
                if (target_srv1 < 180) target_srv1 += SERVO_STEP;
            } else {
                if (target_srv1 > 0) target_srv1 -= SERVO_STEP;
            }
        }

        if (joy2_x > j2x_max) {
            if (target_srv2 < 180) target_srv2 += SERVO_STEP;
        }
        else if (joy2_x < j2x_min) {
            if (target_srv2 > 0) target_srv2 -= SERVO_STEP;
        }
        if (target_srv1 > srv1_max) target_srv1 = srv1_max;
        if (target_srv1 < srv1_min) target_srv1 = srv1_min;

        if (target_srv2 > srv2_max) target_srv2 = srv2_max;
        if (target_srv2 < srv2_min) target_srv2 = srv2_min;

        // P1 → SRV3
        if (p1_val > p1_max) {
            if (target_srv3 < srv3_max) target_srv3 += SERVO_STEP;
        } else if (p1_val < p1_min) {
            if (target_srv3 > srv3_min) target_srv3 -= SERVO_STEP;
        }
        if (target_srv3 > srv3_max) target_srv3 = srv3_max;
        if (target_srv3 < srv3_min) target_srv3 = srv3_min;

        if (srv3_angle < target_srv3) srv3_angle++;
        else if (srv3_angle > target_srv3) srv3_angle--;

        // J2 → centrar srv1/srv2 y cancelar giro automático
        if (gpio_get_level(J2_DIN0) == 0)
        {
            if (giro_auto_active) {
                giro_auto_active = false;
                giro_fase = GIRO_IDLE;
                ESP_LOGW(TAG, "Giro Automático CANCELADO por botón");
            }
            target_srv1 = center_srv1;
            target_srv2 = center_srv2;
        }

        // J1 → centrar srv3
        if (gpio_get_level(J1_DIN1) == 0)
        {
            target_srv3 = center_srv3;
        }

        // RAMPA SUAVE (1°)
        if (srv1_angle < target_srv1) {
            srv1_angle++;
        }
        else if (srv1_angle > target_srv1) {
            srv1_angle--;
        }

        if (srv2_angle < target_srv2) {
            srv2_angle++;
        }
        else if (srv2_angle > target_srv2) {
            srv2_angle--;
        }

        // ENVIAR MOTORES — solo si HMI no tiene el control
        if (!hmi_motor_override) {
            portENTER_CRITICAL(&paramLock);
            mb_cmd.motor_cmd = cmd;
            mb_cmd.motor_vel = vel;
            portEXIT_CRITICAL(&paramLock);
        }

        if ((cmd != prev_cmd) || (vel != prev_vel))
        {
            hmi_send_data(HMI_REG_MOTOR, ((int32_t)cmd << 16) | (int32_t)vel);
            ESP_LOGI(TAG, "JOY1[X:%d Y:%d] = cmd:%d vel:%d", joy1_x, joy1_y, cmd, vel);
            prev_cmd = cmd;
            prev_vel = vel;
        }

        if ((srv1_angle != prev_srv1) || (srv2_angle != prev_srv2))
        {
            portENTER_CRITICAL(&paramLock);
            mb_cmd.srv1_angle = srv1_angle;
            mb_cmd.srv2_angle = srv2_angle;
            mb_servo_updated_joy = true;
            portEXIT_CRITICAL(&paramLock);

            hmi_send_data(HMI_REG_SRV1_ANGLE, (int32_t)srv1_angle);
            hmi_send_data(HMI_REG_SRV2_ANGLE, (int32_t)srv2_angle);

            prev_srv1 = srv1_angle;
            prev_srv2 = srv2_angle;
        }

        if (srv3_angle != prev_srv3)
        {
            portENTER_CRITICAL(&paramLock);
            mb_cmd.srv3_angle = (uint16_t)srv3_angle;
            mb_srv3_updated = true;
            portEXIT_CRITICAL(&paramLock);

            hmi_send_data(HMI_REG_SRV3_ANGLE, (int32_t)srv3_angle);

            prev_srv3 = srv3_angle;
        }

        // Joysticks + P1 → HMI cada 200ms — ADC crudo (0-4095)
        hmi_joy_counter++;
        if (hmi_joy_counter >= 4) {
            hmi_joy_counter = 0;
            hmi_send_data(HMI_REG_JOY1, ((int32_t)joy1_x << 16) | (int32_t)joy1_y);
            hmi_send_data(HMI_REG_JOY2, ((int32_t)joy2_x << 16) | (int32_t)joy2_y);
            hmi_send_data(HMI_REG_P1,   (int32_t)p1_val);

            ESP_LOGI(TAG, "J1X=%d J1Y=%d J2X=%d J2Y=%d P1=%d BTN1=%d BTN2=%d ENC=%ld",
                joy1_x, joy1_y, joy2_x, joy2_y, p1_val,
                (!gpio_get_level(J1_DIN1)), (!gpio_get_level(J2_DIN0)),
                (long)encoder_count);
        }

        // Botones → HMI solo cuando cambia (1=presionado, 0=suelto)
        uint8_t buttons = (uint8_t)(((!gpio_get_level(J1_DIN1)) << 1) | (!gpio_get_level(J2_DIN0)));
        if (buttons != prev_buttons) {
            hmi_send_data(HMI_REG_BUTTONS, (int32_t)buttons);
            prev_buttons = buttons;
        }

        vTaskDelay(pdMS_TO_TICKS(UPD_TIME));
    }
}


void vTaskModbusControl (void *pvParameters)
{
    int64_t last_read_time = 0;

    uint16_t data_rx[10];

    uint16_t motor_data[2];
    uint16_t servo_data[2];
    uint16_t pwm_data;
    uint16_t srv3_data;

    const uint8_t  UPD_TIME = 50;

    mb_param_request_t req_motor = {
        .slave_addr = MB_SLAVE_ADDR,
        .command    = 0x10,
        .reg_start  = 0x0000,
        .reg_size   = 2,
    };

    mb_param_request_t req_servo = {
        .slave_addr = MB_SLAVE_ADDR,
        .command    = 0x10,
        .reg_start  = 0x0002,
        .reg_size   = 2,
    };

    mb_param_request_t req_pwm = {
        .slave_addr = MB_SLAVE_ADDR,
        .command    = 0x10,
        .reg_start  = 0x0004,
        .reg_size   = 1,
    };

    mb_param_request_t req_srv3 = {
        .slave_addr = MB_SLAVE_ADDR,
        .command    = 0x10,
        .reg_start  = 0x0005,
        .reg_size   = 1,
    };

    mb_param_request_t req_input = {
        .slave_addr = MB_SLAVE_ADDR,
        .command    = 0x04,
        .reg_start  = 0x0000,
        .reg_size   = 6,
    };

    mb_param_request_t req_serial = {
        .slave_addr = MB_SLAVE_ADDR,
        .command    = 0x04,
        .reg_start  = 0x0006,
        .reg_size   = 6,
    };

    bool serial_read_done = false;


    while (1)
    {
        portENTER_CRITICAL(&paramLock);
        motor_data[0] = mb_cmd.motor_cmd;
        motor_data[1] = mb_cmd.motor_vel;
        portEXIT_CRITICAL(&paramLock);

        if(ESP_OK != mbc_master_send_request(mb_master_handle, &req_motor, motor_data)) 
        {
            ESP_LOGE(TAG,"Error al enviar solicitud req_motor");
        }

        if (mb_servo_updated_joy || mb_servo_updated_hmi)
        {
            portENTER_CRITICAL(&paramLock);
            mb_servo_updated_joy = false;
            mb_servo_updated_hmi = false;
            servo_data[0] = mb_cmd.srv1_angle;
            servo_data[1] = mb_cmd.srv2_angle;
            portEXIT_CRITICAL(&paramLock);

            if(ESP_OK != mbc_master_send_request(mb_master_handle, &req_servo, servo_data)) 
            {
                ESP_LOGE(TAG,"Error al enviar solicitud req_servo");
            }
        }

        if (mb_led_pwm_updated)
        {
            portENTER_CRITICAL(&paramLock);
            mb_led_pwm_updated = false;
            pwm_data = mb_cmd.led_pwm;
            portEXIT_CRITICAL(&paramLock);

            if(ESP_OK != mbc_master_send_request(mb_master_handle, &req_pwm, &pwm_data))
            {
                ESP_LOGE(TAG,"Error al enviar solicitud req_pwm");
            }
        }

        if (mb_srv3_updated)
        {
            portENTER_CRITICAL(&paramLock);
            mb_srv3_updated = false;
            srv3_data = mb_cmd.srv3_angle;
            portEXIT_CRITICAL(&paramLock);

            if(ESP_OK != mbc_master_send_request(mb_master_handle, &req_srv3, &srv3_data))
            {
                ESP_LOGE(TAG,"Error al enviar solicitud req_srv3");
            }
        }

        int64_t now = esp_timer_get_time() / 1000;

        if ((now - last_read_time) >= 1000)
        {
            last_read_time = now;

            if (ESP_OK == mbc_master_send_request(mb_master_handle, &req_input, data_rx))
            {
                int16_t pitch_raw = (int16_t)data_rx[0];
                int16_t roll_raw  = (int16_t)data_rx[1];
                int16_t temp_raw  = (int16_t)data_rx[2];

                uint16_t voltage = data_rx[3];
                (void)data_rx[4];
                (void)data_rx[5];

                hmi_send_data(HMI_REG_ANGLE_X, (int32_t)roll_raw);
                hmi_send_data(HMI_REG_ANGLE_Y, (int32_t)pitch_raw);
                hmi_send_data(HMI_REG_ROBOT_VOLTAGE, (int32_t)voltage);
                hmi_send_data(HMI_REG_ONLINE_INDICATOR, (int32_t)255);

                appBattery[1] = voltage;
                appTilt[0]    = pitch_raw;
                appTilt[1]    = roll_raw;
                appTemp       = temp_raw;

                if (!serial_read_done) {
                    uint16_t serial_regs[6] = {0};
                    if (ESP_OK == mbc_master_send_request(mb_master_handle, &req_serial, serial_regs)) {
                        // Extraer los 6 chars ASCII del sufijo (regs 0x0009-0x000B)
                        char suffix[7] = {
                            (char)(serial_regs[3] >> 8),
                            (char)(serial_regs[3] & 0xFF),
                            (char)(serial_regs[4] >> 8),
                            (char)(serial_regs[4] & 0xFF),
                            (char)(serial_regs[5] >> 8),
                            (char)(serial_regs[5] & 0xFF),
                            '\0'
                        };

                        int32_t serial_num = 0;
                        for (int i = 0; i < 6; i++) {
                            if (suffix[i] >= '0' && suffix[i] <= '9') {
                                serial_num = serial_num * 10 + (suffix[i] - '0');
                            }
                        }

                        hmi_send_data(HMI_REG_ROBOT_SERIAL, serial_num);

                        ESP_LOGI(TAG, "Robot serial: RD90R-%s (%ld)", suffix, (long)serial_num);

                        serial_read_done = true;
                    }
                }
            }
            else
            {
                ESP_LOGW(TAG, "Error en lectura Modbus");
                hmi_send_data(HMI_REG_ONLINE_INDICATOR, (int32_t)0);
            }
        }

        last_modbus_activity_time = esp_timer_get_time() / 1000;

        vTaskDelay(pdMS_TO_TICKS(UPD_TIME));
    }
}


void vTaskEncoder(void *pvParameters)
{
    int32_t count;
    int32_t last_count = 0;

    while (1)
    {
        if (hmi_encoder_reset) 
        {
            portENTER_CRITICAL(&paramLock);
            hmi_encoder_reset = false;
            encoder_count     = 0;
            portEXIT_CRITICAL(&paramLock);
        }

        count = encoder_count;

        if (count != last_count)
        {
            hmi_send_data(HMI_REG_ENCODER, count);

            appEncoder = count;

            ESP_LOGI("ENCODER", "Count: %ld", count);

            last_count = count;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void vTaskPowerMonitor (void *pvParameters) {
    float voltage, current, power;
    wait_for_sensor(&dev_ina260_handle);

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));

        if(ESP_OK != ina260_trigger(&dev_ina260_handle)) {
            ESP_LOGE(TAG, "Error en la configuracion de un solo disparo");
            continue;
        }

        wait_for_sensor(&dev_ina260_handle);

        if(ESP_OK != ina260_get_bus_voltage(&dev_ina260_handle, &voltage)) {
            ESP_LOGE(TAG, "Error en la lectura de voltaje");
            continue;
        }

        if(ESP_OK != ina260_get_current(&dev_ina260_handle, &current)) {
            ESP_LOGE(TAG, "Error en la lectura de voltaje");
            continue;
        }

        if(ESP_OK != ina260_get_power(&dev_ina260_handle, &power)) {
            ESP_LOGE(TAG, "Error en la lectura de voltaje");
            continue;
        }

        int32_t voltage_mV = (int32_t)(voltage * 1000.0f);

        appBattery[0] = voltage_mV;
        
        hmi_send_data(HMI_REG_CONSOLE_VOLTAGE, voltage_mV);
    }
}


void vTaskHmiRxProcess(void *pvParameters)
{
    hmi_rx_frame_t rx_frame;

    while (1)
    {
        if (xQueueReceive(xQueueHmiRx, &rx_frame, portMAX_DELAY))
        {
            hmi_process_buffer(rx_frame.buffer, rx_frame.len);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void vTaskHmiTransmit(void *pvParameters)
{
    hmi_tx_frame_t txFrame;
    uint8_t frame[7];

    while (1)
    {
        if (xQueueReceive(xQueueHmiTx, &txFrame, portMAX_DELAY))
        {
            frame[0] = HMI_HEADER_1;
            frame[1] = HMI_HEADER_2;

            frame[2] = txFrame.reg;

            frame[3] = (txFrame.value >> 24);
            frame[4] = (txFrame.value >> 16);
            frame[5] = (txFrame.value >> 8);
            frame[6] = (txFrame.value);

            int sent = uart_write_bytes(HMI_UART_PORT, frame, sizeof(frame));
            if (sent != sizeof(frame)) {
                ESP_LOGE(TAG, "HMI TX error: reg=0x%02X sent=%d", txFrame.reg, sent);
            } else {
                ESP_LOGD(TAG, "HMI TX: [5A][A5][%02X][%02X][%02X][%02X][%02X]",
                    frame[2], frame[3], frame[4], frame[5], frame[6]);
            }
        }
    }
}


void vTaskUartHmiEvents  (void *pvParameters) {
    uart_event_t event;
    hmi_rx_frame_t rx_frame;

    while (1) {
	    if(pdTRUE == xQueueReceive(xQueueUartEvent, (void *)&event, pdMS_TO_TICKS(1000))) 
        {
	        bzero(rx_frame.buffer, sizeof(rx_frame.buffer));
	        
	        switch (event.type) 
            {
                case UART_DATA:	  
                    rx_frame.len = event.size;

                    if (rx_frame.len > sizeof(rx_frame.buffer)) 
                    {
                        rx_frame.len = sizeof(rx_frame.buffer);
                    }

                    uart_read_bytes(HMI_UART_PORT, rx_frame.buffer, rx_frame.len, pdMS_TO_TICKS(10));

                    if(pdFALSE == xQueueSendToBack(xQueueHmiRx, &rx_frame, pdMS_TO_TICKS(10))) {
                        ESP_LOGE(TAG, "HMI_UART_PORT: UART_DATA, Cola Llena");
                    }  
                break;
                    
                case UART_FIFO_OVF:
                    ESP_LOGE(TAG, "UART_HMI_PORT: UART_FIFO_OVF");
                    uart_flush_input(HMI_UART_PORT);
                    xQueueReset(xQueueUartEvent);
                break;
                    
                case UART_BUFFER_FULL:
                    ESP_LOGE(TAG, "UART_HMI_PORT: UART_BUFFER_FULL");
                    uart_flush_input(HMI_UART_PORT);
                    xQueueReset(xQueueUartEvent);
                break;
                    
                case UART_BREAK:
                    ESP_LOGE(TAG, "UART_HMI_PORT: UART_BREAK");
                break;
                    
                case UART_PARITY_ERR:
                    ESP_LOGE(TAG, "UART_HMI_PORT: UART_PARITY_ERR");
                break;
                    
                case UART_FRAME_ERR:
                    ESP_LOGE(TAG, "UART_HMI_PORT: UART_FRAME_ERR");
                break;
                    
                default:
                    ESP_LOGE(TAG, "UART_HMI_PORT: Uart event type: %d \n\n", event.type);
                break;
	        }
	    }
    }
}


void vTaskModbusWatchdog(void *pvParameters)
{
    const uint32_t MODBUS_INACTIVITY_RESET_MS = (3 * 60 * 1000);
    const uint32_t WATCHDOG_CHECK_PERIOD_MS   = 1000;

    while (1)
    {
        int64_t now = esp_timer_get_time() / 1000;

        if ((now - last_modbus_activity_time) > MODBUS_INACTIVITY_RESET_MS)
        {
            ESP_LOGE(TAG, "MODBUS INACTIVITY 5min -> SYSTEM RESTART");

            vTaskDelay(pdMS_TO_TICKS(200));

            esp_restart();
        }

        vTaskDelay(pdMS_TO_TICKS(WATCHDOG_CHECK_PERIOD_MS));
    }
}

//*************************************************************************************
//*****************************FUNCIONES DE UTILIDAD **********************************
//*************************************************************************************

void hmi_process_buffer(uint8_t *buffer, uint16_t len)
{
    uint8_t reg;
    int32_t value;

    for (int i = 0; i <= len - 7; i++)
    {
        if (buffer[i] == HMI_HEADER_1 && buffer[i+1] == HMI_HEADER_2)
        {
            reg = buffer[i+2];

            value = ((int32_t)buffer[i+3] << 24) |
                    ((int32_t)buffer[i+4] << 16) |
                    ((int32_t)buffer[i+5] << 8 ) |
                    ((int32_t)buffer[i+6]);

            hmi_handle_reg(reg, value);

            i += 6;
        }
    }
}

void hmi_handle_reg(uint8_t reg, int32_t value)
{
    switch (reg) 
    {
        case HMI_REG_ANGLE_HEAD_CHANGED:
        if (giro_auto_active) {
        break;  // 🔥 ignorar completamente
            }       
        mb_servo_updated_hmi = true;

        portENTER_CRITICAL(&paramLock);
        mb_cmd.srv1_angle = (uint16_t)value;
        portEXIT_CRITICAL(&paramLock);

        ESP_LOGI(TAG, "HMI_REG_ANGLE_HEAD_CHANGED = %d", value);
        break;
        /*************************************************************/

        case HMI_REG_ANGLE_NECK_CHANGED:
        if (giro_auto_active) {
        break;  // 🔥 ignorar completamente
            } 
        mb_servo_updated_hmi = true;

        portENTER_CRITICAL(&paramLock);
        mb_cmd.srv2_angle = (uint16_t)value;
        portEXIT_CRITICAL(&paramLock);
        
        ESP_LOGI(TAG, "HMI_REG_ANGLE_NECK_CHANGED = %d", value);
        break;
        /*************************************************************/

        case HMI_REG_ROBOT_LED_CHANGED:
        portENTER_CRITICAL(&paramLock);
        mb_led_pwm_updated = true;
        mb_cmd.led_pwm     = (uint16_t)value;
        portEXIT_CRITICAL(&paramLock);

        ESP_LOGI(TAG, "HMI_REG_ROBOT_LED_CHANGED = %d", value);
        break;
        /*************************************************************/

        case HMI_REG_GIRO_AUTOMATICO:
        if (value == 1) {
            // Solo activar si no está ya activo
            if (!giro_auto_active) {
                giro_auto_active = true;
                giro_fase = GIRO_FASE_1;
                ESP_LOGI(TAG, "HMI_REG_GIRO_AUTOMATICO ACTIVADO (ejecución única)");
            } else {
                ESP_LOGW(TAG, "Giro automático ya está en ejecución");
            }
        } else {
            // Desactivar giro automático manualmente
            giro_auto_active = false;
            giro_fase = GIRO_IDLE;
            ESP_LOGI(TAG, "HMI_REG_GIRO_AUTOMATICO DESACTIVADO manualmente");
            }
        break;
        /*************************************************************/

        case HMI_REG_START_DEMO:
        ESP_LOGI(TAG, "HMI_REG_START_DEMO = %d", value);
        break;
        /*************************************************************/

        case HMI_REG_STOP_DEMO:
        ESP_LOGI(TAG, "HMI_REG_STOP_DEMO = %d", value);
        break;
        /*************************************************************/

        case HMI_REG_CENTER:
        {
            int16_t neck_angle = (int16_t)((value >> 16) & 0xFFFF);
            int16_t head_angle = (int16_t)(value & 0xFFFF);

            center_srv1 = head_angle;
            center_srv2 = neck_angle;

            save_center_to_nvs();

            ESP_LOGI(TAG, "HMI_REG_CENTER: cabeza=%d cuello=%d", head_angle, neck_angle);
        }
        break;
        /*************************************************************/

        case HMI_REG_CENTER_SRV3:
        {
            int16_t c = (int16_t)(value & 0xFFFF);
            if (c < 0)   c = 0;
            if (c > 270) c = 270;
            center_srv3 = c;
            save_center_to_nvs();
            ESP_LOGI(TAG, "HMI_REG_CENTER_SRV3: %d", center_srv3);
        }
        break;
        /*************************************************************/

        case HMI_REG_SRV3_LIMITS:
        {
            int16_t lmax = (int16_t)((value >> 16) & 0xFFFF);
            int16_t lmin = (int16_t)(value & 0xFFFF);
            srv3_max = lmax;
            srv3_min = lmin;
            save_limits_to_nvs();
            ESP_LOGI(TAG, "HMI_REG_SRV3_LIMITS: min=%d max=%d", lmin, lmax);
        }
        break;
        /*************************************************************/

        case HMI_REG_SRV1_LIMITS:
        {
            int16_t lmax = (int16_t)((value >> 16) & 0xFFFF);
            int16_t lmin = (int16_t)(value & 0xFFFF);
            srv1_max = lmax;
            srv1_min = lmin;
            save_limits_to_nvs();
            ESP_LOGI(TAG, "HMI_REG_SRV1_LIMITS: min=%d max=%d", lmin, lmax);
        }
        break;
        /*************************************************************/

        case HMI_REG_SRV2_LIMITS:
        {
            int16_t lmax = (int16_t)((value >> 16) & 0xFFFF);
            int16_t lmin = (int16_t)(value & 0xFFFF);
            srv2_max = lmax;
            srv2_min = lmin;
            save_limits_to_nvs();
            ESP_LOGI(TAG, "HMI_REG_SRV2_LIMITS: min=%d max=%d", lmin, lmax);
        }
        break;
        /*************************************************************/

        case HMI_REG_ENCODER:
        hmi_encoder_reset = true;
        ESP_LOGI(TAG, "HMI_REG_ENCODER = %d", value);
        break;
        /*************************************************************/

        case HMI_REG_MOTOR:
        {
            uint16_t hmi_cmd = (uint16_t)((value >> 16) & 0xFFFF);
            uint16_t hmi_vel = (uint16_t)(value & 0xFFFF);
            portENTER_CRITICAL(&paramLock);
            mb_cmd.motor_cmd = hmi_cmd;
            mb_cmd.motor_vel = hmi_vel;
            hmi_motor_override = (hmi_cmd != 0);
            portEXIT_CRITICAL(&paramLock);
            ESP_LOGI(TAG, "HMI_REG_MOTOR: cmd=%d vel=%d", hmi_cmd, hmi_vel);
        }
        break;
        /*************************************************************/

        case HMI_REG_PING:
        hmi_send_data(HMI_REG_PONG, 1);
        break;
        /*************************************************************/

        default:
        ESP_LOGW(TAG, "Unknown REG: 0x%02X", reg);
        break;
        /*************************************************************/
    }
}


void save_center_to_nvs(void)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_i16(handle, NVS_KEY_CENTER_SRV1, center_srv1);
        nvs_set_i16(handle, NVS_KEY_CENTER_SRV2, center_srv2);
        nvs_set_i16(handle, NVS_KEY_CENTER_SRV3, center_srv3);
        nvs_commit(handle);
        nvs_close(handle);
        ESP_LOGI(TAG, "Centro guardado NVS: srv1=%d srv2=%d srv3=%d", center_srv1, center_srv2, center_srv3);
    }
}

void load_center_from_nvs(void)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
        int16_t val;
        if (nvs_get_i16(handle, NVS_KEY_CENTER_SRV1, &val) == ESP_OK) center_srv1 = val;
        if (nvs_get_i16(handle, NVS_KEY_CENTER_SRV2, &val) == ESP_OK) center_srv2 = val;
        if (nvs_get_i16(handle, NVS_KEY_CENTER_SRV3, &val) == ESP_OK) center_srv3 = val;
        nvs_close(handle);
        ESP_LOGI(TAG, "Centro cargado NVS: srv1=%d srv2=%d srv3=%d", center_srv1, center_srv2, center_srv3);
    }
}

void save_limits_to_nvs(void)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_i16(handle, NVS_KEY_srv1_min, srv1_min);
        nvs_set_i16(handle, NVS_KEY_srv1_max, srv1_max);
        nvs_set_i16(handle, NVS_KEY_srv2_min, srv2_min);
        nvs_set_i16(handle, NVS_KEY_srv2_max, srv2_max);
        nvs_set_i16(handle, NVS_KEY_srv3_min, srv3_min);
        nvs_set_i16(handle, NVS_KEY_srv3_max, srv3_max);
        nvs_commit(handle);
        nvs_close(handle);
        ESP_LOGI(TAG, "Limites guardados NVS: srv1=%d-%d srv2=%d-%d srv3=%d-%d", srv1_min, srv1_max, srv2_min, srv2_max, srv3_min, srv3_max);
    }
}

void load_limits_from_nvs(void)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
        int16_t val;
        if (nvs_get_i16(handle, NVS_KEY_srv1_min, &val) == ESP_OK) srv1_min = val;
        if (nvs_get_i16(handle, NVS_KEY_srv1_max, &val) == ESP_OK) srv1_max = val;
        if (nvs_get_i16(handle, NVS_KEY_srv2_min, &val) == ESP_OK) srv2_min = val;
        if (nvs_get_i16(handle, NVS_KEY_srv2_max, &val) == ESP_OK) srv2_max = val;
        if (nvs_get_i16(handle, NVS_KEY_srv3_min, &val) == ESP_OK) srv3_min = val;
        if (nvs_get_i16(handle, NVS_KEY_srv3_max, &val) == ESP_OK) srv3_max = val;
        nvs_close(handle);
        ESP_LOGI(TAG, "Limites cargados NVS: srv1=%d-%d srv2=%d-%d srv3=%d-%d", srv1_min, srv1_max, srv2_min, srv2_max, srv3_min, srv3_max);
    }
}

void hmi_send_data(uint8_t reg, int32_t value)
{
    hmi_tx_frame_t frame = {
        .reg   = reg,
        .value = value
    };

    if(pdFALSE == xQueueSend(xQueueHmiTx, &frame, pdMS_TO_TICKS(10))) {
        ESP_LOGE(TAG, "xQueueHmiTx llena");
    }
}

void wait_for_sensor(ina260_t *dev)
{
    bool ready, alert, overflow;
    int timeout = 100;

    while (timeout--) 
    {
        ESP_ERROR_CHECK(ina260_get_status(dev, &ready, &alert, &overflow));
        if (alert) ESP_LOGW(TAG, "ALERT! ");
        if (overflow) ESP_LOGW(TAG, "OVERFLOW! ");
        if (ready) break;

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

uint16_t adc_multi_sampling_channel (adc_oneshot_unit_handle_t handle, adc_channel_t chan) 
{
    uint16_t samples = 0;
    int valRaw = 0;

    for(uint8_t i = 1; i <= 10; i++) 
    {
        adc_oneshot_read(handle, chan, &valRaw);
        samples = (samples + (uint16_t)valRaw);
    }

    return ((uint16_t)samples / (uint16_t)10);
}



//*************************************************************************************
//***************************** CONFIGURACION DE HARDWARE *****************************
//*************************************************************************************

void vHardwareInit (void)
{
	vGpioInit();
    vAdcInit();
    vUartInit();
	vI2cInit();
	vMbMasterInit();
    load_center_from_nvs();
    load_limits_from_nvs();

	ESP_LOGI(TAG, "Configuracion de drivers ESP-IDF exitoso");
}

void vGpioInit(void) {	
	gpio_config_t gpio_out_cfg = {
		.pin_bit_mask = DOUT_MASK_CONFIG,
		.mode         = GPIO_MODE_INPUT_OUTPUT,
		.pull_up_en   = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type    = GPIO_INTR_DISABLE,
	};
	gpio_config(&gpio_out_cfg);
	
	gpio_set_level(DOUT1, 0);
	gpio_set_level(DOUT2, 0);
	gpio_set_level(DOUT3, 0);

	gpio_config_t gpio_in_cfg = {
		.pin_bit_mask = DIN_MASK_CONFIG,
		.mode         = GPIO_MODE_INPUT,
		.pull_up_en   = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type    = GPIO_INTR_DISABLE,
	};
	gpio_config(&gpio_in_cfg); 

    gpio_set_pull_mode(J1_DIN1, GPIO_PULLUP_ENABLE);
    gpio_set_pull_mode(J2_DIN0, GPIO_PULLUP_ENABLE);

    gpio_set_intr_type(ENC_A_DIN0, GPIO_INTR_POSEDGE);
    
	gpio_install_isr_service(0); 

    gpio_isr_handler_add(ENC_A_DIN0, encoder_a_isr_handler, NULL);
}

void vI2cInit(void) {
    i2cdev_init();
    memset(&dev_ina260_handle, 0, sizeof(ina260_t));
    ESP_ERROR_CHECK(ina260_init_desc(&dev_ina260_handle, INA_DEV_ADDR, INA_I2C_PORT, INA_I2C_SDA_PIN, INA_I2C_SCL_PIN));

    ESP_ERROR_CHECK(ina260_init(&dev_ina260_handle));
    ESP_ERROR_CHECK(ina260_set_config(&dev_ina260_handle, INA260_MODE_TRIG_SHUNT_BUS, INA260_AVG_128, INA260_CT_1100, INA260_CT_1100));
}

void vAdcInit(void)
{
    adc_oneshot_unit_init_cfg_t adc1_config = {
        .unit_id  = ADC_UNIT_1,
        .clk_src  = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    adc_oneshot_new_unit(&adc1_config, &adc1_handle);

    adc_oneshot_chan_cfg_t channelConfig = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_3, &channelConfig);
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &channelConfig);
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_5, &channelConfig);
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &channelConfig);
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_7, &channelConfig);
}

void vUartInit(void)
{
    uart_config_t uart_config = {
        .baud_rate  = HMI_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .stop_bits  = UART_STOP_BITS_1,
        .parity     = UART_PARITY_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
    };
    uart_param_config(HMI_UART_PORT, &uart_config);

    uart_set_pin(HMI_UART_PORT, HMI_UART_TXD, HMI_UART_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uart_driver_install(HMI_UART_PORT, UART_BUFFER_SIZE, UART_BUFFER_SIZE, 20, &xQueueUartEvent, 0);
}

void vMbMasterInit(void) {
    mb_communication_info_t mb_com_info = {
		.ser_opts.mode             = MB_RTU,
		.ser_opts.port             = MB_UART_PORT,
		.ser_opts.response_tout_ms = 100,
		.ser_opts.baudrate         = MB_MASTER_BAUD,
		.ser_opts.data_bits        = UART_DATA_8_BITS,
		.ser_opts.stop_bits        = UART_STOP_BITS_1,
		.ser_opts.parity           = UART_PARITY_DISABLE
	};

    if (ESP_OK != mbc_master_create_serial(&mb_com_info, &mb_master_handle)){
        ESP_LOGE(TAG, "mb controller initialization fail");
		return;
    }

    if (ESP_OK != uart_set_pin(MB_UART_PORT, MB_UART_TXD, MB_UART_RXD, MB_UART_DIR, UART_PIN_NO_CHANGE)) {
        ESP_LOGE(TAG, "Error al configurar los pines UART");
        return;
    }

    if (ESP_OK != mbc_master_start(mb_master_handle)) {
        ESP_LOGE(TAG, "mb controller start  fail");
		return;
    }

    if(ESP_OK != uart_set_mode(MB_UART_PORT, UART_MODE_RS485_HALF_DUPLEX)){
        ESP_LOGE(TAG, "mb serial set mode failure, uart_set_mode()");
		return;
    }

	ESP_LOGW(TAG, "Modbus RTU Stack inicializado Ok!");
}


/********************************************************************************************************** */
/********************************************************************************************************** */
/********************************************************************************************************** */

int tiltChrAccess(uint16_t connHandle, uint16_t attrHandle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;

    switch (ctxt->op)
    {
        case BLE_GATT_ACCESS_OP_READ_CHR:

            if (connHandle != BLE_HS_CONN_HANDLE_NONE) {
                ESP_LOGI(TAG, "Lectura TILT; conn=%d attr=%d", connHandle, attrHandle);
            }

            if (attrHandle == tiltChrValHandle) {
                rc = os_mbuf_append(ctxt->om, appTilt, sizeof(appTilt));
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            break;

        default:
            break;
    }

    ESP_LOGE(TAG, "Acceso inesperado TILT, op=%d", ctxt->op);
    return BLE_ATT_ERR_UNLIKELY;
}

int battChrAccess(uint16_t connHandle, uint16_t attrHandle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;

    switch (ctxt->op)
    {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            ESP_LOGI(TAG, "Lectura BATTERY");

            if (attrHandle == battChrValHandle) {
                rc = os_mbuf_append(ctxt->om, appBattery, sizeof(appBattery));
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            break;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

int tempChrAccess(uint16_t connHandle, uint16_t attrHandle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;

    switch (ctxt->op)
    {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            ESP_LOGI(TAG, "Lectura TEMP");

            if (attrHandle == tempChrValHandle) {
                rc = os_mbuf_append(ctxt->om, &appTemp, sizeof(appTemp));
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            break;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

int encChrAccess(uint16_t connHandle, uint16_t attrHandle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;

    switch (ctxt->op)
    {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            ESP_LOGI(TAG, "Lectura ENCODER");

            if (attrHandle == encChrValHandle) {
                rc = os_mbuf_append(ctxt->om, &appEncoder, sizeof(appEncoder));
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            break;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

void onStackReset (int reason) 
{
    ESP_LOGI(TAG, "NimBLE stack se ha resetado, razon: %d", reason);
}

void onStackSync (void) 
{
    startAdvertisingInit();
}

void nimbleHostConfigInit (void) 
{
    ble_hs_cfg.reset_cb          = onStackReset;
    ble_hs_cfg.sync_cb           = onStackSync;
    ble_hs_cfg.store_status_cb   = ble_store_util_status_rr;

    ble_store_config_init();                                
}

int gapInit (void) 
{
    int rc = 0;

    ble_svc_gap_init();

    rc = ble_svc_gap_device_name_set(DEVICE_NAME);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error configurando nombre GAP (%s), codigo: %d", DEVICE_NAME, rc);
        return rc;
    }

    return rc;
}

void startAdvertising (void) 
{ 
    int rc = 0;
    const char *name;
    struct ble_hs_adv_fields advFields  = {0};
    struct ble_gap_adv_params advParams = {0};

    advFields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    name = ble_svc_gap_device_name();
    advFields.name = (uint8_t *)name;
    advFields.name_len = strlen(name);
    advFields.name_is_complete = 1;

    advFields.le_role = BLE_GAP_LE_ROLE_PERIPHERAL;
    advFields.le_role_is_present = 1;

    rc = ble_gap_adv_set_fields(&advFields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error configurando advertising, codigo: %d", rc);
        return;
    }
    
    advParams.conn_mode = BLE_GAP_CONN_MODE_UND;
    advParams.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(ownAddrType, NULL, BLE_HS_FOREVER, &advParams, gapEventHandler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error iniciando advertising, codigo: %d", rc);
        return;
    }

    ESP_LOGI(TAG, "Advertising iniciado");
}

void startAdvertisingInit (void) 
{
    int rc = 0;
    
    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "El dispositivo no tiene direccion BLE disponible");
        return;
    }

    rc = ble_hs_id_infer_auto(0, &ownAddrType);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error infiriendo tipo de direccion, codigo: %d", rc);
        return;
    }

    ble_hs_id_copy_addr(ownAddrType, addrVal, NULL);

    int32_t ble_pin = (((int32_t)addrVal[3] << 16) |
                       ((int32_t)addrVal[4] <<  8) |
                        (int32_t)addrVal[5]) % 1000000;

    hmi_send_data(HMI_REG_BLUETOOTH_PASSWORD, ble_pin);
    ESP_LOGI(TAG, "BLE PIN: %06ld", (long)ble_pin);

    startAdvertising();
}

int gapEventHandler (struct ble_gap_event *event, void *arg) 
{
    int rc = 0;                       

    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            ESP_LOGW(TAG, "BLE_GAP_EVENT_CONNECT");

            if (event->connect.status == 0) {
                bleConnHandle = event->connect.conn_handle;

                ESP_LOGI(TAG, "Conexion BLE establecida");

                hmi_send_data(HMI_REG_BLUETOOTH_INDICATOR, (int32_t)255);

            } else {
                ESP_LOGE(TAG, "Conexion BLE con error, reiniciando publicidad");
                startAdvertising();
            }

            return rc;
        
        case BLE_GAP_EVENT_DISCONNECT:
            bleConnHandle = BLE_HS_CONN_HANDLE_NONE;
            ESP_LOGW(TAG, "BLE_GAP_EVENT_DISCONNECT");

            hmi_send_data(HMI_REG_BLUETOOTH_INDICATOR, (int32_t)0);

            startAdvertising();

            return rc;

        case BLE_GAP_EVENT_SUBSCRIBE:
            ESP_LOGW(TAG, "BLE_GAP_EVENT_SUBSCRIBE");
            gattSvrSubscribeCb(event);

            return rc;
    }

    return rc;
}

int gattSvcInit(void) 
{
    int rc;

    ble_svc_gatt_init(); 

    rc = ble_gatts_count_cfg(gattSrvServices); 
    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(gattSrvServices); 
    if (rc != 0) {
        return rc;
    }

    return 0;
}

void gattSvrSubscribeCb (struct ble_gap_event *event)
{
    uint16_t attr_handle = event->subscribe.attr_handle;
    uint16_t conn_handle = event->subscribe.conn_handle;

    if (attr_handle == tiltChrValHandle) {
        tiltNotify.subConnHandle = conn_handle;
        tiltNotify.notifyEnabled = event->subscribe.cur_notify;
    }
    else if (attr_handle == battChrValHandle) {
        battNotify.subConnHandle = conn_handle;
        battNotify.notifyEnabled = event->subscribe.cur_notify;
    }
    else if (attr_handle == tempChrValHandle) {
        tempNotify.subConnHandle = conn_handle;
        tempNotify.notifyEnabled = event->subscribe.cur_notify;
    }
    else if (attr_handle == encChrValHandle) {
        encNotify.subConnHandle = conn_handle;
        encNotify.notifyEnabled = event->subscribe.cur_notify;
    }
}

void vBleServiceInit (void) 
{
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando NVS flash, codigo: %d", ret);
        return;
    }

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando NimBLE stack, codigo: %d", ret);
        return;
    }

    ret = gapInit();
    if (ret != 0) {
        ESP_LOGE(TAG, "Error inicializando servicio GAP, codigo: %d", ret);
        return;
    }

    ret = gattSvcInit();
    if (ret != 0) {
        ESP_LOGE(TAG, "Error inicializando servicio GATT, codigo: %d", ret);
        return;
    }

    nimbleHostConfigInit();
}