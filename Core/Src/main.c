/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : MPU6050 + BMP280 + MAVLink v2 (USB & UART1) + PID + SBUS
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "usbd_cdc_if.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "mavlink/common/mavlink.h"
#pragma GCC diagnostic pop
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float i_limit;    /* Batas maksimal akumulasi Integral (us) */
    float out_limit;  /* Batas maksimal simpangan servo dari 1500 us */
} PID_t;

typedef struct {
    uint8_t identitas;
    uint8_t data_mentah[14];

    int16_t accX_raw, accY_raw, accZ_raw;
    int16_t gyroX_raw, gyroY_raw, gyroZ_raw;
    int16_t temp_raw;

    float accX_g, accY_g, accZ_g;
    float gyroX_dps, gyroY_dps, gyroZ_dps;
    float suhu_c;
    float sudut_accX, sudut_accY;
    float dt;

    float compX, compY, compZ;

    float kalmanX, kalmanY, kalmanZ;
    float err_gyroX, err_gyroY;
    float kalman_gainX, kalman_gainY;
    float Q_drift;
    float E_acc;
    float gyroX_bias, gyroY_bias, gyroZ_bias;

    uint8_t i2c_err_count;
    uint8_t sensor_failsafe;
} MPU6050_t;

typedef struct {
    uint8_t  addr;
    uint8_t  chip_id;
    uint16_t dig_T1, dig_P1;
    int16_t  dig_T2, dig_T3;
    int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

    float    suhu_c;
    float    tekanan_pa;
    float    tekanan_hpa;
    float    tekanan_ref_p0;
    float    ketinggian_m;
    float    ketinggian_filter;
    float    climb_mps;
    uint32_t waktu_baro;
    uint32_t read_ok_count;
    uint32_t read_err_count;
} BMP280_t;

typedef struct {
    PID_t pid_roll;
    PID_t pid_pitch;

    /* Gain Altitude Hold (Outer Loop Barometer -> Target Pitch) & Yaw Damper */
    float kp_alt;
    float kd_alt;
    float kd_yaw;

    /* Mode & Setpoint Kendali */
    uint8_t enable_alt_hold;
    float target_alt;      /* Target ketinggian (meter) */
    float target_roll;     /* Sudut roll yang diminta ke aileron (derajat) */
    float target_pitch;    /* Sudut pitch yang diminta ke elevator (derajat) */

    /* Arah gerak servo (+1.0f atau -1.0f) */
    float dir_roll_L;
    float dir_roll_R;
    float dir_pitch;
    float dir_yaw;

    /* Trim mekanik (us) */
    float trim_roll_L;
    float trim_roll_R;
    float trim_pitch;
    float trim_yaw;

    /* Output PWM akhir (1000 - 2000 us) */
    uint16_t aileron_L;    /* PA6 (TIM3_CH1) - Aileron Kiri */
    uint16_t aileron_R;    /* PA7 (TIM3_CH2) - Aileron Kanan */
    uint16_t elevator;     /* PB0 (TIM3_CH3) - Elevator */
    uint16_t rudder;       /* PB1 (TIM3_CH4) - Rudder */
} PWM_Servo_t;

typedef struct {
    char   name[16];       /* Nama parameter MAVLink (maks 16 karakter) */
    float *val_ptr;        /* Pointer langsung ke variabel di struct pwm */
} MavParamItem_t;

#define MAV_RX_BUF_SIZE 256U
typedef struct {
    uint8_t buf[MAV_RX_BUF_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} MavRingBuf_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU_ADDR          (0x68U << 1)
#define BMP280_ADDR_LOW   (0x76U << 1) /* Pin SDO ke GND */
#define BMP280_ADDR_HIGH  (0x77U << 1) /* Pin SDO ke 3.3V */

#define RAD_TO_DEG        57.295779513f
#define DEG_TO_RAD        0.01745329252f

#define SYS_ID            1U
#define COMP_ID_FC        1U

#define BARO_UPDATE_MS    50U          /* Update barometer 20 Hz */
#define TELEM_UPDATE_MS   50U          /* Kirim telemetri MAVLink USB & Radio 20 Hz */
#define PARAM_STREAM_MS   20U          /* Jeda kirim tiap parameter saat Mission Planner sync */

/* Konstanta SBUS & Mode Terbang */
#define SBUS_FRAME_LEN    25U
#define SBUS_HEADER       0x0FU
#define SBUS_TIMEOUT_MS   100U

#define MODE_MANUAL       0U           /* Kendali penuh stick pilot (tanpa koreksi PID) */
#define MODE_STABILIZE    1U           /* Auto-level: PID menjaga sudut sesuai arah stick */
#define MODE_ALTHOLD      2U           /* Altitude Hold: PID menjaga sudut & ketinggian baro */

/* Custom Mode ID standar ArduPlane agar terbaca di HUD Mission Planner */
#define PLANE_MODE_MANUAL     0U
#define PLANE_MODE_STABILIZE  2U
#define PLANE_MODE_FBWA       5U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* ----------------------- Struct MPU6050, BMP280, PWM -------------------- */
MPU6050_t mpu = {
    .dt        = 0.01f,
    .err_gyroX = 1.0f,
    .err_gyroY = 1.0f,
    .Q_drift   = 0.003f,
    .E_acc     = 2.0f
};

BMP280_t bmp = {
    .tekanan_pa     = 101325.0f,
    .tekanan_hpa    = 1013.25f,
    .tekanan_ref_p0 = 101325.0f
};

PWM_Servo_t pwm = {
    .pid_roll  = { .kp = 7.5f, .ki = 1.2f, .kd = 0.65f, .integral = 0.0f, .i_limit = 120.0f, .out_limit = 400.0f },
    .pid_pitch = { .kp = 8.0f, .ki = 1.2f, .kd = 0.70f, .integral = 0.0f, .i_limit = 120.0f, .out_limit = 400.0f },
    .kp_alt     = 2.0f,
    .kd_alt     = 1.5f,
    .kd_yaw     = 3.0f,
    .dir_roll_L =  1.0f,
    .dir_roll_R = -1.0f,
    .dir_pitch  =  1.0f,
    .dir_yaw    =  1.0f,
    .aileron_L  = 1500U,
    .aileron_R  = 1500U,
    .elevator   = 1500U,
    .rudder     = 1500U
};

/* ---------- Tabel Parameter Live Tuning untuk Mission Planner ----------- */
MavParamItem_t param_table[] = {
    { "ROLL_KP",     &pwm.pid_roll.kp },
    { "ROLL_KI",     &pwm.pid_roll.ki },
    { "ROLL_KD",     &pwm.pid_roll.kd },
    { "ROLL_IMAX",   &pwm.pid_roll.i_limit },
    { "PITCH_KP",    &pwm.pid_pitch.kp },
    { "PITCH_KI",    &pwm.pid_pitch.ki },
    { "PITCH_KD",    &pwm.pid_pitch.kd },
    { "PITCH_IMAX",  &pwm.pid_pitch.i_limit },
    { "ALT_KP",      &pwm.kp_alt },
    { "ALT_KD",      &pwm.kd_alt },
    { "YAW_KD",      &pwm.kd_yaw },
    { "DIR_ROLL_L",  &pwm.dir_roll_L },
    { "DIR_ROLL_R",  &pwm.dir_roll_R },
    { "DIR_PITCH",   &pwm.dir_pitch },
    { "DIR_YAW",     &pwm.dir_yaw },
    { "TRIM_ROLL_L", &pwm.trim_roll_L },
    { "TRIM_ROLL_R", &pwm.trim_roll_R },
    { "TRIM_PITCH",  &pwm.trim_pitch },
    { "TRIM_YAW",    &pwm.trim_yaw }
};
#define PARAM_COUNT ((uint16_t)(sizeof(param_table) / sizeof(param_table[0])))

uint8_t  param_streaming    = 0U;
uint16_t param_stream_idx   = 0U;
uint32_t waktu_param_stream = 0U;

/* ------------------------------- MAVLink -------------------------------- */
uint32_t waktu_sekarang = 0;
uint32_t waktu_sebelumnya = 0;
uint32_t waktu_hb = 0;
uint32_t waktu_led = 0;
uint32_t waktu_telem = 0;

mavlink_message_t msg_hb;
mavlink_message_t msg_att;
mavlink_message_t msg_gpos;
mavlink_message_t msg_alt;
mavlink_message_t msg_vfr;
mavlink_message_t msg_servo;
mavlink_message_t msg_param;

uint8_t usb_tx_buf[MAVLINK_MAX_PACKET_LEN * 6];
uint8_t uart_tx_buf[MAVLINK_MAX_PACKET_LEN * 6];
uint8_t param_usb_buf[MAVLINK_MAX_PACKET_LEN];
uint8_t param_uart_buf[MAVLINK_MAX_PACKET_LEN];

MavRingBuf_t mav_usb_rx  = { .head = 0U, .tail = 0U };
MavRingBuf_t mav_uart_rx = { .head = 0U, .tail = 0U };
uint8_t radio_rx_byte = 0U;

mavlink_message_t rx_msg_usb;
mavlink_status_t  rx_status_usb;
mavlink_message_t rx_msg_uart;
mavlink_status_t  rx_status_uart;

/* ------------------------------- SBUS RC -------------------------------- */
uint8_t  sbus_rx_byte = 0U;
uint8_t  sbus_buf[SBUS_FRAME_LEN];
uint8_t  sbus_idx = 0U;
uint32_t sbus_last_byte_ms = 0U;
uint32_t sbus_last_frame_ms = 0U;

uint16_t sbus_ch[16] = { 992U, 992U, 172U, 992U, 992U };       /* Nilai mentah 172 - 1811 */
uint16_t sbus_us[16] = { 1500U, 1500U, 1000U, 1500U, 1500U };  /* Konversi 1000 - 2000 us */

uint8_t  sbus_failsafe = 1U;
uint8_t  flight_mode   = MODE_STABILIZE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef BMP280_Init(BMP280_t *dev);
float BMP280_ReadPressure(BMP280_t *dev);
void BMP280_UpdateAltitude(BMP280_t *dev);
static void MAVLink_ProcessIncoming(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void MavRingBuf_Push(MavRingBuf_t *rb, uint8_t byte)
{
    uint16_t next = (uint16_t)((rb->head + 1U) % MAV_RX_BUF_SIZE);
    if (next != rb->tail)
    {
        rb->buf[rb->head] = byte;
        rb->head = next;
    }
}

static uint8_t MavRingBuf_Pop(MavRingBuf_t *rb, uint8_t *byte)
{
    if (rb->head == rb->tail)
    {
        return 0U;
    }
    *byte = rb->buf[rb->tail];
    rb->tail = (uint16_t)((rb->tail + 1U) % MAV_RX_BUF_SIZE);
    return 1U;
}

static void MAVLink_SendParamValue(uint16_t idx)
{
    if (idx >= PARAM_COUNT)
    {
        return;
    }

    uint16_t len = mavlink_msg_param_value_pack(
        SYS_ID, COMP_ID_FC, &msg_param,
        param_table[idx].name,
        *param_table[idx].val_ptr,
        MAV_PARAM_TYPE_REAL32,
        PARAM_COUNT,
        idx);

    len = (uint16_t)mavlink_msg_to_send_buffer(param_usb_buf, &msg_param);

    /* Kirim via USB CDC (tunggu sebentar jika endpoint USB sedang sibuk) */
    for (uint8_t retry = 0U; retry < 5U; retry++)
    {
        if (CDC_Transmit_FS(param_usb_buf, len) == USBD_OK)
        {
            break;
        }
        HAL_Delay(1U);
    }

    /* Kirim juga via Radio Telemetri 915 MHz (USART1) */
    memcpy(param_uart_buf, param_usb_buf, len);
    (void)HAL_UART_Transmit(&huart1, param_uart_buf, len, 50U);
}

static int16_t MAVLink_FindParamByName(const char *param_id)
{
    char clean_name[17];
    memset(clean_name, 0, sizeof(clean_name));
    memcpy(clean_name, param_id, 16U);

    for (uint16_t i = 0U; i < PARAM_COUNT; i++)
    {
        if (strncmp(clean_name, param_table[i].name, 16U) == 0)
        {
            return (int16_t)i;
        }
    }
    return -1;
}

static void MAVLink_HandleMessage(const mavlink_message_t *msg)
{
    if (msg->msgid == MAVLINK_MSG_ID_PARAM_REQUEST_LIST)
    {
        mavlink_param_request_list_t req;
        mavlink_msg_param_request_list_decode(msg, &req);

        if ((req.target_system == SYS_ID || req.target_system == 0U) &&
            (req.target_component == COMP_ID_FC || req.target_component == MAV_COMP_ID_ALL))
        {
            param_stream_idx = 0U;
            param_streaming = 1U;
            waktu_param_stream = 0U;
        }
    }
    else if (msg->msgid == MAVLINK_MSG_ID_PARAM_REQUEST_READ)
    {
        mavlink_param_request_read_t req;
        mavlink_msg_param_request_read_decode(msg, &req);

        if ((req.target_system == SYS_ID || req.target_system == 0U) &&
            (req.target_component == COMP_ID_FC || req.target_component == MAV_COMP_ID_ALL))
        {
            if (req.param_index >= 0 && (uint16_t)req.param_index < PARAM_COUNT)
            {
                MAVLink_SendParamValue((uint16_t)req.param_index);
            }
            else
            {
                int16_t idx = MAVLink_FindParamByName(req.param_id);
                if (idx >= 0)
                {
                    MAVLink_SendParamValue((uint16_t)idx);
                }
            }
        }
    }
    else if (msg->msgid == MAVLINK_MSG_ID_PARAM_SET)
    {
        mavlink_param_set_t set;
        mavlink_msg_param_set_decode(msg, &set);

        if ((set.target_system == SYS_ID || set.target_system == 0U) &&
            (set.target_component == COMP_ID_FC || set.target_component == MAV_COMP_ID_ALL))
        {
            int16_t idx = MAVLink_FindParamByName(set.param_id);
            if (idx >= 0 && isfinite(set.param_value))
            {
                *param_table[idx].val_ptr = set.param_value;

                /* Reset integral jika Ki diubah menjadi 0 */
                if (pwm.pid_roll.ki == 0.0f)  pwm.pid_roll.integral = 0.0f;
                if (pwm.pid_pitch.ki == 0.0f) pwm.pid_pitch.integral = 0.0f;

                /* Balas langsung ke Mission Planner sebagai konfirmasi Write Params */
                MAVLink_SendParamValue((uint16_t)idx);
            }
        }
    }
}

static void MAVLink_ProcessIncoming(void)
{
    uint8_t byte = 0U;

    /* 1. Proses paket MAVLink masuk dari USB Type-C */
    while (MavRingBuf_Pop(&mav_usb_rx, &byte) != 0U)
    {
        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &rx_msg_usb, &rx_status_usb) == MAVLINK_FRAMING_OK)
        {
            MAVLink_HandleMessage(&rx_msg_usb);
        }
    }

    /* 2. Proses paket MAVLink masuk dari Radio Telemetri 915 MHz (USART1) */
    while (MavRingBuf_Pop(&mav_uart_rx, &byte) != 0U)
    {
        if (mavlink_parse_char(MAVLINK_COMM_1, byte, &rx_msg_uart, &rx_status_uart) == MAVLINK_FRAMING_OK)
        {
            MAVLink_HandleMessage(&rx_msg_uart);
        }
    }

    /* 3. Kirim daftar parameter satu per satu secara bertahap agar radio tidak penuh */
    if (param_streaming != 0U)
    {
        uint32_t now = HAL_GetTick();
        if ((now - waktu_param_stream) >= PARAM_STREAM_MS)
        {
            waktu_param_stream = now;
            if (param_stream_idx < PARAM_COUNT)
            {
                MAVLink_SendParamValue(param_stream_idx);
                param_stream_idx++;
            }
            else
            {
                param_streaming = 0U;
            }
        }
    }
}

/* Auto-detect alamat BMP280 (0x76 atau 0x77), cek Chip ID, dan baca kalibrasi */
HAL_StatusTypeDef BMP280_Init(BMP280_t *dev)
{
    uint8_t calib[24];
    uint8_t config = 0x10U;     /* IIR filter x16, standby 0.5 ms */
    uint8_t ctrl_meas = 0x57U;  /* Temp x2, Pressure x16, Normal mode */

    if (HAL_I2C_IsDeviceReady(&hi2c1, BMP280_ADDR_LOW, 3U, 100U) == HAL_OK)
    {
        dev->addr = BMP280_ADDR_LOW;
    }
    else if (HAL_I2C_IsDeviceReady(&hi2c1, BMP280_ADDR_HIGH, 3U, 100U) == HAL_OK)
    {
        dev->addr = BMP280_ADDR_HIGH;
    }
    else
    {
        return HAL_ERROR;
    }

    if (HAL_I2C_Mem_Read(&hi2c1, dev->addr, 0xD0U,
                         I2C_MEMADD_SIZE_8BIT, &dev->chip_id, 1U, 100U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 0x58 = BMP280, 0x60 = BME280 */
    if ((dev->chip_id != 0x58U) && (dev->chip_id != 0x60U))
    {
        return HAL_ERROR;
    }

    if (HAL_I2C_Mem_Read(&hi2c1, dev->addr, 0x88U,
                         I2C_MEMADD_SIZE_8BIT, calib, sizeof(calib), 100U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    dev->dig_T1 = (uint16_t)(((uint16_t)calib[1] << 8) | calib[0]);
    dev->dig_T2 = (int16_t)(((uint16_t)calib[3] << 8) | calib[2]);
    dev->dig_T3 = (int16_t)(((uint16_t)calib[5] << 8) | calib[4]);
    dev->dig_P1 = (uint16_t)(((uint16_t)calib[7] << 8) | calib[6]);
    dev->dig_P2 = (int16_t)(((uint16_t)calib[9] << 8) | calib[8]);
    dev->dig_P3 = (int16_t)(((uint16_t)calib[11] << 8) | calib[10]);
    dev->dig_P4 = (int16_t)(((uint16_t)calib[13] << 8) | calib[12]);
    dev->dig_P5 = (int16_t)(((uint16_t)calib[15] << 8) | calib[14]);
    dev->dig_P6 = (int16_t)(((uint16_t)calib[17] << 8) | calib[16]);
    dev->dig_P7 = (int16_t)(((uint16_t)calib[19] << 8) | calib[18]);
    dev->dig_P8 = (int16_t)(((uint16_t)calib[21] << 8) | calib[20]);
    dev->dig_P9 = (int16_t)(((uint16_t)calib[23] << 8) | calib[22]);

    if (dev->dig_P1 == 0U)
    {
        return HAL_ERROR;
    }

    if (HAL_I2C_Mem_Write(&hi2c1, dev->addr, 0xF5U,
                          I2C_MEMADD_SIZE_8BIT, &config, 1U, 100U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (HAL_I2C_Mem_Write(&hi2c1, dev->addr, 0xF4U,
                          I2C_MEMADD_SIZE_8BIT, &ctrl_meas, 1U, 100U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

float BMP280_ReadPressure(BMP280_t *dev)
{
    uint8_t raw[6];

    if (dev->addr == 0U)
    {
        dev->read_err_count++;
        return NAN;
    }

    if (HAL_I2C_Mem_Read(&hi2c1, dev->addr, 0xF7U,
                         I2C_MEMADD_SIZE_8BIT, raw, sizeof(raw), 15U) != HAL_OK)
    {
        dev->read_err_count++;
        return NAN;
    }

    int32_t adc_P = (int32_t)(((uint32_t)raw[0] << 12) |
                              ((uint32_t)raw[1] << 4) | (raw[2] >> 4));
    int32_t adc_T = (int32_t)(((uint32_t)raw[3] << 12) |
                              ((uint32_t)raw[4] << 4) | (raw[5] >> 4));

    int32_t var1 = ((((adc_T >> 3) - ((int32_t)dev->dig_T1 << 1))) *
                    ((int32_t)dev->dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - (int32_t)dev->dig_T1) *
                       ((adc_T >> 4) - (int32_t)dev->dig_T1)) >> 12) *
                    ((int32_t)dev->dig_T3)) >> 14;
    int32_t t_fine = var1 + var2;
    dev->suhu_c = (float)((t_fine * 5 + 128) >> 8) / 100.0f;

    int64_t p_var1 = ((int64_t)t_fine) - 128000;
    int64_t p_var2 = p_var1 * p_var1 * (int64_t)dev->dig_P6;
    p_var2 += (p_var1 * (int64_t)dev->dig_P5) << 17;
    p_var2 += ((int64_t)dev->dig_P4) << 35;
    p_var1 = ((p_var1 * p_var1 * (int64_t)dev->dig_P3) >> 8) +
             ((p_var1 * (int64_t)dev->dig_P2) << 12);
    p_var1 = (((((int64_t)1) << 47) + p_var1) * (int64_t)dev->dig_P1) >> 33;

    if (p_var1 == 0)
    {
        dev->read_err_count++;
        return NAN;
    }

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - p_var2) * 3125) / p_var1;
    p_var1 = ((int64_t)dev->dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    p_var2 = ((int64_t)dev->dig_P8 * p) >> 19;
    p = ((p + p_var1 + p_var2) >> 8) + (((int64_t)dev->dig_P7) << 4);

    dev->tekanan_pa = (float)p / 256.0f;
    dev->tekanan_hpa = dev->tekanan_pa / 100.0f;

    if (!isfinite(dev->tekanan_pa) || dev->tekanan_pa < 30000.0f || dev->tekanan_pa > 120000.0f)
    {
        dev->read_err_count++;
        return NAN;
    }

    dev->read_ok_count++;
    return dev->tekanan_pa;
}

void BMP280_UpdateAltitude(BMP280_t *dev)
{
    static float altitude_previous = 0.0f;
    static uint8_t altitude_initialized = 0U;

    uint32_t now = HAL_GetTick();
    uint32_t elapsed_ms = now - dev->waktu_baro;

    if (elapsed_ms < BARO_UPDATE_MS)
    {
        return;
    }

    dev->waktu_baro = now;
    float pressure = BMP280_ReadPressure(dev);

    if (!isfinite(pressure) || dev->tekanan_ref_p0 <= 0.0f)
    {
        return;
    }

    float ratio = pressure / dev->tekanan_ref_p0;
    if (ratio <= 0.1f || ratio >= 2.0f)
    {
        return;
    }

    dev->ketinggian_m = 44330.0f * (1.0f - powf(ratio, 0.19029495f));

    if (!isfinite(dev->ketinggian_m))
    {
        return;
    }

    if (altitude_initialized == 0U)
    {
        dev->ketinggian_filter = dev->ketinggian_m;
        altitude_previous = dev->ketinggian_filter;
        altitude_initialized = 1U;
        dev->climb_mps = 0.0f;
    }
    else
    {
        float dt_baro = elapsed_ms / 1000.0f;
        dev->ketinggian_filter = (0.85f * dev->ketinggian_filter) + (0.15f * dev->ketinggian_m);
        if (dt_baro > 0.0f)
        {
            dev->climb_mps = (dev->ketinggian_filter - altitude_previous) / dt_baro;
        }
        altitude_previous = dev->ketinggian_filter;
    }
}

static float clamp_f(float val, float min_v, float max_v)
{
    if (val < min_v) return min_v;
    if (val > max_v) return max_v;
    return val;
}

static float PID_Compute(PID_t *pid, float setpoint_deg, float angle_deg, float gyro_dps, float dt_sec)
{
    float error = setpoint_deg - angle_deg;

    /* Deadband 0.3 derajat agar servo tidak berdengung saat pesawat diam datar */
    if (fabsf(error) < 0.3f)
    {
        error = 0.0f;
    }

    /* Akumulasi Integral dengan Anti-Windup */
    pid->integral += pid->ki * error * dt_sec;
    pid->integral = clamp_f(pid->integral, -pid->i_limit, pid->i_limit);

    /* P (sudut) + I (angin konstan) - D (redaman langsung dari gyro) */
    float output = (pid->kp * error) + pid->integral - (pid->kd * gyro_dps);

    return clamp_f(output, -pid->out_limit, pid->out_limit);
}

static uint16_t SBUS_ToMicroseconds(uint16_t val)
{
    if (val <= 172U) return 1000U;
    if (val >= 1811U) return 2000U;
    return (uint16_t)(1000.0f + (((float)(val - 172U) * 1000.0f) / 1639.0f) + 0.5f);
}

static void SBUS_ParseFrame(const uint8_t *buf)
{
    /* Unpack 8 kanal pertama (masing-masing 11-bit) dari byte 1 sampai 11 */
    sbus_ch[0] = (uint16_t)((buf[1]        | (buf[2]  << 8)) & 0x07FFU);
    sbus_ch[1] = (uint16_t)(((buf[2] >> 3) | (buf[3]  << 5)) & 0x07FFU);
    sbus_ch[2] = (uint16_t)(((buf[3] >> 6) | (buf[4]  << 2) | (buf[5] << 10)) & 0x07FFU);
    sbus_ch[3] = (uint16_t)(((buf[5] >> 1) | (buf[6]  << 7)) & 0x07FFU);
    sbus_ch[4] = (uint16_t)(((buf[6] >> 4) | (buf[7]  << 4)) & 0x07FFU);
    sbus_ch[5] = (uint16_t)(((buf[7] >> 7) | (buf[8]  << 1) | (buf[9] << 9)) & 0x07FFU);
    sbus_ch[6] = (uint16_t)(((buf[9] >> 2) | (buf[10] << 6)) & 0x07FFU);
    sbus_ch[7] = (uint16_t)(((buf[10] >> 5)| (buf[11] << 3)) & 0x07FFU);

    for (uint8_t i = 0U; i < 8U; i++)
    {
        sbus_us[i] = SBUS_ToMicroseconds(sbus_ch[i]);
    }

    /* Byte 23 bit ke-3 (0x08) adalah flag failsafe bawaan receiver SBUS */
    sbus_failsafe = ((buf[23] & 0x08U) != 0U) ? 1U : 0U;
    sbus_last_frame_ms = HAL_GetTick();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
    HAL_Delay(1000U);

    /* -------- Inisialisasi MPU6050 -------- */
    if (HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x75U,
                         I2C_MEMADD_SIZE_8BIT, &mpu.identitas, 1U, HAL_MAX_DELAY) != HAL_OK)
    {
        Error_Handler();
    }

    /* Mendukung chip MPU6050 standar (0x68) maupun varian batch 0x72 */
    if ((mpu.identitas != 0x68U) && (mpu.identitas != 0x72U))
    {
        Error_Handler();
    }

    uint8_t data_konfig = 0x00U;
    if (HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x6BU,
                          I2C_MEMADD_SIZE_8BIT, &data_konfig, 1U, HAL_MAX_DELAY) != HAL_OK)
    {
        Error_Handler();
    }

    if ((HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x1CU,
                           I2C_MEMADD_SIZE_8BIT, &data_konfig, 1U, HAL_MAX_DELAY) != HAL_OK) ||
        (HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x1BU,
                           I2C_MEMADD_SIZE_8BIT, &data_konfig, 1U, HAL_MAX_DELAY) != HAL_OK))
    {
        Error_Handler();
    }

    /* -------- Inisialisasi BMP280 -------- */
    if (BMP280_Init(&bmp) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_Delay(100U);

    /* Ambil rata-rata 20 sampel awal sebagai P0 (0.0 meter) */
    float sum_p0 = 0.0f;
    uint8_t valid_samples = 0U;
    for (uint8_t i = 0U; i < 20U; i++)
    {
        float p_sample = BMP280_ReadPressure(&bmp);
        if (isfinite(p_sample))
        {
            sum_p0 += p_sample;
            valid_samples++;
        }
        HAL_Delay(20U);
    }

    if (valid_samples == 0U)
    {
        Error_Handler();
    }

    bmp.tekanan_ref_p0 = sum_p0 / (float)valid_samples;
    bmp.ketinggian_m = 0.0f;
    bmp.ketinggian_filter = 0.0f;
    bmp.climb_mps = 0.0f;

    bmp.waktu_baro = HAL_GetTick();
    waktu_sebelumnya = bmp.waktu_baro;
    waktu_hb = bmp.waktu_baro;
    waktu_led = bmp.waktu_baro;
    waktu_telem = bmp.waktu_baro;

    /* Aktifkan DLPF ~42 Hz pada register 0x1A MPU6050 untuk meredam getaran motor */
    uint8_t dlpf_cfg = 0x03U;
    if (HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x1AU,
                          I2C_MEMADD_SIZE_8BIT, &dlpf_cfg, 1U, HAL_MAX_DELAY) != HAL_OK)
    {
        Error_Handler();
    }

    /* Kalibrasi 500 sampel bias gyro & sudut awal saat pesawat diam (~1 detik) */
    float sum_gx = 0.0f, sum_gy = 0.0f, sum_gz = 0.0f;
    uint16_t gyro_samples = 0U;

    for (uint16_t i = 0U; i < 500U; i++)
    {
        if (HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x3BU,
                             I2C_MEMADD_SIZE_8BIT, mpu.data_mentah, 14U, 5U) == HAL_OK)
        {
            int16_t ax = (int16_t)((mpu.data_mentah[0] << 8) | mpu.data_mentah[1]);
            int16_t ay = (int16_t)((mpu.data_mentah[2] << 8) | mpu.data_mentah[3]);
            int16_t az = (int16_t)((mpu.data_mentah[4] << 8) | mpu.data_mentah[5]);
            int16_t gx = (int16_t)((mpu.data_mentah[8] << 8) | mpu.data_mentah[9]);
            int16_t gy = (int16_t)((mpu.data_mentah[10] << 8) | mpu.data_mentah[11]);
            int16_t gz = (int16_t)((mpu.data_mentah[12] << 8) | mpu.data_mentah[13]);

            sum_gx += (gx / 131.0f);
            sum_gy += (gy / 131.0f);
            sum_gz += (gz / 131.0f);
            gyro_samples++;

            /* Set sudut awal filter mengikuti posisi pesawat saat diam */
            if (gyro_samples == 1U)
            {
                mpu.kalmanX = atan2f((float)ay, (float)az) * RAD_TO_DEG;
                mpu.kalmanY = atan2f((float)(-ax), (float)az) * RAD_TO_DEG;
                mpu.compX = mpu.kalmanX;
                mpu.compY = mpu.kalmanY;
            }
        }
        HAL_Delay(2U);
    }

    if (gyro_samples > 0U)
    {
        mpu.gyroX_bias = sum_gx / (float)gyro_samples;
        mpu.gyroY_bias = sum_gy / (float)gyro_samples;
        mpu.gyroZ_bias = sum_gz / (float)gyro_samples;
    }

    /* Mulai sinyal PWM 50 Hz pada PA6, PA7, PB0, PB1 dan set posisi netral 1500 us */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm.aileron_L);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm.aileron_R);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm.elevator);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm.rudder);

    /* Mulai menerima data SBUS (USART2 PA3) dan perintah MAVLink Radio (USART1 PA10) */
    HAL_UART_Receive_IT(&huart2, &sbus_rx_byte, 1U);
    HAL_UART_Receive_IT(&huart1, &radio_rx_byte, 1U);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        waktu_sekarang = HAL_GetTick();
        mpu.dt = (waktu_sekarang - waktu_sebelumnya) / 1000.0f;
        if (mpu.dt <= 0.001f || mpu.dt > 0.05f)
        {
            mpu.dt = 0.01f;
        }

        /* Proses permintaan / perubahan parameter dari Mission Planner (USB & Radio 915 MHz) */
        MAVLink_ProcessIncoming();

        /* Baca dan filter ketinggian barometer tiap 50 ms (20 Hz) */
        BMP280_UpdateAltitude(&bmp);

        /* Baca 14 byte data MPU6050 dengan timeout 5 ms */
        if (HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x3BU,
                             I2C_MEMADD_SIZE_8BIT, mpu.data_mentah, 14U, 5U) == HAL_OK)
        {
            mpu.i2c_err_count = 0U;
            mpu.sensor_failsafe = 0U;

            mpu.accX_raw  = (int16_t)((mpu.data_mentah[0] << 8) | mpu.data_mentah[1]);
            mpu.accY_raw  = (int16_t)((mpu.data_mentah[2] << 8) | mpu.data_mentah[3]);
            mpu.accZ_raw  = (int16_t)((mpu.data_mentah[4] << 8) | mpu.data_mentah[5]);
            mpu.temp_raw  = (int16_t)((mpu.data_mentah[6] << 8) | mpu.data_mentah[7]);
            mpu.gyroX_raw = (int16_t)((mpu.data_mentah[8] << 8) | mpu.data_mentah[9]);
            mpu.gyroY_raw = (int16_t)((mpu.data_mentah[10] << 8) | mpu.data_mentah[11]);
            mpu.gyroZ_raw = (int16_t)((mpu.data_mentah[12] << 8) | mpu.data_mentah[13]);

            mpu.accX_g = mpu.accX_raw / 16384.0f;
            mpu.accY_g = mpu.accY_raw / 16384.0f;
            mpu.accZ_g = mpu.accZ_raw / 16384.0f;
            mpu.suhu_c = (mpu.temp_raw / 340.0f) + 36.53f;
            mpu.gyroX_dps = (mpu.gyroX_raw / 131.0f) - mpu.gyroX_bias;
            mpu.gyroY_dps = (mpu.gyroY_raw / 131.0f) - mpu.gyroY_bias;
            mpu.gyroZ_dps = (mpu.gyroZ_raw / 131.0f) - mpu.gyroZ_bias;

            /* Complementary Filter */
            mpu.sudut_accX = atan2f(mpu.accY_g, mpu.accZ_g) * RAD_TO_DEG;
            mpu.sudut_accY = atan2f(-mpu.accX_g, mpu.accZ_g) * RAD_TO_DEG;

            mpu.compX = 0.98f * (mpu.compX + (mpu.gyroX_dps * mpu.dt)) + 0.02f * mpu.sudut_accX;
            mpu.compY = 0.98f * (mpu.compY + (mpu.gyroY_dps * mpu.dt)) + 0.02f * mpu.sudut_accY;
            mpu.compZ += mpu.gyroZ_dps * mpu.dt;

            /* Kalman Filter */
            mpu.kalmanX += mpu.gyroX_dps * mpu.dt;
            mpu.err_gyroX += mpu.Q_drift;

            mpu.kalmanY += mpu.gyroY_dps * mpu.dt;
            mpu.err_gyroY += mpu.Q_drift;

            mpu.kalmanZ += mpu.gyroZ_dps * mpu.dt;

            mpu.kalman_gainX = mpu.err_gyroX / (mpu.err_gyroX + mpu.E_acc);
            mpu.kalmanX += mpu.kalman_gainX * (mpu.sudut_accX - mpu.kalmanX);
            mpu.err_gyroX = (1.0f - mpu.kalman_gainX) * mpu.err_gyroX;

            mpu.kalman_gainY = mpu.err_gyroY / (mpu.err_gyroY + mpu.E_acc);
            mpu.kalmanY += mpu.kalman_gainY * (mpu.sudut_accY - mpu.kalmanY);
            mpu.err_gyroY = (1.0f - mpu.kalman_gainY) * mpu.err_gyroY;

            /* ---------------- BACA MODE TERBANG & SETPOINT SBUS ---------------- */
            static uint8_t prev_mode = MODE_STABILIZE;

            /* Jika sinyal SBUS hilang > 100 ms atau flag failsafe aktif, paksa Auto-Level datar */
            if ((waktu_sekarang - sbus_last_frame_ms) > SBUS_TIMEOUT_MS || sbus_failsafe != 0U)
            {
                sbus_failsafe = 1U;
                flight_mode = MODE_STABILIZE;
                pwm.target_roll = 0.0f;
                pwm.target_pitch = 0.0f;
                pwm.enable_alt_hold = 0U;
            }
            else
            {
                /* CH5 (Sakelar 3-Posisi): <1300 = Manual, 1300..1700 = Stabilize, >1700 = Alt Hold */
                if (sbus_us[4] < 1300U)
                {
                    flight_mode = MODE_MANUAL;
                    pwm.enable_alt_hold = 0U;
                }
                else if (sbus_us[4] < 1700U)
                {
                    flight_mode = MODE_STABILIZE;
                    pwm.enable_alt_hold = 0U;
                }
                else
                {
                    flight_mode = MODE_ALTHOLD;
                    pwm.enable_alt_hold = 1U;
                    /* Kunci ketinggian saat ini sebagai target tepat ketika sakelar masuk Alt Hold */
                    if (prev_mode != MODE_ALTHOLD)
                    {
                        pwm.target_alt = bmp.ketinggian_filter;
                    }
                }
            }
            prev_mode = flight_mode;

            /* ---------------- HITUNG OUTPUT SERVO BERDASARKAN MODE ---------------- */
            float out_roll = 0.0f;
            float out_pitch = 0.0f;
            float out_yaw = 0.0f;

            if (flight_mode == MODE_MANUAL)
            {
                /* Reset integral PID saat mode manual agar tidak menumpuk (anti-windup) */
                pwm.pid_roll.integral = 0.0f;
                pwm.pid_pitch.integral = 0.0f;

                /* Teruskan gerakan stick langsung ke servo (-400 .. +400 us) */
                out_roll  = ((float)sbus_us[0] - 1500.0f) * 0.8f;
                out_pitch = ((float)sbus_us[1] - 1500.0f) * 0.8f;
                out_yaw   = ((float)sbus_us[3] - 1500.0f) * 0.8f;
            }
            else
            {
                /* Deadband stick 15 us di sekitar titik tengah (1500 us) */
                float stick_roll  = (fabsf((float)sbus_us[0] - 1500.0f) < 15.0f) ? 0.0f : ((float)sbus_us[0] - 1500.0f);
                float stick_pitch = (fabsf((float)sbus_us[1] - 1500.0f) < 15.0f) ? 0.0f : ((float)sbus_us[1] - 1500.0f);
                float stick_yaw   = (fabsf((float)sbus_us[3] - 1500.0f) < 15.0f) ? 0.0f : ((float)sbus_us[3] - 1500.0f);

                /* Konversi stick ke sudut target (Roll maks +-35 deg, Pitch maks +-30 deg) */
                pwm.target_roll = (stick_roll / 500.0f) * 35.0f;

                if (pwm.enable_alt_hold != 0U)
                {
                    float err_alt = pwm.target_alt - bmp.ketinggian_filter;
                    pwm.target_pitch = clamp_f((pwm.kp_alt * err_alt) - (pwm.kd_alt * bmp.climb_mps), -10.0f, 10.0f);
                }
                else
                {
                    pwm.target_pitch = (stick_pitch / 500.0f) * 30.0f;
                }

                out_roll  = PID_Compute(&pwm.pid_roll,  pwm.target_roll,  mpu.kalmanX, mpu.gyroX_dps, mpu.dt);
                out_pitch = PID_Compute(&pwm.pid_pitch, pwm.target_pitch, mpu.kalmanY, mpu.gyroY_dps, mpu.dt);
                out_yaw   = clamp_f((stick_yaw * 0.7f) - (pwm.kd_yaw * mpu.gyroZ_dps), -350.0f, 350.0f);
            }

            pwm.aileron_L = (uint16_t)(clamp_f(1500.0f + pwm.trim_roll_L + (pwm.dir_roll_L * out_roll),  1000.0f, 2000.0f) + 0.5f);
            pwm.aileron_R = (uint16_t)(clamp_f(1500.0f + pwm.trim_roll_R + (pwm.dir_roll_R * out_roll),  1000.0f, 2000.0f) + 0.5f);
            pwm.elevator  = (uint16_t)(clamp_f(1500.0f + pwm.trim_pitch  + (pwm.dir_pitch  * out_pitch), 1000.0f, 2000.0f) + 0.5f);
            pwm.rudder    = (uint16_t)(clamp_f(1500.0f + pwm.trim_yaw    + (pwm.dir_yaw    * out_yaw),   1000.0f, 2000.0f) + 0.5f);

            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm.aileron_L); /* PA6 - Aileron Kiri */
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm.aileron_R); /* PA7 - Aileron Kanan */
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm.elevator);  /* PB0 - Elevator */
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm.rudder);    /* PB1 - Rudder (opsional) */
        }
        else
        {
            /* Failsafe: jika baca I2C gagal 3x berturut-turut, kembalikan servo ke netral */
            if (mpu.i2c_err_count < 255U)
            {
                mpu.i2c_err_count++;
            }
            if (mpu.i2c_err_count >= 3U)
            {
                mpu.sensor_failsafe = 1U;
                pwm.pid_roll.integral = 0.0f;
                pwm.pid_pitch.integral = 0.0f;

                pwm.aileron_L = 1500U;
                pwm.aileron_R = 1500U;
                pwm.elevator  = 1500U;
                pwm.rudder    = 1500U;

                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1500U);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1500U);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1500U);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1500U);
            }
        }

        /* ---------------- PAKET TELEMETRI MAVLINK V2 (20 Hz) ---------------- */
        if ((waktu_sekarang - waktu_telem) >= TELEM_UPDATE_MS)
        {
            waktu_telem = waktu_sekarang;
            uint16_t total_len = 0U;

            /* 1. HEARTBEAT (1 Hz) dengan identitas ArduPlane agar HUD Mission Planner aktif */
            if ((waktu_sekarang - waktu_hb) >= 1000U)
            {
                waktu_hb = waktu_sekarang;
                uint32_t custom_mode = PLANE_MODE_STABILIZE;
                if (flight_mode == MODE_MANUAL)
                {
                    custom_mode = PLANE_MODE_MANUAL;
                }
                else if (flight_mode == MODE_ALTHOLD)
                {
                    custom_mode = PLANE_MODE_FBWA;
                }

                mavlink_msg_heartbeat_pack(
                    SYS_ID, COMP_ID_FC, &msg_hb,
                    MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_ARDUPILOTMEGA,
                    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED,
                    custom_mode, MAV_STATE_ACTIVE);
                total_len += (uint16_t)mavlink_msg_to_send_buffer(&usb_tx_buf[total_len], &msg_hb);
            }

            /* 2. ATTITUDE (Roll, Pitch, Yaw) */
            float gx_rad = mpu.gyroX_dps * DEG_TO_RAD;
            float gy_rad = mpu.gyroY_dps * DEG_TO_RAD;
            float gz_rad = mpu.gyroZ_dps * DEG_TO_RAD;

            mavlink_msg_attitude_pack(
                SYS_ID, COMP_ID_FC, &msg_att, waktu_sekarang,
                mpu.kalmanX * DEG_TO_RAD, mpu.kalmanY * DEG_TO_RAD, mpu.kalmanZ * DEG_TO_RAD,
                gx_rad, gy_rad, gz_rad);
            total_len += (uint16_t)mavlink_msg_to_send_buffer(&usb_tx_buf[total_len], &msg_att);

            /* 3. GLOBAL_POSITION_INT (Sumber utama HUD Altitude & rel_alt di Mission Planner) */
            float heading_deg_f = fmodf(mpu.kalmanZ, 360.0f);
            if (heading_deg_f < 0.0f) heading_deg_f += 360.0f;
            int16_t  heading_deg  = (int16_t)heading_deg_f;
            uint16_t heading_cdeg = (uint16_t)(heading_deg_f * 100.0f);

            int32_t alt_mm = (int32_t)(bmp.ketinggian_filter * 1000.0f);
            int16_t vz_cms = (int16_t)(-bmp.climb_mps * 100.0f); /* NED: positif ke bawah */

            mavlink_msg_global_position_int_pack(
                SYS_ID, COMP_ID_FC, &msg_gpos, waktu_sekarang,
                0, 0, alt_mm, alt_mm,
                0, 0, vz_cms, heading_cdeg);
            total_len += (uint16_t)mavlink_msg_to_send_buffer(&usb_tx_buf[total_len], &msg_gpos);

            /* 4. ALTITUDE (#141 - Mendukung pembacaan barometer & ketinggian relatif GCS modern) */
            mavlink_msg_altitude_pack(
                SYS_ID, COMP_ID_FC, &msg_alt,
                (uint64_t)waktu_sekarang * 1000ULL,
                bmp.ketinggian_filter,
                bmp.ketinggian_filter,
                bmp.ketinggian_filter,
                bmp.ketinggian_filter,
                bmp.ketinggian_filter,
                bmp.ketinggian_filter);
            total_len += (uint16_t)mavlink_msg_to_send_buffer(&usb_tx_buf[total_len], &msg_alt);

            /* 5. VFR_HUD (Climb rate, heading, dan fallback altitude) */
            mavlink_msg_vfr_hud_pack(
                SYS_ID, COMP_ID_FC, &msg_vfr,
                0.0f, 0.0f, heading_deg, 0U,
                bmp.ketinggian_filter, bmp.climb_mps);
            total_len += (uint16_t)mavlink_msg_to_send_buffer(&usb_tx_buf[total_len], &msg_vfr);

            /* 6. SERVO_OUTPUT_RAW (Pulsa PWM 1000 - 2000 us) */
            mavlink_msg_servo_output_raw_pack(
                SYS_ID, COMP_ID_FC, &msg_servo,
                waktu_sekarang * 1000U, 0U,
                pwm.aileron_L, pwm.aileron_R, pwm.elevator, pwm.rudder,
                0U, 0U, 0U, 0U,
                0U, 0U, 0U, 0U,
                0U, 0U, 0U, 0U);
            total_len += (uint16_t)mavlink_msg_to_send_buffer(&usb_tx_buf[total_len], &msg_servo);

            if (total_len > 0U)
            {
                /* Kirim ke PC via USB CDC */
                (void)CDC_Transmit_FS(usb_tx_buf, total_len);

                /* Salin ke buffer khusus UART1 agar tidak bertabrakan dengan DMA USB */
                memcpy(uart_tx_buf, usb_tx_buf, total_len);
                (void)HAL_UART_Transmit(&huart1, uart_tx_buf, total_len, 50U);
            }
        }

        /* Kedipkan LED PC13 setiap 250 ms sebagai tanda sistem berjalan normal */
        if ((waktu_sekarang - waktu_led) >= 250U)
        {
            waktu_led = waktu_sekarang;
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        }

        HAL_Delay(10U);
        waktu_sebelumnya = waktu_sekarang;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 1500;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 100000;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_2;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void MAVLink_PushRxBytes(const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0U; i < len; i++)
    {
        MavRingBuf_Push(&mav_usb_rx, buf[i]);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        uint32_t now = HAL_GetTick();

        /* Jika jeda antar-byte > 3 ms, berarti paket baru dimulai dari byte ke-0 */
        if ((now - sbus_last_byte_ms) > 3U)
        {
            sbus_idx = 0U;
        }
        sbus_last_byte_ms = now;

        if (sbus_idx == 0U)
        {
            if (sbus_rx_byte == SBUS_HEADER)
            {
                sbus_buf[sbus_idx++] = sbus_rx_byte;
            }
        }
        else
        {
            sbus_buf[sbus_idx++] = sbus_rx_byte;
            if (sbus_idx >= SBUS_FRAME_LEN)
            {
                SBUS_ParseFrame(sbus_buf);
                sbus_idx = 0U;
            }
        }

        HAL_UART_Receive_IT(&huart2, &sbus_rx_byte, 1U);
    }
    else if (huart->Instance == USART1)
    {
        MavRingBuf_Push(&mav_uart_rx, radio_rx_byte);
        HAL_UART_Receive_IT(&huart1, &radio_rx_byte, 1U);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        sbus_idx = 0U;
        HAL_UART_Receive_IT(&huart2, &sbus_rx_byte, 1U);
    }
    else if (huart->Instance == USART1)
    {
        HAL_UART_Receive_IT(&huart1, &radio_rx_byte, 1U);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
