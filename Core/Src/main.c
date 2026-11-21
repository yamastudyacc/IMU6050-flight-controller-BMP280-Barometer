/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Anti-Jitter Fixed-Wing Flight Controller
  *                   Manual + Stabilize + Auto (AltHold) + Gimbal Jitter Filter
  *                   + Median-3 SBUS/CRSF + MAVLink v2 Telemetry
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
    uint8_t identitas;
    uint8_t ready;
    uint8_t data_mentah[14];

    int16_t accX_raw, accY_raw, accZ_raw;
    int16_t gyroX_raw, gyroY_raw, gyroZ_raw;

    float accX_g, accY_g, accZ_g;
    float gyroX_dps, gyroY_dps, gyroZ_dps;
    float gyroX_filt, gyroY_filt, gyroZ_filt;
    float sudut_accX, sudut_accY;
    float dt;

    float kalmanX, kalmanY, kalmanZ;
    float roll_filt, pitch_filt;
    float err_gyroX, err_gyroY;
    float kalman_gainX, kalman_gainY;
    float Q_drift;
    float E_acc;
    float gyroX_bias, gyroY_bias, gyroZ_bias;
} MPU6050_t;

typedef struct {
    uint8_t  addr;
    uint8_t  chip_id;
    uint8_t  ready;
    uint16_t dig_T1, dig_P1;
    int16_t  dig_T2, dig_T3;
    int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

    float    suhu_c;
    float    tekanan_pa;
    float    tekanan_ref_p0;
    float    ketinggian_m;
    float    ketinggian_filter;
    float    climb_mps;
    uint32_t waktu_baro;
} BMP280_t;

typedef struct {
    /* Gain PID Attitude & Altitude (dapat diubah langsung lewat Mission Planner) */
    float kp_roll;
    float ki_roll;
    float kd_roll;

    float kp_pitch;
    float ki_pitch;
    float kd_pitch;

    float kp_yaw;

    float kp_alt;
    float ki_alt;

    /* State internal PID */
    float i_roll;
    float i_pitch;
    float i_alt;
    float target_alt_m;

    /* Output halus PID dalam satuan us (-500 s/d +500) */
    float smooth_roll_us;
    float smooth_pitch_us;
    float smooth_yaw_us;
} AutoPID_t;

typedef struct {
    /* Arah gerak servo (+1.0f normal, -1.0f dibalik/reverse) */
    float dir_roll_L;
    float dir_roll_R;
    float dir_pitch;
    float dir_yaw;

    /* Trim titik tengah mekanik (us) */
    float trim_roll_L;
    float trim_roll_R;
    float trim_pitch;
    float trim_yaw;

    /* Output PWM akhir ke servo (1000 - 2000 us) */
    uint16_t aileron_L;    /* PA6 (TIM3_CH1) - Aileron Kiri */
    uint16_t aileron_R;    /* PA7 (TIM3_CH2) - Aileron Kanan */
    uint16_t elevator;     /* PB0 (TIM3_CH3) - Elevator */
    uint16_t rudder;       /* PB1 (TIM3_CH4) - Rudder */
} PWM_Servo_t;

typedef struct {
    char   name[16];
    float *val_ptr;
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
#define MPU_ADDR              (0x68U << 1)
#define BMP280_ADDR_LOW       (0x76U << 1)
#define BMP280_ADDR_HIGH      (0x77U << 1)

#define RAD_TO_DEG            57.295779513f
#define DEG_TO_RAD            0.01745329252f

#define SYS_ID                1U
#define COMP_ID_FC            1U

#define BARO_UPDATE_MS        50U          /* Update barometer 20 Hz */
#define TELEM_UPDATE_MS       50U          /* Kirim telemetri MAVLink USB & Radio 20 Hz */
#define PARAM_STREAM_MS       20U          /* Jeda kirim parameter ke Mission Planner */
#define RC_DIAG_MS            2000U        /* Kirim status mode & remote setiap 2 detik */

/* Konstanta Receiver (Inverted SBUS, Uninverted SBUS, & CRSF) */
#define RC_PROTO_SBUS_INV     0U           /* Inverted SBUS 100k baud */
#define RC_PROTO_SBUS_NORM    1U           /* Uninverted SBUS 100k baud */
#define RC_PROTO_CRSF         2U           /* CRSF 420k baud (ELRS) */
#define RC_PROTO_COUNT        3U
#define RC_PROTO_SWITCH_MS    800U         /* Jeda pencarian protokol sebelum terkunci */

#define SBUS_FRAME_LEN        25U
#define SBUS_HEADER           0x0FU
#define CRSF_MAX_FRAME_LEN    64U
#define CRSF_FRAMETYPE_RC     0x16U

#define RC_HOLD_TIMEOUT_MS    300U         /* Tahan posisi servo terakhir 300 ms saat paket drop */
#define STICK_DEADBAND_US     18.0f        /* Redam jitter potensiometer stick di titik tengah */
#define SERVO_DEADBAND_US     4            /* Abaikan perubahan <= 4 us agar servo diam total */
#define MAV_RC_TIMEOUT_MS     800U

/* ArduPlane Custom Mode IDs */
#define PLANE_MODE_MANUAL     0U
#define PLANE_MODE_STABILIZE  2U
#define PLANE_MODE_AUTO       10U
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
MPU6050_t mpu = {
    .dt        = 0.01f,
    .err_gyroX = 1.0f,
    .err_gyroY = 1.0f,
    .Q_drift   = 0.003f,
    .E_acc     = 2.0f
};

BMP280_t bmp = {
    .tekanan_pa     = 101325.0f,
    .tekanan_ref_p0 = 101325.0f
};

AutoPID_t pid = {
    .kp_roll      = 5.5f,
    .ki_roll      = 0.6f,
    .kd_roll      = 0.35f,
    .kp_pitch     = 6.0f,
    .ki_pitch     = 0.6f,
    .kd_pitch     = 0.35f,
    .kp_yaw       = 0.45f,
    .kp_alt       = 2.0f,
    .ki_alt       = 0.20f,
    .target_alt_m = 0.0f
};

PWM_Servo_t pwm = {
    .dir_roll_L  =  1.0f,
    .dir_roll_R  =  1.0f,
    .dir_pitch   = -1.0f,
    .dir_yaw     =  1.0f,
    .trim_roll_L =  0.0f,
    .trim_roll_R =  0.0f,
    .trim_pitch  =  0.0f,
    .trim_yaw    =  0.0f,
    .aileron_L   = 1500U,
    .aileron_R   = 1500U,
    .elevator    = 1500U,
    .rudder      = 1500U
};

MavParamItem_t param_table[] = {
    { "DIR_ROLL_L",  &pwm.dir_roll_L },
    { "DIR_ROLL_R",  &pwm.dir_roll_R },
    { "DIR_PITCH",   &pwm.dir_pitch },
    { "DIR_YAW",     &pwm.dir_yaw },
    { "TRIM_ROLL_L", &pwm.trim_roll_L },
    { "TRIM_ROLL_R", &pwm.trim_roll_R },
    { "TRIM_PITCH",  &pwm.trim_pitch },
    { "TRIM_YAW",    &pwm.trim_yaw },
    { "KP_ROLL",     &pid.kp_roll },
    { "KI_ROLL",     &pid.ki_roll },
    { "KD_ROLL",     &pid.kd_roll },
    { "KP_PITCH",    &pid.kp_pitch },
    { "KI_PITCH",    &pid.ki_pitch },
    { "KD_PITCH",    &pid.kd_pitch },
    { "KP_YAW",      &pid.kp_yaw },
    { "KP_ALT",      &pid.kp_alt },
    { "KI_ALT",      &pid.ki_alt }
};
#define PARAM_COUNT ((uint16_t)(sizeof(param_table) / sizeof(param_table[0])))

uint8_t  flight_mode        = PLANE_MODE_MANUAL;
uint8_t  prev_flight_mode   = PLANE_MODE_MANUAL;
uint16_t ch5_last_us        = 1000U;
uint8_t  ch5_initialized    = 0U;

float    filt_stick_roll_us  = 0.0f;
float    filt_stick_pitch_us = 0.0f;
float    filt_stick_yaw_us   = 0.0f;

uint8_t  param_streaming    = 0U;
uint16_t param_stream_idx   = 0U;
uint32_t waktu_param_stream = 0U;

/* ------------------------------- MAVLink -------------------------------- */
uint32_t waktu_sekarang = 0U;
uint32_t waktu_sebelumnya = 0U;
uint32_t waktu_hb = 0U;
uint32_t waktu_led = 0U;
uint32_t waktu_telem = 0U;
uint32_t waktu_rc_diag = 0U;

mavlink_message_t msg_hb;
mavlink_message_t msg_att;
mavlink_message_t msg_gpos;
mavlink_message_t msg_vfr;
mavlink_message_t msg_servo;
mavlink_message_t msg_rc;
mavlink_message_t msg_param;
mavlink_message_t msg_ack;
mavlink_message_t msg_statustext;

uint8_t usb_tx_buf[MAVLINK_MAX_PACKET_LEN * 8];
uint8_t uart_tx_buf[MAVLINK_MAX_PACKET_LEN * 8];
uint8_t ctrl_usb_buf[MAVLINK_MAX_PACKET_LEN];
uint8_t ctrl_uart_buf[MAVLINK_MAX_PACKET_LEN];

MavRingBuf_t mav_usb_rx  = { .head = 0U, .tail = 0U };
MavRingBuf_t mav_uart_rx = { .head = 0U, .tail = 0U };
uint8_t radio_rx_byte = 0U;

mavlink_message_t rx_msg_usb;
mavlink_status_t  rx_status_usb;
mavlink_message_t rx_msg_uart;
mavlink_status_t  rx_status_uart;

/* ---------------- Receiver Inverted-SBUS / SBUS / CRSF 16-Channel ------- */
volatile uint32_t rc_total_bytes      = 0U;
volatile uint32_t rc_valid_frames     = 0U;
volatile uint32_t rc_uart_errors      = 0U;
volatile uint32_t rc_glitches_blocked = 0U;
volatile uint8_t  last_raw_bytes[4]   = {0};
volatile uint8_t  raw_ring_idx        = 0U;

uint8_t  usart2_irq_available         = 0U;
uint8_t  rc_protocol_mode             = RC_PROTO_SBUS_INV;
uint8_t  rc_proto_locked              = 0U;
uint32_t rc_last_switch_ms            = 0U;

uint8_t  sbus_rx_byte                 = 0U;
uint8_t  rc_buf[CRSF_MAX_FRAME_LEN];
volatile uint8_t  rc_idx              = 0U;
volatile uint8_t  rc_frame_sync_ready = 1U;
uint8_t  crsf_expected_len            = 0U;
uint32_t rc_last_byte_cycles          = 0U;
volatile uint32_t rc_last_frame_ms    = 0U;

uint16_t sbus_ch[16] = {
    992U, 992U, 172U, 992U, 172U, 992U, 992U, 992U,
    992U, 992U, 992U, 992U, 992U, 992U, 992U, 992U
};
volatile uint16_t sbus_us[16] = {
    1500U, 1500U, 1000U, 1500U, 1000U, 1500U, 1500U, 1500U,
    1500U, 1500U, 1500U, 1500U, 1500U, 1500U, 1500U, 1500U
};
uint16_t sbus_hist[16][3];
uint8_t  rc_first_frame_done  = 0U;

uint16_t mav_rc_us[16]      = {
    1500U, 1500U, 1000U, 1500U, 1000U, 1500U, 1500U, 1500U,
    1500U, 1500U, 1500U, 1500U, 1500U, 1500U, 1500U, 1500U
};
uint16_t mav_rc_active_mask = 0U;
uint32_t mav_rc_last_ms     = 0U;

uint16_t active_rc_us[16]   = {
    1500U, 1500U, 1000U, 1500U, 1000U, 1500U, 1500U, 1500U,
    1500U, 1500U, 1500U, 1500U, 1500U, 1500U, 1500U, 1500U
};

volatile uint8_t sbus_failsafe = 1U;
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

static float clamp_f(float val, float min_v, float max_v)
{
    if (val < min_v) return min_v;
    if (val > max_v) return max_v;
    return val;
}

static uint16_t clamp_u16(uint16_t val, uint16_t min_v, uint16_t max_v)
{
    if (val < min_v) return min_v;
    if (val > max_v) return max_v;
    return val;
}

static float apply_deadband_f(float val, float db)
{
    if (val > db)  return val - db;
    if (val < -db) return val + db;
    return 0.0f;
}

static float shape_stick_us(uint16_t raw_us, float deadband_us)
{
    float delta = (float)raw_us - 1500.0f;
    if (delta > deadband_us)
    {
        return ((delta - deadband_us) * 500.0f) / (500.0f - deadband_us);
    }
    if (delta < -deadband_us)
    {
        return ((delta + deadband_us) * 500.0f) / (500.0f - deadband_us);
    }
    return 0.0f;
}

static uint16_t apply_deadband_u16(uint16_t prev_val, uint16_t new_val, int32_t db)
{
    int32_t diff = (int32_t)new_val - (int32_t)prev_val;
    if (diff < 0) diff = -diff;
    return (diff <= db) ? prev_val : new_val;
}

static uint16_t median3_u16(uint16_t a, uint16_t b, uint16_t c)
{
    if (a > b) { uint16_t t = a; a = b; b = t; }
    if (b > c) { uint16_t t = b; b = c; c = t; }
    if (a > b) { uint16_t t = a; a = b; b = t; }
    return b;
}

static uint16_t norm1000_to_us(int16_t val)
{
    if (val <= -1000) return 1000U;
    if (val >= 1000)  return 2000U;
    return (uint16_t)(1500 + (val / 2));
}

static void AutoPID_Reset(float current_alt_m, float init_roll_us, float init_pitch_us, float init_yaw_us)
{
    pid.i_roll          = 0.0f;
    pid.i_pitch         = 0.0f;
    pid.i_alt           = 0.0f;
    pid.target_alt_m    = current_alt_m;
    pid.smooth_roll_us  = init_roll_us;
    pid.smooth_pitch_us = init_pitch_us;
    pid.smooth_yaw_us   = init_yaw_us;
}

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

static uint16_t RC_ToMicroseconds(uint16_t val)
{
    if (val <= 172U) return 1000U;
    if (val >= 1811U) return 2000U;
    return (uint16_t)(1000.0f + (((float)(val - 172U) * 1000.0f) / 1639.0f) + 0.5f);
}

static void RC_Unpack16Channels(const uint8_t *p, uint16_t ch[16])
{
    ch[0]  = (uint16_t)((p[0]       | (p[1]  << 8)) & 0x07FFU);
    ch[1]  = (uint16_t)(((p[1] >> 3)| (p[2]  << 5)) & 0x07FFU);
    ch[2]  = (uint16_t)(((p[2] >> 6)| (p[3]  << 2) | (p[4]  << 10)) & 0x07FFU);
    ch[3]  = (uint16_t)(((p[4] >> 1)| (p[5]  << 7)) & 0x07FFU);
    ch[4]  = (uint16_t)(((p[5] >> 4)| (p[6]  << 4)) & 0x07FFU);
    ch[5]  = (uint16_t)(((p[6] >> 7)| (p[7]  << 1) | (p[8]  << 9))  & 0x07FFU);
    ch[6]  = (uint16_t)(((p[8] >> 2)| (p[9]  << 6)) & 0x07FFU);
    ch[7]  = (uint16_t)(((p[9] >> 5)| (p[10] << 3)) & 0x07FFU);
    ch[8]  = (uint16_t)((p[11]      | (p[12] << 8)) & 0x07FFU);
    ch[9]  = (uint16_t)(((p[12]>> 3)| (p[13] << 5)) & 0x07FFU);
    ch[10] = (uint16_t)(((p[13]>> 6)| (p[14] << 2) | (p[15] << 10)) & 0x07FFU);
    ch[11] = (uint16_t)(((p[15]>> 1)| (p[16] << 7)) & 0x07FFU);
    ch[12] = (uint16_t)(((p[16]>> 4)| (p[17] << 4)) & 0x07FFU);
    ch[13] = (uint16_t)(((p[17]>> 7)| (p[18] << 1) | (p[19] << 9))  & 0x07FFU);
    ch[14] = (uint16_t)(((p[19]>> 2)| (p[20] << 6)) & 0x07FFU);
    ch[15] = (uint16_t)(((p[20]>> 5)| (p[21] << 3)) & 0x07FFU);
}

/* Filter Median 3-Sampel: menolak lonjakan 1-frame dari SBUS/CRSF */
static void RC_CommitFilteredChannels(const uint16_t raw_ch[16])
{
    for (uint8_t i = 0U; i < 16U; i++)
    {
        uint16_t new_us = RC_ToMicroseconds(raw_ch[i]);

        if (rc_first_frame_done == 0U)
        {
            sbus_hist[i][0] = new_us;
            sbus_hist[i][1] = new_us;
            sbus_hist[i][2] = new_us;
            sbus_us[i]      = new_us;
        }
        else
        {
            sbus_hist[i][0] = sbus_hist[i][1];
            sbus_hist[i][1] = sbus_hist[i][2];
            sbus_hist[i][2] = new_us;

            uint16_t med_us = median3_u16(sbus_hist[i][0], sbus_hist[i][1], sbus_hist[i][2]);
            int32_t spike = (int32_t)new_us - (int32_t)med_us;
            if (spike < 0) spike = -spike;
            if (spike > 20)
            {
                rc_glitches_blocked++;
            }

            sbus_us[i] = med_us;
        }
    }
    rc_first_frame_done = 1U;
}

static uint8_t CRSF_Crc8(const uint8_t *ptr, uint8_t len)
{
    uint8_t crc = 0U;
    for (uint8_t i = 0U; i < len; i++)
    {
        crc ^= ptr[i];
        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x80U) != 0U)
            {
                crc = (uint8_t)((crc << 1) ^ 0xD5U);
            }
            else
            {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

static void SBUS_ParseFrame(const uint8_t *buf)
{
    if (buf[0] != SBUS_HEADER || (buf[24] != 0x00U && (buf[24] & 0x0FU) != 0x04U))
    {
        return;
    }

    if ((buf[23] & 0x0CU) != 0U)
    {
        if ((buf[23] & 0x08U) != 0U)
        {
            sbus_failsafe = 1U;
        }
        return;
    }

    RC_Unpack16Channels(&buf[1], sbus_ch);
    RC_CommitFilteredChannels(sbus_ch);

    sbus_failsafe = 0U;
    rc_proto_locked = 1U;
    rc_valid_frames++;
    rc_last_frame_ms = HAL_GetTick();
}

static void CRSF_ParseFrame(const uint8_t *buf, uint8_t total_len)
{
    uint8_t payload_plus_type_len = buf[1];
    if (total_len < 4U || payload_plus_type_len < 2U)
    {
        return;
    }

    uint8_t calc_crc = CRSF_Crc8(&buf[2], (uint8_t)(payload_plus_type_len - 1U));
    if (calc_crc != buf[total_len - 1U])
    {
        return;
    }

    rc_proto_locked = 1U;
    rc_valid_frames++;
    rc_last_frame_ms = HAL_GetTick();

    if (buf[2] == CRSF_FRAMETYPE_RC && payload_plus_type_len >= 24U)
    {
        RC_Unpack16Channels(&buf[3], sbus_ch);
        RC_CommitFilteredChannels(sbus_ch);
        sbus_failsafe = 0U;
    }
}

static void RC_ProcessIncomingByte(uint8_t byte)
{
    uint32_t now_cycles = DWT->CYCCNT;
    uint32_t gap_cycles = now_cycles - rc_last_byte_cycles;
    rc_last_byte_cycles = now_cycles;

    rc_total_bytes++;
    last_raw_bytes[raw_ring_idx] = byte;
    raw_ring_idx = (uint8_t)((raw_ring_idx + 1U) % 4U);

    if (gap_cycles > (SystemCoreClock / 400U))
    {
        rc_idx = 0U;
        rc_frame_sync_ready = 1U;
    }

    if (rc_protocol_mode == RC_PROTO_CRSF)
    {
        if (rc_idx == 0U)
        {
            if (rc_frame_sync_ready != 0U && (byte == 0xC8U || byte == 0xEEU || byte == 0xEAU))
            {
                rc_buf[rc_idx++] = byte;
                rc_frame_sync_ready = 0U;
            }
        }
        else if (rc_idx == 1U)
        {
            if (byte >= 2U && byte <= 62U)
            {
                rc_buf[rc_idx++] = byte;
                crsf_expected_len = (uint8_t)(byte + 2U);
            }
            else
            {
                rc_idx = 0U;
            }
        }
        else
        {
            rc_buf[rc_idx++] = byte;
            if (rc_idx >= crsf_expected_len)
            {
                CRSF_ParseFrame(rc_buf, crsf_expected_len);
                rc_idx = 0U;
            }
        }
    }
    else
    {
        if (rc_idx == 0U)
        {
            if (rc_frame_sync_ready != 0U && byte == SBUS_HEADER)
            {
                rc_buf[rc_idx++] = byte;
                rc_frame_sync_ready = 0U;
            }
        }
        else
        {
            rc_buf[rc_idx++] = byte;
            if (rc_idx >= SBUS_FRAME_LEN)
            {
                SBUS_ParseFrame(rc_buf);
                rc_idx = 0U;
            }
        }
    }
}

static void RC_SetProtocol(uint8_t proto)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    rc_protocol_mode = proto;
    rc_idx = 0U;
    rc_frame_sync_ready = 1U;

    if (proto == RC_PROTO_SBUS_INV)
    {
        if (usart2_irq_available != 0U)
        {
            (void)HAL_UART_AbortReceive_IT(&huart2);
            HAL_NVIC_DisableIRQ(USART2_IRQn);
        }
        CLEAR_BIT(huart2.Instance->CR1, USART_CR1_RE | USART_CR1_RXNEIE);

        GPIO_InitStruct.Pin  = GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
        HAL_NVIC_SetPriority(EXTI3_IRQn, 0U, 0U);
        HAL_NVIC_EnableIRQ(EXTI3_IRQn);
    }
    else
    {
        HAL_NVIC_DisableIRQ(EXTI3_IRQn);
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);

        GPIO_InitStruct.Pin       = GPIO_PIN_3;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        huart2.Instance = USART2;
        if (proto == RC_PROTO_CRSF)
        {
            huart2.Init.BaudRate   = 420000U;
            huart2.Init.WordLength = UART_WORDLENGTH_8B;
            huart2.Init.StopBits   = UART_STOPBITS_1;
            huart2.Init.Parity     = UART_PARITY_NONE;
        }
        else
        {
            huart2.Init.BaudRate   = 100000U;
            huart2.Init.WordLength = UART_WORDLENGTH_9B;
            huart2.Init.StopBits   = UART_STOPBITS_2;
            huart2.Init.Parity     = UART_PARITY_EVEN;
        }
        huart2.Init.Mode         = UART_MODE_TX_RX;
        huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
        huart2.Init.OverSampling = UART_OVERSAMPLING_16;
        (void)HAL_UART_Init(&huart2);

        __HAL_UART_CLEAR_OREFLAG(&huart2);
        __HAL_UART_CLEAR_FEFLAG(&huart2);
        __HAL_UART_CLEAR_NEFLAG(&huart2);

        if (usart2_irq_available != 0U)
        {
            HAL_NVIC_SetPriority(USART2_IRQn, 0U, 0U);
            HAL_NVIC_EnableIRQ(USART2_IRQn);
            (void)HAL_UART_Receive_IT(&huart2, &sbus_rx_byte, 1U);
        }
    }
}

static void UART_PollFallbackIfNoIrq(void)
{
    if (rc_protocol_mode != RC_PROTO_SBUS_INV && usart2_irq_available == 0U)
    {
        uint32_t sr = huart2.Instance->SR;
        if ((sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) != 0U)
        {
            rc_uart_errors++;
            rc_idx = 0U;
            rc_frame_sync_ready = 0U;
            (void)huart2.Instance->DR;
        }
        else if ((sr & USART_SR_RXNE) != 0U)
        {
            uint8_t b = (uint8_t)(huart2.Instance->DR & 0xFFU);
            RC_ProcessIncomingByte(b);
        }
    }

    if (NVIC_GetEnableIRQ(USART1_IRQn) == 0U)
    {
        uint32_t sr1 = huart1.Instance->SR;
        if ((sr1 & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) != 0U)
        {
            (void)huart1.Instance->DR;
        }
        else if ((sr1 & USART_SR_RXNE) != 0U)
        {
            uint8_t b1 = (uint8_t)(huart1.Instance->DR & 0xFFU);
            MavRingBuf_Push(&mav_uart_rx, b1);
        }
    }
}

static void UART1_TransmitNonBlockingRC(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0U; i < len; i++)
    {
        uint32_t t0 = HAL_GetTick();
        while ((huart1.Instance->SR & USART_SR_TXE) == 0U)
        {
            UART_PollFallbackIfNoIrq();
            if ((HAL_GetTick() - t0) > 5U)
            {
                return;
            }
        }
        huart1.Instance->DR = data[i];
        UART_PollFallbackIfNoIrq();
    }
}

static void DelayAndPollUart(uint32_t wait_ms)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < wait_ms)
    {
        UART_PollFallbackIfNoIrq();
    }
}

static void MAVLink_SendDirect(const mavlink_message_t *msg)
{
    uint16_t len = (uint16_t)mavlink_msg_to_send_buffer(ctrl_usb_buf, msg);

    for (uint8_t retry = 0U; retry < 2U; retry++)
    {
        if (CDC_Transmit_FS(ctrl_usb_buf, len) == USBD_OK)
        {
            break;
        }
    }

    memcpy(ctrl_uart_buf, ctrl_usb_buf, len);
    UART1_TransmitNonBlockingRC(ctrl_uart_buf, len);
}

static void MAVLink_SendStatusText(uint8_t severity, const char *fmt_msg)
{
    char text_buf[50];
    memset(text_buf, 0, sizeof(text_buf));
    strncpy(text_buf, fmt_msg, sizeof(text_buf) - 1U);

    (void)mavlink_msg_statustext_pack(
        SYS_ID, COMP_ID_FC, &msg_statustext,
        severity, text_buf, 0U, 0U);

    MAVLink_SendDirect(&msg_statustext);
}

static void MAVLink_SendParamValue(uint16_t idx)
{
    if (idx >= PARAM_COUNT)
    {
        return;
    }

    (void)mavlink_msg_param_value_pack(
        SYS_ID, COMP_ID_FC, &msg_param,
        param_table[idx].name,
        *param_table[idx].val_ptr,
        MAV_PARAM_TYPE_REAL32,
        PARAM_COUNT,
        idx);

    MAVLink_SendDirect(&msg_param);
}

static void MAVLink_SendCommandAck(uint16_t command, uint8_t result)
{
    (void)mavlink_msg_command_ack_pack(
        SYS_ID, COMP_ID_FC, &msg_ack,
        command, result, 0U, 0, 0U, 0U);

    MAVLink_SendDirect(&msg_ack);
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
    if (msg->msgid == MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE)
    {
        mavlink_rc_channels_override_t ov;
        mavlink_msg_rc_channels_override_decode(msg, &ov);

        if ((ov.target_system == SYS_ID || ov.target_system == 0U) &&
            (ov.target_component == COMP_ID_FC || ov.target_component == MAV_COMP_ID_ALL))
        {
            const uint16_t raw_ch[16] = {
                ov.chan1_raw,  ov.chan2_raw,  ov.chan3_raw,  ov.chan4_raw,
                ov.chan5_raw,  ov.chan6_raw,  ov.chan7_raw,  ov.chan8_raw,
                ov.chan9_raw,  ov.chan10_raw, ov.chan11_raw, ov.chan12_raw,
                ov.chan13_raw, ov.chan14_raw, ov.chan15_raw, ov.chan16_raw
            };

            for (uint8_t i = 0U; i < 16U; i++)
            {
                if (raw_ch[i] >= 800U && raw_ch[i] <= 2200U)
                {
                    mav_rc_us[i] = clamp_u16(raw_ch[i], 1000U, 2000U);
                    mav_rc_active_mask |= (uint16_t)(1U << i);
                }
                else if (raw_ch[i] == 0U)
                {
                    mav_rc_active_mask &= (uint16_t)(~(1U << i));
                }
            }

            if (mav_rc_active_mask != 0U)
            {
                mav_rc_last_ms = HAL_GetTick();
            }
        }
    }
    else if (msg->msgid == MAVLINK_MSG_ID_MANUAL_CONTROL)
    {
        mavlink_manual_control_t mc;
        mavlink_msg_manual_control_decode(msg, &mc);

        if (mc.target == SYS_ID || mc.target == 0U)
        {
            if (mc.y != INT16_MAX)
            {
                mav_rc_us[0] = norm1000_to_us(mc.y);
                mav_rc_active_mask |= 0x0001U;
            }
            if (mc.x != INT16_MAX)
            {
                mav_rc_us[1] = norm1000_to_us(mc.x);
                mav_rc_active_mask |= 0x0002U;
            }
            if (mc.z != INT16_MAX)
            {
                uint16_t thr = (mc.z < 0) ? 1000U : (uint16_t)(1000 + mc.z);
                mav_rc_us[2] = clamp_u16(thr, 1000U, 2000U);
                mav_rc_active_mask |= 0x0004U;
            }
            if (mc.r != INT16_MAX)
            {
                mav_rc_us[3] = norm1000_to_us(mc.r);
                mav_rc_active_mask |= 0x0008U;
            }

            if (mav_rc_active_mask != 0U)
            {
                mav_rc_last_ms = HAL_GetTick();
            }
        }
    }
    else if (msg->msgid == MAVLINK_MSG_ID_COMMAND_LONG)
    {
        mavlink_command_long_t cmd;
        mavlink_msg_command_long_decode(msg, &cmd);

        if ((cmd.target_system == SYS_ID || cmd.target_system == 0U) &&
            (cmd.target_component == COMP_ID_FC || cmd.target_component == MAV_COMP_ID_ALL))
        {
            if (cmd.command == MAV_CMD_DO_SET_MODE)
            {
                uint32_t req_mode = (uint32_t)(cmd.param2 + 0.5f);
                if (req_mode == PLANE_MODE_MANUAL ||
                    req_mode == PLANE_MODE_STABILIZE ||
                    req_mode == PLANE_MODE_AUTO)
                {
                    flight_mode = (uint8_t)req_mode;
                    MAVLink_SendCommandAck(MAV_CMD_DO_SET_MODE, MAV_RESULT_ACCEPTED);
                }
                else
                {
                    MAVLink_SendCommandAck(MAV_CMD_DO_SET_MODE, MAV_RESULT_UNSUPPORTED);
                }
            }
        }
    }
    else if (msg->msgid == MAVLINK_MSG_ID_PARAM_REQUEST_LIST)
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
                MAVLink_SendParamValue((uint16_t)idx);
            }
        }
    }
}

static void MAVLink_ProcessIncoming(void)
{
    uint8_t byte = 0U;

    while (MavRingBuf_Pop(&mav_usb_rx, &byte) != 0U)
    {
        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &rx_msg_usb, &rx_status_usb) == MAVLINK_FRAMING_OK)
        {
            MAVLink_HandleMessage(&rx_msg_usb);
        }
    }

    while (MavRingBuf_Pop(&mav_uart_rx, &byte) != 0U)
    {
        if (mavlink_parse_char(MAVLINK_COMM_1, byte, &rx_msg_uart, &rx_status_uart) == MAVLINK_FRAMING_OK)
        {
            MAVLink_HandleMessage(&rx_msg_uart);
        }
    }

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

HAL_StatusTypeDef BMP280_Init(BMP280_t *dev)
{
    uint8_t calib[24];
    uint8_t config = 0x10U;
    uint8_t ctrl_meas = 0x57U;

    if (HAL_I2C_IsDeviceReady(&hi2c1, BMP280_ADDR_LOW, 2U, 30U) == HAL_OK)
    {
        dev->addr = BMP280_ADDR_LOW;
    }
    else if (HAL_I2C_IsDeviceReady(&hi2c1, BMP280_ADDR_HIGH, 2U, 30U) == HAL_OK)
    {
        dev->addr = BMP280_ADDR_HIGH;
    }
    else
    {
        return HAL_ERROR;
    }

    if (HAL_I2C_Mem_Read(&hi2c1, dev->addr, 0xD0U,
                         I2C_MEMADD_SIZE_8BIT, &dev->chip_id, 1U, 30U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if ((dev->chip_id != 0x58U) && (dev->chip_id != 0x60U))
    {
        return HAL_ERROR;
    }

    if (HAL_I2C_Mem_Read(&hi2c1, dev->addr, 0x88U,
                         I2C_MEMADD_SIZE_8BIT, calib, sizeof(calib), 50U) != HAL_OK)
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
                          I2C_MEMADD_SIZE_8BIT, &config, 1U, 30U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (HAL_I2C_Mem_Write(&hi2c1, dev->addr, 0xF4U,
                          I2C_MEMADD_SIZE_8BIT, &ctrl_meas, 1U, 30U) != HAL_OK)
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
        return NAN;
    }

    if (HAL_I2C_Mem_Read(&hi2c1, dev->addr, 0xF7U,
                         I2C_MEMADD_SIZE_8BIT, raw, sizeof(raw), 10U) != HAL_OK)
    {
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
        return NAN;
    }

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - p_var2) * 3125) / p_var1;
    p_var1 = ((int64_t)dev->dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    p_var2 = ((int64_t)dev->dig_P8 * p) >> 19;
    p = ((p + p_var1 + p_var2) >> 8) + (((int64_t)dev->dig_P7) << 4);

    dev->tekanan_pa = (float)p / 256.0f;

    if (!isfinite(dev->tekanan_pa) || dev->tekanan_pa < 30000.0f || dev->tekanan_pa > 120000.0f)
    {
        return NAN;
    }

    return dev->tekanan_pa;
}

void BMP280_UpdateAltitude(BMP280_t *dev)
{
    static float altitude_previous = 0.0f;
    static uint8_t altitude_initialized = 0U;

    if (dev->ready == 0U)
    {
        return;
    }

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
    HAL_NVIC_SetPriority(SysTick_IRQn, 2U, 0U);
    HAL_NVIC_SetPriority(OTG_FS_IRQn, 3U, 0U);
    HAL_NVIC_SetPriority(USART1_IRQn, 2U, 0U);

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    usart2_irq_available = (NVIC_GetEnableIRQ(USART2_IRQn) != 0U) ? 1U : 0U;

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm.aileron_L);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm.aileron_R);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm.elevator);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm.rudder);

    HAL_Delay(300U);

    /* -------- Inisialisasi MPU6050 + Kalibrasi Gyro Bias -------- */
    if (HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x75U,
                         I2C_MEMADD_SIZE_8BIT, &mpu.identitas, 1U, 50U) == HAL_OK &&
        ((mpu.identitas == 0x68U) || (mpu.identitas == 0x72U)))
    {
        uint8_t data_konfig = 0x00U;
        uint8_t dlpf_cfg    = 0x03U; /* Hardware DLPF 44 Hz untuk meredam getaran mekanis */
        if (HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x6BU, I2C_MEMADD_SIZE_8BIT, &data_konfig, 1U, 50U) == HAL_OK &&
            HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x1CU, I2C_MEMADD_SIZE_8BIT, &data_konfig, 1U, 50U) == HAL_OK &&
            HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x1BU, I2C_MEMADD_SIZE_8BIT, &data_konfig, 1U, 50U) == HAL_OK &&
            HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x1AU, I2C_MEMADD_SIZE_8BIT, &dlpf_cfg, 1U, 50U) == HAL_OK)
        {
            mpu.ready = 1U;
            float sum_gx = 0.0f, sum_gy = 0.0f, sum_gz = 0.0f;
            uint16_t gyro_samples = 0U;

            for (uint16_t i = 0U; i < 150U; i++)
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

                    if (gyro_samples == 1U)
                    {
                        mpu.kalmanX    = atan2f((float)ay, (float)az) * RAD_TO_DEG;
                        mpu.kalmanY    = atan2f((float)(-ax), (float)az) * RAD_TO_DEG;
                        mpu.roll_filt  = mpu.kalmanX;
                        mpu.pitch_filt = mpu.kalmanY;
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
        }
    }

    /* -------- Inisialisasi BMP280 -------- */
    if (BMP280_Init(&bmp) == HAL_OK)
    {
        float sum_p0 = 0.0f;
        uint8_t valid_samples = 0U;
        for (uint8_t i = 0U; i < 10U; i++)
        {
            float p_sample = BMP280_ReadPressure(&bmp);
            if (isfinite(p_sample))
            {
                sum_p0 += p_sample;
                valid_samples++;
            }
            HAL_Delay(15U);
        }
        if (valid_samples > 0U)
        {
            bmp.tekanan_ref_p0 = sum_p0 / (float)valid_samples;
            bmp.ready = 1U;
        }
    }

    bmp.waktu_baro = HAL_GetTick();
    waktu_sebelumnya = bmp.waktu_baro;
    waktu_hb = bmp.waktu_baro;
    waktu_led = bmp.waktu_baro;
    waktu_telem = bmp.waktu_baro;
    waktu_rc_diag = bmp.waktu_baro;
    rc_last_switch_ms = bmp.waktu_baro;

    RC_SetProtocol(RC_PROTO_SBUS_INV);

    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_CLEAR_FEFLAG(&huart1);
    __HAL_UART_CLEAR_NEFLAG(&huart1);
    if (NVIC_GetEnableIRQ(USART1_IRQn) != 0U)
    {
        (void)HAL_UART_Receive_IT(&huart1, &radio_rx_byte, 1U);
    }
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

        UART_PollFallbackIfNoIrq();
        MAVLink_ProcessIncoming();
        BMP280_UpdateAltitude(&bmp);

        /* 1. Baca & Filter MPU6050 */
        if (mpu.ready != 0U &&
            HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x3BU,
                             I2C_MEMADD_SIZE_8BIT, mpu.data_mentah, 14U, 5U) == HAL_OK)
        {
            mpu.accX_raw  = (int16_t)((mpu.data_mentah[0] << 8) | mpu.data_mentah[1]);
            mpu.accY_raw  = (int16_t)((mpu.data_mentah[2] << 8) | mpu.data_mentah[3]);
            mpu.accZ_raw  = (int16_t)((mpu.data_mentah[4] << 8) | mpu.data_mentah[5]);
            mpu.gyroX_raw = (int16_t)((mpu.data_mentah[8] << 8) | mpu.data_mentah[9]);
            mpu.gyroY_raw = (int16_t)((mpu.data_mentah[10] << 8) | mpu.data_mentah[11]);
            mpu.gyroZ_raw = (int16_t)((mpu.data_mentah[12] << 8) | mpu.data_mentah[13]);

            mpu.accX_g = mpu.accX_raw / 16384.0f;
            mpu.accY_g = mpu.accY_raw / 16384.0f;
            mpu.accZ_g = mpu.accZ_raw / 16384.0f;
            mpu.gyroX_dps = (mpu.gyroX_raw / 131.0f) - mpu.gyroX_bias;
            mpu.gyroY_dps = (mpu.gyroY_raw / 131.0f) - mpu.gyroY_bias;
            mpu.gyroZ_dps = (mpu.gyroZ_raw / 131.0f) - mpu.gyroZ_bias;

            mpu.gyroX_filt += 0.20f * (mpu.gyroX_dps - mpu.gyroX_filt);
            mpu.gyroY_filt += 0.20f * (mpu.gyroY_dps - mpu.gyroY_filt);
            mpu.gyroZ_filt += 0.20f * (mpu.gyroZ_dps - mpu.gyroZ_filt);

            mpu.sudut_accX = atan2f(mpu.accY_g, mpu.accZ_g) * RAD_TO_DEG;
            mpu.sudut_accY = atan2f(-mpu.accX_g, mpu.accZ_g) * RAD_TO_DEG;

            mpu.kalmanX += mpu.gyroX_dps * mpu.dt;
            mpu.err_gyroX += mpu.Q_drift;
            mpu.kalman_gainX = mpu.err_gyroX / (mpu.err_gyroX + mpu.E_acc);
            mpu.kalmanX += mpu.kalman_gainX * (mpu.sudut_accX - mpu.kalmanX);
            mpu.err_gyroX = (1.0f - mpu.kalman_gainX) * mpu.err_gyroX;

            mpu.kalmanY += mpu.gyroY_dps * mpu.dt;
            mpu.err_gyroY += mpu.Q_drift;
            mpu.kalman_gainY = mpu.err_gyroY / (mpu.err_gyroY + mpu.E_acc);
            mpu.kalmanY += mpu.kalman_gainY * (mpu.sudut_accY - mpu.kalmanY);
            mpu.err_gyroY = (1.0f - mpu.kalman_gainY) * mpu.err_gyroY;

            mpu.kalmanZ += mpu.gyroZ_dps * mpu.dt;

            mpu.roll_filt  += 0.25f * (mpu.kalmanX - mpu.roll_filt);
            mpu.pitch_filt += 0.25f * (mpu.kalmanY - mpu.pitch_filt);
        }

        /* 2. Last-Value Hold 300 ms + Pencarian Protokol Remote */
        uint8_t rc_hold_active = (rc_last_frame_ms != 0U &&
                                  (waktu_sekarang - rc_last_frame_ms) <= RC_HOLD_TIMEOUT_MS) ? 1U : 0U;

        if (rc_hold_active == 0U && rc_proto_locked == 0U)
        {
            if ((waktu_sekarang - rc_last_switch_ms) >= RC_PROTO_SWITCH_MS)
            {
                rc_last_switch_ms = waktu_sekarang;
                uint8_t next_proto = (uint8_t)((rc_protocol_mode + 1U) % RC_PROTO_COUNT);
                RC_SetProtocol(next_proto);
            }
        }
        else
        {
            rc_last_switch_ms = waktu_sekarang;
        }

        static const uint16_t default_rc_us[16] = {
            1500U, 1500U, 1000U, 1500U, 1000U, 1500U, 1500U, 1500U,
            1500U, 1500U, 1500U, 1500U, 1500U, 1500U, 1500U, 1500U
        };

        uint8_t mav_rc_ok = (mav_rc_active_mask != 0U &&
                             (waktu_sekarang - mav_rc_last_ms) <= MAV_RC_TIMEOUT_MS) ? 1U : 0U;

        if (mav_rc_ok == 0U)
        {
            mav_rc_active_mask = 0U;
        }

        for (uint8_t i = 0U; i < 16U; i++)
        {
            uint16_t ch_val = sbus_us[i];
            active_rc_us[i] = (rc_hold_active != 0U) ? ch_val : default_rc_us[i];

            if (mav_rc_ok != 0U && ((mav_rc_active_mask & (uint16_t)(1U << i)) != 0U))
            {
                active_rc_us[i] = mav_rc_us[i];
            }
        }

        /* Haluskan input stick & terapkan deadband tengah 18 us untuk meredam jitter potensiometer */
        float raw_roll_cmd  = shape_stick_us(active_rc_us[0], STICK_DEADBAND_US);
        float raw_pitch_cmd = shape_stick_us(active_rc_us[1], STICK_DEADBAND_US);
        float raw_yaw_cmd   = shape_stick_us(active_rc_us[3], STICK_DEADBAND_US);

        filt_stick_roll_us  += 0.45f * (raw_roll_cmd  - filt_stick_roll_us);
        filt_stick_pitch_us += 0.45f * (raw_pitch_cmd - filt_stick_pitch_us);
        filt_stick_yaw_us   += 0.45f * (raw_yaw_cmd   - filt_stick_yaw_us);

        if (fabsf(filt_stick_roll_us)  < 2.0f) filt_stick_roll_us  = 0.0f;
        if (fabsf(filt_stick_pitch_us) < 2.0f) filt_stick_pitch_us = 0.0f;
        if (fabsf(filt_stick_yaw_us)   < 2.0f) filt_stick_yaw_us   = 0.0f;

        /* 3. Pemilihan Mode dengan Hysteresis +-25 us pada CH5 */
        uint16_t ch5 = active_rc_us[4];
        int32_t ch5_delta = (int32_t)ch5 - (int32_t)ch5_last_us;
        if (ch5_delta < 0) ch5_delta = -ch5_delta;

        if (ch5_initialized == 0U || ch5_delta > 35)
        {
            ch5_initialized = 1U;
            ch5_last_us = ch5;

            if (flight_mode == PLANE_MODE_MANUAL)
            {
                if (ch5 > 1725U)
                {
                    flight_mode = PLANE_MODE_AUTO;
                }
                else if (ch5 > 1325U)
                {
                    flight_mode = PLANE_MODE_STABILIZE;
                }
            }
            else if (flight_mode == PLANE_MODE_STABILIZE)
            {
                if (ch5 < 1275U)
                {
                    flight_mode = PLANE_MODE_MANUAL;
                }
                else if (ch5 > 1725U)
                {
                    flight_mode = PLANE_MODE_AUTO;
                }
            }
            else /* PLANE_MODE_AUTO */
            {
                if (ch5 < 1275U)
                {
                    flight_mode = PLANE_MODE_MANUAL;
                }
                else if (ch5 < 1675U)
                {
                    flight_mode = PLANE_MODE_STABILIZE;
                }
            }
        }

        if (mpu.ready == 0U)
        {
            flight_mode = PLANE_MODE_MANUAL;
        }

        if (flight_mode != prev_flight_mode)
        {
            AutoPID_Reset(bmp.ketinggian_filter, filt_stick_roll_us, filt_stick_pitch_us, filt_stick_yaw_us);
            prev_flight_mode = flight_mode;
        }

        /* 4. Perhitungan Kendali Servo (MANUAL vs STABILIZE vs AUTO) */
        float out_roll_us  = 0.0f;
        float out_pitch_us = 0.0f;
        float out_yaw_us   = 0.0f;

        if (flight_mode == PLANE_MODE_MANUAL)
        {
            out_roll_us  = filt_stick_roll_us;
            out_pitch_us = filt_stick_pitch_us;
            out_yaw_us   = filt_stick_yaw_us;
        }
        else
        {
            float stick_roll  = filt_stick_roll_us  / 500.0f;
            float stick_pitch = filt_stick_pitch_us / 500.0f;
            float stick_yaw   = filt_stick_yaw_us   / 500.0f;

            float target_roll_deg  = stick_roll  * 45.0f;
            float target_pitch_deg = stick_pitch * 30.0f;

            if (flight_mode == PLANE_MODE_AUTO && bmp.ready != 0U)
            {
                if (fabsf(stick_pitch) < 0.05f)
                {
                    float alt_err = apply_deadband_f(pid.target_alt_m - bmp.ketinggian_filter, 0.30f);
                    if (active_rc_us[2] > 1150U && fabsf(alt_err) > 0.0f)
                    {
                        pid.i_alt = clamp_f(pid.i_alt + (pid.ki_alt * alt_err * mpu.dt), -8.0f, 8.0f);
                    }
                    else if (active_rc_us[2] <= 1150U)
                    {
                        pid.i_alt = 0.0f;
                    }
                    float alt_pitch_cmd = clamp_f((pid.kp_alt * alt_err) + pid.i_alt, -15.0f, 15.0f);
                    target_pitch_deg += alt_pitch_cmd;
                }
                else
                {
                    pid.target_alt_m = bmp.ketinggian_filter;
                    pid.i_alt = 0.0f;
                }
            }

            /* Deadband sudut 2.5 derajat agar servo diam total saat pesawat diam di meja */
            float err_roll  = apply_deadband_f(target_roll_deg  - mpu.roll_filt,  2.5f);
            float err_pitch = apply_deadband_f(target_pitch_deg - mpu.pitch_filt, 2.5f);

            if (active_rc_us[2] > 1150U)
            {
                if (fabsf(err_roll) > 0.0f)
                {
                    pid.i_roll = clamp_f(pid.i_roll + (pid.ki_roll * err_roll * mpu.dt), -120.0f, 120.0f);
                }
                if (fabsf(err_pitch) > 0.0f)
                {
                    pid.i_pitch = clamp_f(pid.i_pitch + (pid.ki_pitch * err_pitch * mpu.dt), -120.0f, 120.0f);
                }
            }
            else
            {
                pid.i_roll  = 0.0f;
                pid.i_pitch = 0.0f;
            }

            float raw_pid_roll  = (pid.kp_roll  * err_roll)  + pid.i_roll  - (pid.kd_roll  * mpu.gyroX_filt);
            float raw_pid_pitch = (pid.kp_pitch * err_pitch) + pid.i_pitch - (pid.kd_pitch * mpu.gyroY_filt);
            float raw_pid_yaw   = (stick_yaw * 500.0f) - (pid.kp_yaw * mpu.gyroZ_filt);

            raw_pid_roll  = clamp_f(raw_pid_roll,  -500.0f, 500.0f);
            raw_pid_pitch = clamp_f(raw_pid_pitch, -500.0f, 500.0f);
            raw_pid_yaw   = clamp_f(raw_pid_yaw,   -500.0f, 500.0f);

            pid.smooth_roll_us  += 0.25f * (raw_pid_roll  - pid.smooth_roll_us);
            pid.smooth_pitch_us += 0.25f * (raw_pid_pitch - pid.smooth_pitch_us);
            pid.smooth_yaw_us   += 0.25f * (raw_pid_yaw   - pid.smooth_yaw_us);

            out_roll_us  = pid.smooth_roll_us;
            out_pitch_us = pid.smooth_pitch_us;
            out_yaw_us   = pid.smooth_yaw_us;
        }

        uint16_t target_ail_L = (uint16_t)(clamp_f(1500.0f + pwm.trim_roll_L + (pwm.dir_roll_L * out_roll_us),  1000.0f, 2000.0f) + 0.5f);
        uint16_t target_ail_R = (uint16_t)(clamp_f(1500.0f + pwm.trim_roll_R + (pwm.dir_roll_R * out_roll_us),  1000.0f, 2000.0f) + 0.5f);
        uint16_t target_ele   = (uint16_t)(clamp_f(1500.0f + pwm.trim_pitch  + (pwm.dir_pitch  * out_pitch_us), 1000.0f, 2000.0f) + 0.5f);
        uint16_t target_rud   = (uint16_t)(clamp_f(1500.0f + pwm.trim_yaw    + (pwm.dir_yaw    * out_yaw_us),   1000.0f, 2000.0f) + 0.5f);

        pwm.aileron_L = apply_deadband_u16(pwm.aileron_L, target_ail_L, SERVO_DEADBAND_US);
        pwm.aileron_R = apply_deadband_u16(pwm.aileron_R, target_ail_R, SERVO_DEADBAND_US);
        pwm.elevator  = apply_deadband_u16(pwm.elevator,  target_ele,   SERVO_DEADBAND_US);
        pwm.rudder    = apply_deadband_u16(pwm.rudder,    target_rud,   SERVO_DEADBAND_US);

        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm.aileron_L);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm.aileron_R);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm.elevator);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm.rudder);

        /* 5. Kirim Telemetri MAVLink v2 ke Mission Planner (20 Hz) */
        if ((waktu_sekarang - waktu_telem) >= TELEM_UPDATE_MS)
        {
            waktu_telem = waktu_sekarang;
            uint16_t total_len = 0U;

            if ((waktu_sekarang - waktu_hb) >= 1000U)
            {
                waktu_hb = waktu_sekarang;
                mavlink_msg_heartbeat_pack(
                    SYS_ID, COMP_ID_FC, &msg_hb,
                    MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_ARDUPILOTMEGA,
                    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED,
                    flight_mode, MAV_STATE_ACTIVE);
                total_len += (uint16_t)mavlink_msg_to_send_buffer(&usb_tx_buf[total_len], &msg_hb);
            }

            mavlink_msg_attitude_pack(
                SYS_ID, COMP_ID_FC, &msg_att, waktu_sekarang,
                mpu.roll_filt * DEG_TO_RAD, mpu.pitch_filt * DEG_TO_RAD, mpu.kalmanZ * DEG_TO_RAD,
                mpu.gyroX_filt * DEG_TO_RAD, mpu.gyroY_filt * DEG_TO_RAD, mpu.gyroZ_filt * DEG_TO_RAD);
            total_len += (uint16_t)mavlink_msg_to_send_buffer(&usb_tx_buf[total_len], &msg_att);

            float heading_deg_f = fmodf(mpu.kalmanZ, 360.0f);
            if (heading_deg_f < 0.0f) heading_deg_f += 360.0f;
            int16_t  heading_deg  = (int16_t)heading_deg_f;
            uint16_t heading_cdeg = (uint16_t)(heading_deg_f * 100.0f);
            int32_t  alt_mm       = (int32_t)(bmp.ketinggian_filter * 1000.0f);
            int16_t  vz_cms       = (int16_t)(-bmp.climb_mps * 100.0f);

            mavlink_msg_global_position_int_pack(
                SYS_ID, COMP_ID_FC, &msg_gpos, waktu_sekarang,
                0, 0, alt_mm, alt_mm, 0, 0, vz_cms, heading_cdeg);
            total_len += (uint16_t)mavlink_msg_to_send_buffer(&usb_tx_buf[total_len], &msg_gpos);

            uint16_t thr_pct = (active_rc_us[2] <= 1000U) ? 0U : (uint16_t)((active_rc_us[2] - 1000U) / 10U);
            if (thr_pct > 100U) thr_pct = 100U;

            mavlink_msg_vfr_hud_pack(
                SYS_ID, COMP_ID_FC, &msg_vfr,
                0.0f, 0.0f, heading_deg, thr_pct,
                bmp.ketinggian_filter, bmp.climb_mps);
            total_len += (uint16_t)mavlink_msg_to_send_buffer(&usb_tx_buf[total_len], &msg_vfr);

            mavlink_msg_servo_output_raw_pack(
                SYS_ID, COMP_ID_FC, &msg_servo,
                waktu_sekarang * 1000U, 0U,
                pwm.aileron_L, pwm.aileron_R, pwm.elevator, pwm.rudder,
                0U, 0U, 0U, 0U,
                0U, 0U, 0U, 0U,
                0U, 0U, 0U, 0U);
            total_len += (uint16_t)mavlink_msg_to_send_buffer(&usb_tx_buf[total_len], &msg_servo);

            uint8_t rc_rssi = ((rc_hold_active != 0U) || (mav_rc_ok != 0U)) ? 255U : 0U;
            mavlink_msg_rc_channels_pack(
                SYS_ID, COMP_ID_FC, &msg_rc,
                waktu_sekarang, 16U,
                active_rc_us[0],  active_rc_us[1],  active_rc_us[2],  active_rc_us[3],
                active_rc_us[4],  active_rc_us[5],  active_rc_us[6],  active_rc_us[7],
                active_rc_us[8],  active_rc_us[9],  active_rc_us[10], active_rc_us[11],
                active_rc_us[12], active_rc_us[13], active_rc_us[14], active_rc_us[15],
                UINT16_MAX, UINT16_MAX, rc_rssi);
            total_len += (uint16_t)mavlink_msg_to_send_buffer(&usb_tx_buf[total_len], &msg_rc);

            if (total_len > 0U)
            {
                (void)CDC_Transmit_FS(usb_tx_buf, total_len);
                memcpy(uart_tx_buf, usb_tx_buf, total_len);
                UART1_TransmitNonBlockingRC(uart_tx_buf, total_len);
            }
        }

        /* 6. Laporan Status Mode & Remote ke tab Messages Mission Planner */
        if ((waktu_sekarang - waktu_rc_diag) >= RC_DIAG_MS)
        {
            waktu_rc_diag = waktu_sekarang;
            char diag_str[50];
            const char *mode_name = (flight_mode == PLANE_MODE_MANUAL) ? "MANUAL" :
                                    ((flight_mode == PLANE_MODE_STABILIZE) ? "STABILIZE" : "AUTO");

            if (rc_hold_active != 0U)
            {
                (void)snprintf(diag_str, sizeof(diag_str),
                    "[%s] CH1:%u CH5:%u GLT:%lu",
                    mode_name, active_rc_us[0], active_rc_us[4],
                    (unsigned long)rc_glitches_blocked);
                MAVLink_SendStatusText(MAV_SEVERITY_INFO, diag_str);
            }
            else if (rc_total_bytes > 0U || rc_uart_errors > 0U)
            {
                (void)snprintf(diag_str, sizeof(diag_str),
                    "[%s] RC SYNC B:%lu E:%lu %02X %02X",
                    mode_name, (unsigned long)rc_total_bytes, (unsigned long)rc_uart_errors,
                    last_raw_bytes[0], last_raw_bytes[1]);
                MAVLink_SendStatusText(MAV_SEVERITY_WARNING, diag_str);
            }
            else
            {
                (void)snprintf(diag_str, sizeof(diag_str),
                    "[%s] RC PA3 KOSONG (Cek receiver -> PA3)", mode_name);
                MAVLink_SendStatusText(MAV_SEVERITY_WARNING, diag_str);
            }
        }

        uint32_t led_interval = (flight_mode == PLANE_MODE_MANUAL) ? 300U : 100U;
        if ((waktu_sekarang - waktu_led) >= led_interval)
        {
            waktu_led = waktu_sekarang;
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        }

        DelayAndPollUart(10U);
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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

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
  sConfigOC.Pulse = 1500;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
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
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
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
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
void EXTI3_IRQHandler(void)
{
    /* Kompensasi ~20 siklus CPU (1.25 us) latency masuk ke ISR pada clock 16 MHz */
    uint32_t t_start = DWT->CYCCNT - 20U;
    uint32_t bit_cycles = SystemCoreClock / 100000U;

    /* Cek tengah Start Bit (5 us) */
    while ((DWT->CYCCNT - t_start) < (bit_cycles / 2U)) { }
    if ((GPIOA->IDR & GPIO_PIN_3) == 0U)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
        return;
    }

    /* Sampling 8 data bit LSB-first (15 us, 25 us, ..., 85 us) */
    uint8_t byte_val = 0U;
    uint8_t ones_count = 0U;
    for (uint8_t i = 0U; i < 8U; i++)
    {
        uint32_t sample_tick = ((uint32_t)i * bit_cycles) + ((bit_cycles * 3U) / 2U);
        while ((DWT->CYCCNT - t_start) < sample_tick) { }
        if ((GPIOA->IDR & GPIO_PIN_3) == 0U)
        {
            byte_val |= (uint8_t)(1U << i);
            ones_count++;
        }
    }

    /* Sampling Even Parity Bit (95 us) */
    uint32_t parity_tick = (bit_cycles * 19U) / 2U;
    while ((DWT->CYCCNT - t_start) < parity_tick) { }
    uint8_t parity_bit = ((GPIOA->IDR & GPIO_PIN_3) == 0U) ? 1U : 0U;

    /* Sampling Stop Bit 1 (105 us) */
    uint32_t stop1_tick = (bit_cycles * 21U) / 2U;
    while ((DWT->CYCCNT - t_start) < stop1_tick) { }
    uint8_t stop1_bad = ((GPIOA->IDR & GPIO_PIN_3) != 0U) ? 1U : 0U;

    if ((((ones_count + parity_bit) & 1U) == 0U) && (stop1_bad == 0U))
    {
        RC_ProcessIncomingByte(byte_val);
    }
    else
    {
        rc_uart_errors++;
        rc_idx = 0U;
        rc_frame_sync_ready = 0U;
    }

    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
}

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
        if (rc_protocol_mode != RC_PROTO_SBUS_INV)
        {
            RC_ProcessIncomingByte(sbus_rx_byte);
            (void)HAL_UART_Receive_IT(&huart2, &sbus_rx_byte, 1U);
        }
    }
    else if (huart->Instance == USART1)
    {
        MavRingBuf_Push(&mav_uart_rx, radio_rx_byte);
        (void)HAL_UART_Receive_IT(&huart1, &radio_rx_byte, 1U);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        rc_uart_errors++;
        __HAL_UART_CLEAR_OREFLAG(&huart2);
        __HAL_UART_CLEAR_FEFLAG(&huart2);
        __HAL_UART_CLEAR_NEFLAG(&huart2);
        rc_idx = 0U;
        rc_frame_sync_ready = 0U;
        if (rc_protocol_mode != RC_PROTO_SBUS_INV)
        {
            (void)HAL_UART_Receive_IT(&huart2, &sbus_rx_byte, 1U);
        }
    }
    else if (huart->Instance == USART1)
    {
        __HAL_UART_CLEAR_OREFLAG(&huart1);
        __HAL_UART_CLEAR_FEFLAG(&huart1);
        __HAL_UART_CLEAR_NEFLAG(&huart1);
        (void)HAL_UART_Receive_IT(&huart1, &radio_rx_byte, 1U);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
