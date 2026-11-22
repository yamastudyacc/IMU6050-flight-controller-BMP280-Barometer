/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Final Fixed-Wing Flight Controller (100 Hz Deterministic)
  *                   - Default PID Stabilize + Auto (AltHold) + Manual Backup
  *                   - Signed RC Frame Age + Seqlock Atomic Snapshot (No IRQ Lock)
  *                   - Low-Lag PID + Backlash Servo Hysteresis (Smooth Sweep)
  *                   - Whole-Packet Non-Blocking MAVLink Telemetry (40 ms step)
  *                   - MPU6050 Fault/Freeze Detection + 9-Pulse SCL I2C1 Recovery
  *                   - Independent Watchdog (IWDG) Fail-Safe Boot
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
    float z1;
    float z2;
    uint8_t initialized;
} BiquadLPF_t;

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
    /* Gain PID Attitude & Altitude (dapat diatur live di Mission Planner) */
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

#define UART_TX_BUF_SIZE 512U
typedef struct {
    uint8_t buf[UART_TX_BUF_SIZE];
    uint16_t head;
    uint16_t tail;
} UartTxRingBuf_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU_ADDR              (0x68U << 1)
#define BMP280_ADDR_LOW       (0x76U << 1)
#define BMP280_ADDR_HIGH      (0x77U << 1)

/* Pin I2C1 untuk pemulihan bus 9-pulsa SCL jika SDA tertahan LOW */
#define I2C1_SCL_PORT         GPIOB
#define I2C1_SCL_PIN          GPIO_PIN_6
#define I2C1_SDA_PORT         GPIOB
#define I2C1_SDA_PIN          GPIO_PIN_7

#define RAD_TO_DEG            57.295779513f
#define DEG_TO_RAD            0.01745329252f

#define SYS_ID                1U
#define COMP_ID_FC            1U

#define CTRL_LOOP_MS          10U          /* Loop PID tetap 100 Hz (10.0 ms) */
#define BARO_UPDATE_MS        50U          /* Update barometer 20 Hz */
#define TELEM_STEP_MS         40U          /* 1 paket MAVLink tiap 40 ms (~1,3 KB/s) agar link 57600 tidak jenuh */
#define PARAM_STREAM_MS       20U          /* Jeda kirim parameter ke Mission Planner */
#define RC_DIAG_MS            2000U        /* Kirim status PID & sensor setiap 2 detik */

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

#define RC_HOLD_TIMEOUT_MS    300U         /* Tahan posisi terakhir 300 ms saat paket drop */
#define STICK_DEADBAND_US     20.0f        /* Redam jitter potensiometer stick di titik tengah */
#define PID_OUT_SMOOTH        0.50f        /* EMA output PID (lag ~14 ms, responsif & halus) */
#define SERVO_SLEW_MAX_US     50.0f        /* STABILIZE/AUTO: maks 50 us per 10 ms (5000 us/s) */
#define SERVO_SLEW_MANUAL_US  1000.0f      /* MANUAL: tanpa batas slew (rentang penuh = 1000 us) */
#define SERVO_HYST_US         5            /* STABILIZE/AUTO: jendela backlash +-5 us */
#define SERVO_HYST_MANUAL_US  2            /* MANUAL: hysteresis kecil agar tetap tidak mendengung */
#define MAV_RC_TIMEOUT_MS     800U

#define MPU_FAIL_LIMIT        10U          /* 10 siklus (100 ms) gagal/beku -> fault */
#define MPU_OK_RECOVER        50U          /* 50 siklus (0,5 s) sehat berturut-turut -> fault dilepas */
#define MPU_RECOVER_RETRY_MS  1000U        /* Jeda antar percobaan pulihkan I2C/MPU */

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
IWDG_HandleTypeDef hiwdg;   /* di blok USER CODE agar tidak hilang saat regenerate CubeMX */

MPU6050_t mpu = {
    .dt        = 0.01f,
    .err_gyroX = 1.0f,
    .err_gyroY = 1.0f,
    .Q_drift   = 0.001f,
    .E_acc     = 12.0f
};

BiquadLPF_t gyro_lpf_x = {0};
BiquadLPF_t gyro_lpf_y = {0};
BiquadLPF_t gyro_lpf_z = {0};

uint8_t  mpu_fault           = 0U;
uint8_t  mpu_fail_count      = 0U;
uint8_t  mpu_ok_count        = 0U;
uint8_t  mpu_frozen_count    = 0U;
uint8_t  mpu_prev_raw[14]    = {0};
uint32_t mpu_last_recover_ms = 0U;

BMP280_t bmp = {
    .tekanan_pa     = 101325.0f,
    .tekanan_ref_p0 = 101325.0f
};

AutoPID_t pid = {
    .kp_roll      = 6.0f,
    .ki_roll      = 0.65f,
    .kd_roll      = 0.30f,
    .kp_pitch     = 6.5f,
    .ki_pitch     = 0.65f,
    .kd_pitch     = 0.30f,
    .kp_yaw       = 0.40f,
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

/* Default langsung ke STABILIZE agar PID langsung aktif saat menyala */
uint8_t  flight_mode        = PLANE_MODE_STABILIZE;
uint8_t  prev_flight_mode   = PLANE_MODE_STABILIZE;
uint16_t ch5_last_us        = 1000U;
uint8_t  ch5_initialized    = 0U;

float    filt_stick_roll_us  = 0.0f;
float    filt_stick_pitch_us = 0.0f;
float    filt_stick_yaw_us   = 0.0f;

float    servo_slew_ail_L    = 1500.0f;
float    servo_slew_ail_R    = 1500.0f;
float    servo_slew_ele      = 1500.0f;
float    servo_slew_rud      = 1500.0f;

uint8_t  param_streaming    = 0U;
uint16_t param_stream_idx   = 0U;
uint32_t waktu_param_stream = 0U;

/* ------------------------------- MAVLink -------------------------------- */
uint32_t waktu_ctrl    = 0U;
uint32_t waktu_hb      = 0U;
uint32_t waktu_led     = 0U;
uint32_t waktu_telem   = 0U;
uint32_t waktu_rc_diag = 0U;
uint8_t  telem_slot    = 0U;

mavlink_message_t msg_tx;
uint8_t pkt_buf[MAVLINK_MAX_PACKET_LEN];

MavRingBuf_t    mav_usb_rx  = { .head = 0U, .tail = 0U };
MavRingBuf_t    mav_uart_rx = { .head = 0U, .tail = 0U };
UartTxRingBuf_t uart1_tx_rb = { .head = 0U, .tail = 0U };
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
uint16_t sbus_hist[16][5];
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

/* Filter Biquad Butterworth Orde-2 (fc = 15 Hz pada Fs = 100 Hz) */
static float biquad15Hz_Update(BiquadLPF_t *f, float input)
{
    const float b0 =  0.1311064f;
    const float b1 =  0.2622129f;
    const float b2 =  0.1311064f;
    const float a1 = -0.7477892f;
    const float a2 =  0.2722149f;

    if (f->initialized == 0U)
    {
        f->z1 = input * (1.0f - b0);
        f->z2 = input * (b2 - a2);
        f->initialized = 1U;
        return input;
    }

    float out = (b0 * input) + f->z1;
    f->z1 = (b1 * input) - (a1 * out) + f->z2;
    f->z2 = (b2 * input) - (a2 * out);
    return out;
}

/* Kurva Deadband Kubik Halus (C1-smooth): mencegah osilasi bolak-balik di tepi deadband */
static float smooth_deadband_f(float val, float db, float blend)
{
    float abs_v = fabsf(val);
    if (abs_v <= db)
    {
        return 0.0f;
    }

    float excess = abs_v - db;
    float shaped = excess;
    if (blend > 0.001f && excess < blend)
    {
        float u = excess / blend;
        shaped = excess * u * (2.0f - u);
    }

    return (val >= 0.0f) ? shaped : -shaped;
}

static float slew_limit_f(float current, float target, float max_step)
{
    float diff = target - current;
    if (diff > max_step)  return current + max_step;
    if (diff < -max_step) return current - max_step;
    return target;
}

static float shape_stick_us(uint16_t raw_us, float deadband_us)
{
    float delta = (float)raw_us - 1500.0f;
    float shaped = smooth_deadband_f(delta, deadband_us, 35.0f);
    return clamp_f((shaped * 500.0f) / (500.0f - deadband_us), -500.0f, 500.0f);
}

/* Hysteresis tipe backlash: diam selama target masih dalam +-hyst dari output, tetapi begitu
   target keluar dari jendela itu, output MENGIKUTI (tertinggal sebesar hyst), bukan melompat. */
static uint16_t apply_hyst_u16(uint16_t prev_val, uint16_t new_val, int32_t hyst)
{
    int32_t p = (int32_t)prev_val;
    int32_t n = (int32_t)new_val;

    if (n <= 1000 || n >= 2000) return new_val;   /* ujung langkah: capai batas penuh */
    if (n > (p + hyst)) return (uint16_t)(n - hyst);
    if (n < (p - hyst)) return (uint16_t)(n + hyst);
    return prev_val;
}

static uint16_t median5_u16(uint16_t a, uint16_t b, uint16_t c, uint16_t d, uint16_t e)
{
    uint16_t v[5] = { a, b, c, d, e };
    for (uint8_t i = 1U; i < 5U; i++)
    {
        uint16_t key = v[i];
        int8_t j = (int8_t)i - 1;
        while (j >= 0 && v[j] > key)
        {
            v[j + 1] = v[j];
            j--;
        }
        v[j + 1] = key;
    }
    return v[2];
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

static void Uart1Tx_Enqueue(const uint8_t *data, uint16_t len)
{
    uint16_t used       = (uint16_t)((uart1_tx_rb.head + UART_TX_BUF_SIZE - uart1_tx_rb.tail) % UART_TX_BUF_SIZE);
    uint16_t free_bytes = (uint16_t)(UART_TX_BUF_SIZE - 1U - used);

    if (len > free_bytes)
    {
        return; /* Buang SATU paket utuh; jangan sisakan paket terpotong di stream */
    }

    for (uint16_t i = 0U; i < len; i++)
    {
        uart1_tx_rb.buf[uart1_tx_rb.head] = data[i];
        uart1_tx_rb.head = (uint16_t)((uart1_tx_rb.head + 1U) % UART_TX_BUF_SIZE);
    }
}

static void Uart1Tx_ServiceNonBlocking(void)
{
    while (uart1_tx_rb.head != uart1_tx_rb.tail &&
           (huart1.Instance->SR & USART_SR_TXE) != 0U)
    {
        huart1.Instance->DR = uart1_tx_rb.buf[uart1_tx_rb.tail];
        uart1_tx_rb.tail = (uint16_t)((uart1_tx_rb.tail + 1U) % UART_TX_BUF_SIZE);
    }
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

/* Filter Median 5-Sampel: menolak gangguan/jitter hingga 2 frame berturut-turut */
static void RC_CommitFilteredChannels(const uint16_t raw_ch[16])
{
    for (uint8_t i = 0U; i < 16U; i++)
    {
        uint16_t new_us = RC_ToMicroseconds(raw_ch[i]);

        if (rc_first_frame_done == 0U)
        {
            for (uint8_t k = 0U; k < 5U; k++)
            {
                sbus_hist[i][k] = new_us;
            }
            sbus_us[i] = new_us;
        }
        else
        {
            sbus_hist[i][0] = sbus_hist[i][1];
            sbus_hist[i][1] = sbus_hist[i][2];
            sbus_hist[i][2] = sbus_hist[i][3];
            sbus_hist[i][3] = sbus_hist[i][4];
            sbus_hist[i][4] = new_us;

            uint16_t med_us = median5_u16(
                sbus_hist[i][0], sbus_hist[i][1], sbus_hist[i][2],
                sbus_hist[i][3], sbus_hist[i][4]);

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

/* Seqlock: ganjil = penulis sedang menulis, genap = data konsisten tanpa matikan IRQ */
static volatile uint32_t rc_seq = 0U;
#define RC_SeqBegin() do { rc_seq++; __DMB(); } while (0)
#define RC_SeqEnd()   do { __DMB(); rc_seq++; } while (0)

static void RC_GetAtomicSnapshot(uint16_t dst[16], uint32_t *last_ms_out)
{
    uint32_t s1, s2;
    uint8_t tries = 0U;

    do
    {
        s1 = rc_seq;
        __DMB();
        for (uint8_t i = 0U; i < 16U; i++)
        {
            dst[i] = sbus_us[i];
        }
        *last_ms_out = rc_last_frame_ms;
        __DMB();
        s2 = rc_seq;
        tries++;
    } while (((s1 != s2) || ((s1 & 1U) != 0U)) && (tries < 4U));
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

    RC_SeqBegin();
    RC_CommitFilteredChannels(sbus_ch);
    sbus_failsafe = 0U;
    rc_proto_locked = 1U;
    rc_valid_frames++;
    rc_last_frame_ms = HAL_GetTick();
    RC_SeqEnd();
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

    if (buf[2] == CRSF_FRAMETYPE_RC && payload_plus_type_len >= 24U)
    {
        RC_Unpack16Channels(&buf[3], sbus_ch);

        RC_SeqBegin();
        RC_CommitFilteredChannels(sbus_ch);
        sbus_failsafe = 0U;
        /* Hanya frame kanal RC yang memperpanjang hold; frame LinkStats/telemetri
           tidak boleh membuat stick lama dianggap masih segar. */
        rc_last_frame_ms = HAL_GetTick();
        RC_SeqEnd();
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

static void UART_ServiceBackground(void)
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

    Uart1Tx_ServiceNonBlocking();
}

static void MAVLink_SendPacketNonBlocking(const mavlink_message_t *msg)
{
    uint16_t len = (uint16_t)mavlink_msg_to_send_buffer(pkt_buf, msg);
    (void)CDC_Transmit_FS(pkt_buf, len);
    Uart1Tx_Enqueue(pkt_buf, len);
    Uart1Tx_ServiceNonBlocking();
}

static void MAVLink_SendStatusText(uint8_t severity, const char *fmt_msg)
{
    char text_buf[50];
    memset(text_buf, 0, sizeof(text_buf));
    strncpy(text_buf, fmt_msg, sizeof(text_buf) - 1U);

    (void)mavlink_msg_statustext_pack(
        SYS_ID, COMP_ID_FC, &msg_tx,
        severity, text_buf, 0U, 0U);

    MAVLink_SendPacketNonBlocking(&msg_tx);
}

static void MAVLink_SendParamValue(uint16_t idx)
{
    if (idx >= PARAM_COUNT)
    {
        return;
    }

    (void)mavlink_msg_param_value_pack(
        SYS_ID, COMP_ID_FC, &msg_tx,
        param_table[idx].name,
        *param_table[idx].val_ptr,
        MAV_PARAM_TYPE_REAL32,
        PARAM_COUNT,
        idx);

    MAVLink_SendPacketNonBlocking(&msg_tx);
}

static void MAVLink_SendCommandAck(uint16_t command, uint8_t result)
{
    (void)mavlink_msg_command_ack_pack(
        SYS_ID, COMP_ID_FC, &msg_tx,
        command, result, 0U, 0, 0U, 0U);

    MAVLink_SendPacketNonBlocking(&msg_tx);
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

static void I2C1_Delay5us(void)
{
    uint32_t ticks = SystemCoreClock / 200000U; /* ~5 us (100 kHz half-period) */
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U)
    {
        uint32_t start = DWT->CYCCNT;
        while ((DWT->CYCCNT - start) < ticks) { }
    }
    else
    {
        for (volatile uint32_t i = 0U; i < (ticks / 4U); i++) { }
    }
}

/* Lepas bus I2C1 yang macet (SDA tertahan LOW) dengan hingga 9 pulsa SCL (PB6) + STOP */
static void I2C1_RecoverBus(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    (void)HAL_I2C_DeInit(&hi2c1);

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Mode Open-Drain + Pull-Up agar tidak bentrok jika slave masih menarik jalur LOW */
    HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_PIN_SET);

    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Pin = I2C1_SCL_PIN;
    HAL_GPIO_Init(I2C1_SCL_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = I2C1_SDA_PIN;
    HAL_GPIO_Init(I2C1_SDA_PORT, &GPIO_InitStruct);

    I2C1_Delay5us();

    /* Kirim hingga 9 pulsa clock sampai slave melepas SDA (HIGH) */
    for (uint8_t i = 0U; i < 9U; i++)
    {
        if (HAL_GPIO_ReadPin(I2C1_SDA_PORT, I2C1_SDA_PIN) == GPIO_PIN_SET)
        {
            break;
        }
        HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_RESET);
        I2C1_Delay5us();
        HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_SET);
        I2C1_Delay5us();
    }

    /* Bangkitkan kondisi STOP (SCL LOW -> SDA LOW -> SCL HIGH -> SDA HIGH) */
    HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_RESET);
    I2C1_Delay5us();
    HAL_GPIO_WritePin(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_PIN_RESET);
    I2C1_Delay5us();
    HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_SET);
    I2C1_Delay5us();
    HAL_GPIO_WritePin(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_PIN_SET);
    I2C1_Delay5us();

    /* Kembalikan pin PB6/PB7 ke fungsi alternatif hardware I2C1 */
    (void)HAL_I2C_Init(&hi2c1);
}

static HAL_StatusTypeDef MPU_Configure(void)
{
    uint8_t wake = 0x00U;   /* PWR_MGMT_1: bangunkan sensor */
    uint8_t fs   = 0x00U;   /* Accel & gyro full-scale default */
    uint8_t dlpf = 0x04U;   /* DLPF 20 Hz, sama seperti init awal */

    if (HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x6BU, I2C_MEMADD_SIZE_8BIT, &wake, 1U, 5U) != HAL_OK) return HAL_ERROR;
    if (HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x1CU, I2C_MEMADD_SIZE_8BIT, &fs,   1U, 5U) != HAL_OK) return HAL_ERROR;
    if (HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x1BU, I2C_MEMADD_SIZE_8BIT, &fs,   1U, 5U) != HAL_OK) return HAL_ERROR;
    if (HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x1AU, I2C_MEMADD_SIZE_8BIT, &dlpf, 1U, 5U) != HAL_OK) return HAL_ERROR;
    return HAL_OK;
}

/* Baca MPU + cek kesehatan. Return 1 jika data layak dipakai siklus ini. */
static uint8_t MPU_ReadChecked(void)
{
    if (mpu.ready == 0U)
    {
        return 0U;
    }

    uint8_t good = 0U;

    if (HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x3BU,
                         I2C_MEMADD_SIZE_8BIT, mpu.data_mentah, 14U, 5U) == HAL_OK)
    {
        /* 14 byte identik berturut-turut = sensor tidur/beku walau I2C "sukses" */
        if (memcmp(mpu.data_mentah, mpu_prev_raw, 14U) == 0)
        {
            if (mpu_frozen_count < 255U) mpu_frozen_count++;
        }
        else
        {
            mpu_frozen_count = 0U;
        }
        memcpy(mpu_prev_raw, mpu.data_mentah, 14U);
        good = (mpu_frozen_count < MPU_FAIL_LIMIT) ? 1U : 0U;
    }

    if (good != 0U)
    {
        mpu_fail_count = 0U;
        if (mpu_ok_count < 255U) mpu_ok_count++;
        if ((mpu_fault != 0U) && (mpu_ok_count >= MPU_OK_RECOVER))
        {
            mpu_fault = 0U;   /* Mode TIDAK otomatis kembali: pilot harus geser CH5 */
        }
    }
    else
    {
        mpu_ok_count = 0U;
        if (mpu_fail_count < 255U) mpu_fail_count++;

        if (mpu_fail_count >= MPU_FAIL_LIMIT)
        {
            uint32_t t = HAL_GetTick();
            if ((mpu_fault == 0U) || ((t - mpu_last_recover_ms) >= MPU_RECOVER_RETRY_MS))
            {
                mpu_fault = 1U;
                mpu_last_recover_ms = t;
                I2C1_RecoverBus();
                (void)MPU_Configure();
            }
        }
    }

    return good;
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
    /* Jika reset berasal dari watchdog (mis. saat terbang), mulai dalam keadaan fault:
       paksa MANUAL sampai MPU sehat 0,5 s dan pilot menggeser CH5. Bias gyro hasil
       kalibrasi saat bergerak tidak bisa dipercaya penuh. */
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
    {
        mpu_fault = 1U;
    }
    __HAL_RCC_CLEAR_RESET_FLAGS();

    HAL_NVIC_SetPriority(SysTick_IRQn, 2U, 0U);
    HAL_NVIC_SetPriority(OTG_FS_IRQn, 3U, 0U);
    HAL_NVIC_SetPriority(USART1_IRQn, 2U, 0U);

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    usart2_irq_available = (NVIC_GetEnableIRQ(USART2_IRQn) != 0U) ? 1U : 0U;

    /* Aktifkan OC Preload pada TIM3 CH1-CH4 agar pulsa PWM tidak pernah terpotong */
    SET_BIT(htim3.Instance->CCMR1, TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);
    SET_BIT(htim3.Instance->CCMR2, TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm.aileron_L);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm.aileron_R);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm.elevator);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm.rudder);

    HAL_Delay(300U);

    /* Bersihkan bus I2C1 saat boot (antisipasi reset IWDG terjadi di tengah transaksi I2C) */
    I2C1_RecoverBus();

    /* -------- Inisialisasi MPU6050 (Original & Clone) + Kalibrasi Gyro -------- */
    if (HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x75U,
                         I2C_MEMADD_SIZE_8BIT, &mpu.identitas, 1U, 50U) == HAL_OK &&
        (mpu.identitas != 0x00U) && (mpu.identitas != 0xFFU))
    {
        uint8_t data_konfig = 0x00U;
        uint8_t dlpf_cfg    = 0x04U; /* Hardware DLPF 20 Hz untuk meredam getaran mekanis */
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

    uint32_t boot_ms  = HAL_GetTick();
    bmp.waktu_baro    = boot_ms;
    waktu_ctrl        = boot_ms;
    waktu_hb          = boot_ms;
    waktu_led         = boot_ms;
    waktu_telem       = boot_ms;
    waktu_rc_diag     = boot_ms;
    rc_last_switch_ms = boot_ms;

    RC_SetProtocol(RC_PROTO_SBUS_INV);

    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_CLEAR_FEFLAG(&huart1);
    __HAL_UART_CLEAR_NEFLAG(&huart1);
    if (NVIC_GetEnableIRQ(USART1_IRQn) != 0U)
    {
        (void)HAL_UART_Receive_IT(&huart1, &radio_rx_byte, 1U);
    }

    /* Aktifkan Independent Watchdog (~250 ms nominal) */
    hiwdg.Instance       = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_16;
    hiwdg.Init.Reload    = 500U;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
    {
        Error_Handler();
    }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* Layanan latar belakang non-blocking (UART TX/RX + MAVLink masuk) */
        UART_ServiceBackground();
        MAVLink_ProcessIncoming();

        uint32_t now = HAL_GetTick();

        /* Loop Kendali & Sensor Deterministik Tepat 100 Hz (10.0 ms) */
        if ((now - waktu_ctrl) >= CTRL_LOOP_MS)
        {
            if ((now - waktu_ctrl) > 50U)
            {
                waktu_ctrl = now;
            }
            else
            {
                waktu_ctrl += CTRL_LOOP_MS;
            }
            mpu.dt = 0.01f;

            BMP280_UpdateAltitude(&bmp);

            /* 1. Baca & Filter Biquad Orde-2 MPU6050 dengan Deteksi Fault/Freeze */
            if (MPU_ReadChecked() != 0U)
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

                /* Filter Biquad 15 Hz pada laju gyro untuk memotong getaran frekuensi tinggi */
                mpu.gyroX_filt = biquad15Hz_Update(&gyro_lpf_x, mpu.gyroX_dps);
                mpu.gyroY_filt = biquad15Hz_Update(&gyro_lpf_y, mpu.gyroY_dps);
                mpu.gyroZ_filt = biquad15Hz_Update(&gyro_lpf_z, mpu.gyroZ_dps);

                mpu.kalmanX += mpu.gyroX_filt * mpu.dt;
                mpu.kalmanY += mpu.gyroY_filt * mpu.dt;
                mpu.kalmanZ += mpu.gyroZ_filt * mpu.dt;

                /* G-Force Gate: koreksi accelerometer hanya saat bebas getaran (0.85g - 1.15g) */
                float acc_mag = sqrtf((mpu.accX_g * mpu.accX_g) +
                                      (mpu.accY_g * mpu.accY_g) +
                                      (mpu.accZ_g * mpu.accZ_g));
                if (acc_mag >= 0.85f && acc_mag <= 1.15f)
                {
                    mpu.sudut_accX = atan2f(mpu.accY_g, mpu.accZ_g) * RAD_TO_DEG;
                    mpu.sudut_accY = atan2f(-mpu.accX_g, mpu.accZ_g) * RAD_TO_DEG;

                    mpu.err_gyroX += mpu.Q_drift;
                    mpu.kalman_gainX = mpu.err_gyroX / (mpu.err_gyroX + mpu.E_acc);
                    mpu.kalmanX += mpu.kalman_gainX * (mpu.sudut_accX - mpu.kalmanX);
                    mpu.err_gyroX = (1.0f - mpu.kalman_gainX) * mpu.err_gyroX;

                    mpu.err_gyroY += mpu.Q_drift;
                    mpu.kalman_gainY = mpu.err_gyroY / (mpu.err_gyroY + mpu.E_acc);
                    mpu.kalmanY += mpu.kalman_gainY * (mpu.sudut_accY - mpu.kalmanY);
                    mpu.err_gyroY = (1.0f - mpu.kalman_gainY) * mpu.err_gyroY;
                }

                /* Sudut dari Kalman sudah halus (gyro sudah difilter biquad);
                   filter sudut tambahan hanya menambah lag di dalam loop PID. */
                mpu.roll_filt  = mpu.kalmanX;
                mpu.pitch_filt = mpu.kalmanY;
            }

            /* 2. Ambil Snapshot Seqlock Kanal RC + Last-Value Hold 300 ms */
            uint16_t snap_rc_us[16];
            uint32_t snap_frame_ms = 0U;
            RC_GetAtomicSnapshot(snap_rc_us, &snap_frame_ms);

            /* Umur frame bertanda: frame yang masuk SETELAH 'now' diambil (umur negatif)
               tetap dianggap segar, bukan underflow jadi angka raksasa. */
            int32_t rc_age = (int32_t)(now - snap_frame_ms);
            uint8_t rc_hold_active = (snap_frame_ms != 0U &&
                                      rc_age <= (int32_t)RC_HOLD_TIMEOUT_MS) ? 1U : 0U;

            if (rc_hold_active == 0U && rc_proto_locked == 0U)
            {
                if ((now - rc_last_switch_ms) >= RC_PROTO_SWITCH_MS)
                {
                    rc_last_switch_ms = now;
                    uint8_t next_proto = (uint8_t)((rc_protocol_mode + 1U) % RC_PROTO_COUNT);
                    RC_SetProtocol(next_proto);
                }
            }
            else
            {
                rc_last_switch_ms = now;
            }

            static const uint16_t default_rc_us[16] = {
                1500U, 1500U, 1000U, 1500U, 1000U, 1500U, 1500U, 1500U,
                1500U, 1500U, 1500U, 1500U, 1500U, 1500U, 1500U, 1500U
            };

            uint8_t mav_rc_ok = (mav_rc_active_mask != 0U &&
                                 (now - mav_rc_last_ms) <= MAV_RC_TIMEOUT_MS) ? 1U : 0U;

            if (mav_rc_ok == 0U)
            {
                mav_rc_active_mask = 0U;
            }

            for (uint8_t i = 0U; i < 16U; i++)
            {
                active_rc_us[i] = (rc_hold_active != 0U) ? snap_rc_us[i] : default_rc_us[i];

                if (mav_rc_ok != 0U && ((mav_rc_active_mask & (uint16_t)(1U << i)) != 0U))
                {
                    active_rc_us[i] = mav_rc_us[i];
                }
            }

            float raw_roll_cmd  = shape_stick_us(active_rc_us[0], STICK_DEADBAND_US);
            float raw_pitch_cmd = shape_stick_us(active_rc_us[1], STICK_DEADBAND_US);
            float raw_yaw_cmd   = shape_stick_us(active_rc_us[3], STICK_DEADBAND_US);

            filt_stick_roll_us  += 0.45f * (raw_roll_cmd  - filt_stick_roll_us);
            filt_stick_pitch_us += 0.45f * (raw_pitch_cmd - filt_stick_pitch_us);
            filt_stick_yaw_us   += 0.45f * (raw_yaw_cmd   - filt_stick_yaw_us);

            if (fabsf(filt_stick_roll_us)  < 2.0f) filt_stick_roll_us  = 0.0f;
            if (fabsf(filt_stick_pitch_us) < 2.0f) filt_stick_pitch_us = 0.0f;
            if (fabsf(filt_stick_yaw_us)   < 2.0f) filt_stick_yaw_us   = 0.0f;

            /* 3. Pemilihan Mode pada CH5 dengan Hysteresis:
                  < 1250 = STABILIZE (PID Aktif Default)
                  1300 - 1700 = AUTO (PID + AltHold)
                  > 1750 = MANUAL (Cadangan) */
            uint16_t ch5 = active_rc_us[4];
            int32_t ch5_delta = (int32_t)ch5 - (int32_t)ch5_last_us;
            if (ch5_delta < 0) ch5_delta = -ch5_delta;

            if (ch5_initialized == 0U || ch5_delta > 40)
            {
                ch5_initialized = 1U;
                ch5_last_us = ch5;

                if (flight_mode == PLANE_MODE_STABILIZE)
                {
                    if (ch5 > 1750U)
                    {
                        flight_mode = PLANE_MODE_MANUAL;
                    }
                    else if (ch5 > 1300U)
                    {
                        flight_mode = PLANE_MODE_AUTO;
                    }
                }
                else if (flight_mode == PLANE_MODE_AUTO)
                {
                    if (ch5 < 1250U)
                    {
                        flight_mode = PLANE_MODE_STABILIZE;
                    }
                    else if (ch5 > 1750U)
                    {
                        flight_mode = PLANE_MODE_MANUAL;
                    }
                }
                else /* PLANE_MODE_MANUAL */
                {
                    if (ch5 < 1250U)
                    {
                        flight_mode = PLANE_MODE_STABILIZE;
                    }
                    else if (ch5 < 1700U)
                    {
                        flight_mode = PLANE_MODE_AUTO;
                    }
                }
            }

            if ((mpu.ready == 0U) || (mpu_fault != 0U))
            {
                flight_mode = PLANE_MODE_MANUAL;
            }

            if (flight_mode != prev_flight_mode)
            {
                AutoPID_Reset(bmp.ketinggian_filter, filt_stick_roll_us, filt_stick_pitch_us, filt_stick_yaw_us);
                prev_flight_mode = flight_mode;
            }

            /* 4. Perhitungan Kendali PID (STABILIZE / AUTO) vs MANUAL */
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
                        float alt_err = smooth_deadband_f(pid.target_alt_m - bmp.ketinggian_filter, 0.30f, 0.50f);
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

                /* Kurva deadband kubik halus pada sudut & gyro agar bebas osilasi di meja */
                float err_roll  = smooth_deadband_f(target_roll_deg  - mpu.roll_filt,  1.8f, 3.0f);
                float err_pitch = smooth_deadband_f(target_pitch_deg - mpu.pitch_filt, 1.8f, 3.0f);

                float gx_clean = smooth_deadband_f(mpu.gyroX_filt, 1.2f, 2.5f);
                float gy_clean = smooth_deadband_f(mpu.gyroY_filt, 1.2f, 2.5f);
                float gz_clean = smooth_deadband_f(mpu.gyroZ_filt, 1.2f, 2.5f);

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

                float raw_pid_roll  = (pid.kp_roll  * err_roll)  + pid.i_roll  - (pid.kd_roll  * gx_clean);
                float raw_pid_pitch = (pid.kp_pitch * err_pitch) + pid.i_pitch - (pid.kd_pitch * gy_clean);
                float raw_pid_yaw   = (stick_yaw * 500.0f) - (pid.kp_yaw * gz_clean);

                raw_pid_roll  = clamp_f(raw_pid_roll,  -500.0f, 500.0f);
                raw_pid_pitch = clamp_f(raw_pid_pitch, -500.0f, 500.0f);
                raw_pid_yaw   = clamp_f(raw_pid_yaw,   -500.0f, 500.0f);

                pid.smooth_roll_us  += PID_OUT_SMOOTH * (raw_pid_roll  - pid.smooth_roll_us);
                pid.smooth_pitch_us += PID_OUT_SMOOTH * (raw_pid_pitch - pid.smooth_pitch_us);
                pid.smooth_yaw_us   += PID_OUT_SMOOTH * (raw_pid_yaw   - pid.smooth_yaw_us);

                out_roll_us  = pid.smooth_roll_us;
                out_pitch_us = pid.smooth_pitch_us;
                out_yaw_us   = pid.smooth_yaw_us;
            }

            /* 5. Slew-Rate Limiter + Backlash Hysteresis PWM ke 4 Servo (bypass di MANUAL) */
            float desire_ail_L = clamp_f(1500.0f + pwm.trim_roll_L + (pwm.dir_roll_L * out_roll_us),  1000.0f, 2000.0f);
            float desire_ail_R = clamp_f(1500.0f + pwm.trim_roll_R + (pwm.dir_roll_R * out_roll_us),  1000.0f, 2000.0f);
            float desire_ele   = clamp_f(1500.0f + pwm.trim_pitch  + (pwm.dir_pitch  * out_pitch_us), 1000.0f, 2000.0f);
            float desire_rud   = clamp_f(1500.0f + pwm.trim_yaw    + (pwm.dir_yaw    * out_yaw_us),   1000.0f, 2000.0f);

            const uint8_t is_manual = (flight_mode == PLANE_MODE_MANUAL) ? 1U : 0U;
            const float   slew_lim  = (is_manual != 0U) ? SERVO_SLEW_MANUAL_US : SERVO_SLEW_MAX_US;
            const int32_t hyst_us   = (is_manual != 0U) ? SERVO_HYST_MANUAL_US : SERVO_HYST_US;

            servo_slew_ail_L = slew_limit_f(servo_slew_ail_L, desire_ail_L, slew_lim);
            servo_slew_ail_R = slew_limit_f(servo_slew_ail_R, desire_ail_R, slew_lim);
            servo_slew_ele   = slew_limit_f(servo_slew_ele,   desire_ele,   slew_lim);
            servo_slew_rud   = slew_limit_f(servo_slew_rud,   desire_rud,   slew_lim);

            uint16_t target_ail_L = (uint16_t)(servo_slew_ail_L + 0.5f);
            uint16_t target_ail_R = (uint16_t)(servo_slew_ail_R + 0.5f);
            uint16_t target_ele   = (uint16_t)(servo_slew_ele   + 0.5f);
            uint16_t target_rud   = (uint16_t)(servo_slew_rud   + 0.5f);

            pwm.aileron_L = apply_hyst_u16(pwm.aileron_L, target_ail_L, hyst_us);
            pwm.aileron_R = apply_hyst_u16(pwm.aileron_R, target_ail_R, hyst_us);
            pwm.elevator  = apply_hyst_u16(pwm.elevator,  target_ele,   hyst_us);
            pwm.rudder    = apply_hyst_u16(pwm.rudder,    target_rud,   hyst_us);

            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm.aileron_L);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm.aileron_R);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm.elevator);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm.rudder);

            /* 6. Penjadwalan MAVLink Bertahap (1 paket tiap 40 ms -> 0 ms CPU blocking) */
            if ((now - waktu_telem) >= TELEM_STEP_MS)
            {
                waktu_telem = now;

                if ((now - waktu_hb) >= 1000U)
                {
                    waktu_hb = now;
                    mavlink_msg_heartbeat_pack(
                        SYS_ID, COMP_ID_FC, &msg_tx,
                        MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_ARDUPILOTMEGA,
                        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED,
                        flight_mode, MAV_STATE_ACTIVE);
                    MAVLink_SendPacketNonBlocking(&msg_tx);
                }
                else
                {
                    switch (telem_slot)
                    {
                        case 0U:
                        case 3U:
                            mavlink_msg_attitude_pack(
                                SYS_ID, COMP_ID_FC, &msg_tx, now,
                                mpu.roll_filt * DEG_TO_RAD, mpu.pitch_filt * DEG_TO_RAD, mpu.kalmanZ * DEG_TO_RAD,
                                mpu.gyroX_filt * DEG_TO_RAD, mpu.gyroY_filt * DEG_TO_RAD, mpu.gyroZ_filt * DEG_TO_RAD);
                            MAVLink_SendPacketNonBlocking(&msg_tx);
                            break;

                        case 1U:
                        case 4U:
                        {
                            uint8_t rc_rssi = ((rc_hold_active != 0U) || (mav_rc_ok != 0U)) ? 255U : 0U;
                            mavlink_msg_rc_channels_pack(
                                SYS_ID, COMP_ID_FC, &msg_tx,
                                now, 16U,
                                active_rc_us[0],  active_rc_us[1],  active_rc_us[2],  active_rc_us[3],
                                active_rc_us[4],  active_rc_us[5],  active_rc_us[6],  active_rc_us[7],
                                active_rc_us[8],  active_rc_us[9],  active_rc_us[10], active_rc_us[11],
                                active_rc_us[12], active_rc_us[13], active_rc_us[14], active_rc_us[15],
                                UINT16_MAX, UINT16_MAX, rc_rssi);
                            MAVLink_SendPacketNonBlocking(&msg_tx);
                            break;
                        }

                        case 2U:
                            mavlink_msg_servo_output_raw_pack(
                                SYS_ID, COMP_ID_FC, &msg_tx,
                                now * 1000U, 0U,
                                pwm.aileron_L, pwm.aileron_R, pwm.elevator, pwm.rudder,
                                0U, 0U, 0U, 0U,
                                0U, 0U, 0U, 0U,
                                0U, 0U, 0U, 0U);
                            MAVLink_SendPacketNonBlocking(&msg_tx);
                            break;

                        default:
                        {
                            float heading_deg_f = fmodf(mpu.kalmanZ, 360.0f);
                            if (heading_deg_f < 0.0f) heading_deg_f += 360.0f;
                            int16_t  heading_deg  = (int16_t)heading_deg_f;
                            uint16_t heading_cdeg = (uint16_t)(heading_deg_f * 100.0f);
                            int32_t  alt_mm       = (int32_t)(bmp.ketinggian_filter * 1000.0f);
                            int16_t  vz_cms       = (int16_t)(-bmp.climb_mps * 100.0f);

                            mavlink_msg_global_position_int_pack(
                                SYS_ID, COMP_ID_FC, &msg_tx, now,
                                0, 0, alt_mm, alt_mm, 0, 0, vz_cms, heading_cdeg);
                            MAVLink_SendPacketNonBlocking(&msg_tx);

                            uint16_t thr_pct = (active_rc_us[2] <= 1000U) ? 0U : (uint16_t)((active_rc_us[2] - 1000U) / 10U);
                            if (thr_pct > 100U) thr_pct = 100U;
                            mavlink_msg_vfr_hud_pack(
                                SYS_ID, COMP_ID_FC, &msg_tx,
                                0.0f, 0.0f, heading_deg, thr_pct,
                                bmp.ketinggian_filter, bmp.climb_mps);
                            MAVLink_SendPacketNonBlocking(&msg_tx);
                            break;
                        }
                    }
                    telem_slot = (uint8_t)((telem_slot + 1U) % 6U);
                }
            }

            /* 7. Laporan Status PID, Sensor & Statistik Link RC ke Mission Planner */
            if ((now - waktu_rc_diag) >= RC_DIAG_MS)
            {
                waktu_rc_diag = now;
                char diag_str[50];
                const char *mode_name = (flight_mode == PLANE_MODE_STABILIZE) ? "PID-STAB" :
                                        ((flight_mode == PLANE_MODE_AUTO) ? "PID-AUTO" : "MANUAL");
                const uint8_t mpu_ok = ((mpu.ready != 0U) && (mpu_fault == 0U)) ? 1U : 0U;

                (void)snprintf(diag_str, sizeof(diag_str),
                    "[%s] MPU:%s R:%.1f P:%.1f CH5:%u",
                    mode_name,
                    (mpu_ok != 0U) ? "OK" : "ERR",
                    (double)mpu.roll_filt,
                    (double)mpu.pitch_filt,
                    active_rc_us[4]);
                MAVLink_SendStatusText((mpu_ok != 0U) ? MAV_SEVERITY_INFO : MAV_SEVERITY_WARNING, diag_str);

                /* Statistik link RC per 2 detik (integer saja, aman tanpa printf-float):
                   p  = protokol terdeteksi (0 SBUS-inverted, 1 SBUS, 2 CRSF)
                   L  = 1 jika protokol sudah terkunci
                   fr = frame valid (CRSF: termasuk frame LinkStats, jadi sedikit lebih banyak dari laju RC)
                   er = error UART RC, gl = lonjakan > 20 us yang diredam filter median */
                static uint32_t fr_prev = 0U, er_prev = 0U, gl_prev = 0U;
                const uint32_t fr_now = rc_valid_frames;
                const uint32_t er_now = rc_uart_errors;
                const uint32_t gl_now = rc_glitches_blocked;

                (void)snprintf(diag_str, sizeof(diag_str),
                    "RC p:%u L:%u fr:%lu er:%lu gl:%lu",
                    (unsigned)rc_protocol_mode,
                    (unsigned)rc_proto_locked,
                    (unsigned long)(fr_now - fr_prev),
                    (unsigned long)(er_now - er_prev),
                    (unsigned long)(gl_now - gl_prev));
                MAVLink_SendStatusText(MAV_SEVERITY_INFO, diag_str);

                fr_prev = fr_now;
                er_prev = er_now;
                gl_prev = gl_now;
            }

            uint32_t led_interval = (flight_mode == PLANE_MODE_MANUAL) ? 300U : 100U;
            if ((now - waktu_led) >= led_interval)
            {
                waktu_led = now;
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            }

            /* Refresh Watchdog hanya jika loop kontrol 100 Hz berjalan normal */
            (void)HAL_IWDG_Refresh(&hiwdg);
        }
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
