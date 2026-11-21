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
    uint8_t  sehat;
    uint16_t dig_T1, dig_P1;
    int16_t  dig_T2, dig_T3;
    int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

    float    tekanan_pa;
    float    tekanan_hpa;
    float    tekanan_ref_p0;
    float    ketinggian_m;
    float    ketinggian_filter;
    float    climb_mps;
    uint32_t waktu_baro;
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
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU_ADDR          (0x68U << 1)
#define BMP280_ADDR_LOW   (0x76U << 1) /* Pin SDO ke GND */
#define BMP280_ADDR_HIGH  (0x77U << 1) /* Pin SDO ke 3.3V */

#define RAD_TO_DEG        57.295779513f
#define DEG_TO_RAD        0.01745329252f

#define SYS_ID            1U
#define COMP_ID_KALMAN    1U
#define COMP_ID_COMP      2U

#define BARO_UPDATE_MS    50U          /* Update pembacaan BMP280 tiap 50 ms (20 Hz) */
#define ATT_TELEM_MS      20U          /* Kirim Attitude ke GCS tiap 20 ms (50 Hz) */
#define ALT_TELEM_MS      50U          /* Kirim Altitude & VFR_HUD tiap 50 ms (20 Hz) */
#define EXTRA_TELEM_MS    100U         /* Kirim Servo & Tekanan Baro tiap 100 ms (10 Hz) */
#define RADIO_TELEM_MS    50U          /* Kirim telemetri radio USART1 tiap 50 ms (20 Hz) */

/* Konstanta SBUS & Mode Terbang */
#define SBUS_FRAME_LEN    25U
#define SBUS_HEADER       0x0FU
#define SBUS_TIMEOUT_MS   100U

#define MODE_MANUAL       0U           /* Kendali penuh stick pilot (tanpa koreksi PID) */
#define MODE_STABILIZE    1U           /* Auto-level: PID menjaga sudut sesuai arah stick */
#define MODE_ALTHOLD      2U           /* Altitude Hold: PID menjaga sudut & ketinggian baro */
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

/* ------------------------------- MAVLink -------------------------------- */
uint32_t waktu_sekarang = 0;
uint32_t waktu_sebelumnya = 0;
uint32_t waktu_hb = 0;
uint32_t waktu_att = 0;
uint32_t waktu_alt_msg = 0;
uint32_t waktu_extra = 0;
uint32_t waktu_led = 0;
uint32_t waktu_radio = 0;

mavlink_message_t msg_kalman;
mavlink_message_t msg_comp;
mavlink_message_t msg_hb;
mavlink_message_t msg_vfr;
mavlink_message_t msg_gpos;
mavlink_message_t msg_alt;
mavlink_message_t msg_press;
mavlink_message_t msg_servo;
uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN * 8];

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
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Auto-detect alamat BMP280 (0x76 atau 0x77) dan baca koefisien kalibrasi */
HAL_StatusTypeDef BMP280_Init(BMP280_t *dev)
{
    uint8_t calib[24];
    uint8_t config = 0x10U;     /* IIR filter x16 */
    uint8_t ctrl_meas = 0x57U;  /* Temp x2, Pressure x16, Normal mode */

    dev->sehat = 0U;

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

    dev->sehat = 1U;
    return HAL_OK;
}

float BMP280_ReadPressure(BMP280_t *dev)
{
    uint8_t raw[6];

    if (dev->addr == 0U)
    {
        dev->sehat = 0U;
        return NAN;
    }

    if (HAL_I2C_Mem_Read(&hi2c1, dev->addr, 0xF7U,
                         I2C_MEMADD_SIZE_8BIT, raw, sizeof(raw), 5U) != HAL_OK)
    {
        dev->sehat = 0U;
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

    int64_t p_var1 = ((int64_t)t_fine) - 128000;
    int64_t p_var2 = p_var1 * p_var1 * (int64_t)dev->dig_P6;
    p_var2 += (p_var1 * (int64_t)dev->dig_P5) << 17;
    p_var2 += ((int64_t)dev->dig_P4) << 35;
    p_var1 = ((p_var1 * p_var1 * (int64_t)dev->dig_P3) >> 8) +
             ((p_var1 * (int64_t)dev->dig_P2) << 12);
    p_var1 = (((((int64_t)1) << 47) + p_var1) * (int64_t)dev->dig_P1) >> 33;

    if (p_var1 == 0)
    {
        dev->sehat = 0U;
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
        dev->sehat = 0U;
        return NAN;
    }

    dev->sehat = 1U;
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

    /* Tunggu 500 ms dan buang 15 pembacaan awal agar filter IIR x16 BMP280 stabil */
    HAL_Delay(500U);
    for (uint8_t i = 0U; i < 15U; i++)
    {
        (void)BMP280_ReadPressure(&bmp);
        HAL_Delay(25U);
    }

    /* Ambil rata-rata 20 sampel valid sebagai P0 (0.0 meter) */
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
        HAL_Delay(25U);
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
    waktu_att = bmp.waktu_baro;
    waktu_alt_msg = bmp.waktu_baro;
    waktu_extra = bmp.waktu_baro;
    waktu_led = bmp.waktu_baro;
    waktu_radio = bmp.waktu_baro;

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
        mpu.gyroX_bias = sum_gx / (float)gyro_
