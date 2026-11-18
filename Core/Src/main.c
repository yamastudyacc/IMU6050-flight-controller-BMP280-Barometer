/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : MPU6050 + BMP280 Barometer + MAVLink v2 over USB CDC
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

#define BARO_UPDATE_MS    50U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* ------------------------------ MPU6050 ---------------------------------- */
uint8_t identitas = 0;
uint8_t data_mentah[14];

int16_t accX_raw, accY_raw, accZ_raw;
int16_t gyroX_raw, gyroY_raw, gyroZ_raw;
int16_t temp_raw;

float accX_g, accY_g, accZ_g;
float gyroX_dps, gyroY_dps, gyroZ_dps;
float suhu_c;
float sudut_accX, sudut_accY;
float dt = 0.01f;

float compX = 0.0f, compY = 0.0f, compZ = 0.0f;

float kalmanX = 0.0f, kalmanY = 0.0f, kalmanZ = 0.0f;
float err_gyroX = 1.0f, err_gyroY = 1.0f;
float kalman_gainX = 0.0f, kalman_gainY = 0.0f;
float Q_drift = 0.003f;
float E_acc = 0.03f;

/* ------------------------------- BMP280 --------------------------------- */
uint8_t bmp280_addr = 0U;
uint16_t dig_T1, dig_P1;
int16_t  dig_T2, dig_T3;
int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

float tekanan_pa = 101325.0f;
float tekanan_hpa = 1013.25f;
float tekanan_ref_p0 = 101325.0f;
float ketinggian_m = 0.0f;
float ketinggian_filter = 0.0f;
float climb_mps = 0.0f;
uint32_t waktu_baro = 0;

/* ------------------------------- MAVLink -------------------------------- */
uint32_t waktu_sekarang = 0;
uint32_t waktu_sebelumnya = 0;
uint32_t waktu_hb = 0;
uint32_t waktu_led = 0;

mavlink_message_t msg_kalman;
mavlink_message_t msg_comp;
mavlink_message_t msg_hb;
mavlink_message_t msg_vfr;
uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN * 4];
/* ------------------------------- PID & Servo ---------------------------- */
PID_t pid_roll  = { .kp = 7.5f, .ki = 1.2f, .kd = 0.65f, .integral = 0.0f, .i_limit = 120.0f, .out_limit = 400.0f };
PID_t pid_pitch = { .kp = 8.0f, .ki = 1.2f, .kd = 0.70f, .integral = 0.0f, .i_limit = 120.0f, .out_limit = 400.0f };

/* Gain Altitude Hold (Outer Loop Barometer -> Target Pitch) & Yaw Damper */
float kp_alt = 2.0f;
float kd_alt = 1.5f;
float kd_yaw = 3.0f;

/* Mode Altitude Hold: 0 = Off (target pitch 0 derajat), 1 = On (menahan ketinggian target_alt) */
uint8_t enable_alt_hold = 0U;
float target_alt   = 0.0f;   /* Target ketinggian (meter) */
float target_pitch = 0.0f;   /* Sudut pitch yang diminta ke elevator (derajat) */

/* Arah gerak servo (+1.0f atau -1.0f): ubah ke -1.0f jika arah gerak servo terbalik */
float dir_roll  = 1.0f;
float dir_pitch = 1.0f;
float dir_yaw   = 1.0f;

/* Trim mekanik (us) jika posisi netral linkage sedikit meleset dari 1500 us */
float trim_roll  = 0.0f;
float trim_pitch = 0.0f;
float trim_yaw   = 0.0f;

/* Variabel output PWM akhir (1000 - 2000 us) untuk dipantau di Live Expressions */
uint16_t pwm_aileron  = 1500U;
uint16_t pwm_elevator = 1500U;
uint16_t pwm_rudder   = 1500U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef BMP280_Init(void);
float BMP280_ReadPressure(void);
void BMP280_UpdateAltitude(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Auto-detect alamat BMP280 (0x76 atau 0x77) dan baca koefisien kalibrasi */
HAL_StatusTypeDef BMP280_Init(void)
{
    uint8_t calib[24];
    uint8_t config = 0x10U;     /* IIR filter x16 */
    uint8_t ctrl_meas = 0x57U;  /* Temp x2, Pressure x16, Normal mode */

    if (HAL_I2C_IsDeviceReady(&hi2c1, BMP280_ADDR_LOW, 3U, 100U) == HAL_OK)
    {
        bmp280_addr = BMP280_ADDR_LOW;
    }
    else if (HAL_I2C_IsDeviceReady(&hi2c1, BMP280_ADDR_HIGH, 3U, 100U) == HAL_OK)
    {
        bmp280_addr = BMP280_ADDR_HIGH;
    }
    else
    {
        return HAL_ERROR;
    }

    if (HAL_I2C_Mem_Read(&hi2c1, bmp280_addr, 0x88U,
                         I2C_MEMADD_SIZE_8BIT, calib, sizeof(calib), 100U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    dig_T1 = (uint16_t)(((uint16_t)calib[1] << 8) | calib[0]);
    dig_T2 = (int16_t)(((uint16_t)calib[3] << 8) | calib[2]);
    dig_T3 = (int16_t)(((uint16_t)calib[5] << 8) | calib[4]);
    dig_P1 = (uint16_t)(((uint16_t)calib[7] << 8) | calib[6]);
    dig_P2 = (int16_t)(((uint16_t)calib[9] << 8) | calib[8]);
    dig_P3 = (int16_t)(((uint16_t)calib[11] << 8) | calib[10]);
    dig_P4 = (int16_t)(((uint16_t)calib[13] << 8) | calib[12]);
    dig_P5 = (int16_t)(((uint16_t)calib[15] << 8) | calib[14]);
    dig_P6 = (int16_t)(((uint16_t)calib[17] << 8) | calib[16]);
    dig_P7 = (int16_t)(((uint16_t)calib[19] << 8) | calib[18]);
    dig_P8 = (int16_t)(((uint16_t)calib[21] << 8) | calib[20]);
    dig_P9 = (int16_t)(((uint16_t)calib[23] << 8) | calib[22]);

    if (dig_P1 == 0U)
    {
        return HAL_ERROR;
    }

    if (HAL_I2C_Mem_Write(&hi2c1, bmp280_addr, 0xF5U,
                          I2C_MEMADD_SIZE_8BIT, &config, 1U, 100U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (HAL_I2C_Mem_Write(&hi2c1, bmp280_addr, 0xF4U,
                          I2C_MEMADD_SIZE_8BIT, &ctrl_meas, 1U, 100U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

float BMP280_ReadPressure(void)
{
    uint8_t raw[6];

    if (bmp280_addr == 0U)
    {
        return NAN;
    }

    if (HAL_I2C_Mem_Read(&hi2c1, bmp280_addr, 0xF7U,
                         I2C_MEMADD_SIZE_8BIT, raw, sizeof(raw), 50U) != HAL_OK)
    {
        return NAN;
    }

    int32_t adc_P = (int32_t)(((uint32_t)raw[0] << 12) |
                              ((uint32_t)raw[1] << 4) | (raw[2] >> 4));
    int32_t adc_T = (int32_t)(((uint32_t)raw[3] << 12) |
                              ((uint32_t)raw[4] << 4) | (raw[5] >> 4));

    int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) *
                    ((int32_t)dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - (int32_t)dig_T1) *
                       ((adc_T >> 4) - (int32_t)dig_T1)) >> 12) *
                    ((int32_t)dig_T3)) >> 14;
    int32_t t_fine = var1 + var2;

    int64_t p_var1 = ((int64_t)t_fine) - 128000;
    int64_t p_var2 = p_var1 * p_var1 * (int64_t)dig_P6;
    p_var2 += (p_var1 * (int64_t)dig_P5) << 17;
    p_var2 += ((int64_t)dig_P4) << 35;
    p_var1 = ((p_var1 * p_var1 * (int64_t)dig_P3) >> 8) +
             ((p_var1 * (int64_t)dig_P2) << 12);
    p_var1 = (((((int64_t)1) << 47) + p_var1) * (int64_t)dig_P1) >> 33;

    if (p_var1 == 0)
    {
        return NAN;
    }

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - p_var2) * 3125) / p_var1;
    p_var1 = ((int64_t)dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    p_var2 = ((int64_t)dig_P8 * p) >> 19;
    p = ((p + p_var1 + p_var2) >> 8) + (((int64_t)dig_P7) << 4);

    tekanan_pa = (float)p / 256.0f;
    tekanan_hpa = tekanan_pa / 100.0f;

    if (!isfinite(tekanan_pa) || tekanan_pa < 30000.0f || tekanan_pa > 120000.0f)
    {
        return NAN;
    }

    return tekanan_pa;
}

void BMP280_UpdateAltitude(void)
{
    static float altitude_previous = 0.0f;
    static uint8_t altitude_initialized = 0U;

    uint32_t now = HAL_GetTick();
    uint32_t elapsed_ms = now - waktu_baro;

    if (elapsed_ms < BARO_UPDATE_MS)
    {
        return;
    }

    waktu_baro = now;
    float pressure = BMP280_ReadPressure();

    if (!isfinite(pressure) || tekanan_ref_p0 <= 0.0f)
    {
        return;
    }

    float ratio = pressure / tekanan_ref_p0;
    ketinggian_m = 44330.0f * (1.0f - powf(ratio, 0.19029495f));

    if (!isfinite(ketinggian_m))
    {
        return;
    }

    if (altitude_initialized == 0U)
    {
        ketinggian_filter = ketinggian_m;
        altitude_previous = ketinggian_filter;
        altitude_initialized = 1U;
        climb_mps = 0.0f;
    }
    else
    {
        float dt_baro = elapsed_ms / 1000.0f;
        ketinggian_filter = (0.85f * ketinggian_filter) + (0.15f * ketinggian_m);
        if (dt_baro > 0.0f)
        {
            climb_mps = (ketinggian_filter - altitude_previous) / dt_baro;
        }
        altitude_previous = ketinggian_filter;
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
  /* USER CODE BEGIN 2 */
    HAL_Delay(1000U);

    /* -------- Inisialisasi MPU6050 -------- */
    if (HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x75U,
                         I2C_MEMADD_SIZE_8BIT, &identitas, 1U, HAL_MAX_DELAY) != HAL_OK)
    {
        Error_Handler();
    }

    if (identitas != 0x68U)
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
    if (BMP280_Init() != HAL_OK)
    {
        Error_Handler();
    }

    HAL_Delay(100U);

    /* Ambil rata-rata 20 sampel awal sebagai P0 (0.0 meter) */
    float sum_p0 = 0.0f;
    uint8_t valid_samples = 0U;
    for (uint8_t i = 0U; i < 20U; i++)
    {
        float p_sample = BMP280_ReadPressure();
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

    tekanan_ref_p0 = sum_p0 / (float)valid_samples;
    ketinggian_m = 0.0f;
    ketinggian_filter = 0.0f;
    climb_mps = 0.0f;

    waktu_baro = HAL_GetTick();
    waktu_sebelumnya = waktu_baro;
    waktu_hb = waktu_baro;
    waktu_led = waktu_baro;
    /* Aktifkan DLPF ~42 Hz pada register 0x1A MPU6050 untuk meredam getaran motor */
        uint8_t dlpf_cfg = 0x03U;
        if (HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x1AU,
                              I2C_MEMADD_SIZE_8BIT, &dlpf_cfg, 1U, HAL_MAX_DELAY) != HAL_OK)
        {
            Error_Handler();
        }

        /* Mulai sinyal PWM 50 Hz pada PA6, PA7, PB0 dan set posisi netral 1500 us */
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1500U);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1500U);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1500U);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        waktu_sekarang = HAL_GetTick();
        dt = (waktu_sekarang - waktu_sebelumnya) / 1000.0f;
        if (dt <= 0.0f)
        {
            dt = 0.001f;
        }

        /* Baca dan filter ketinggian barometer tiap 50 ms */
        BMP280_UpdateAltitude();

        /* Baca 14 byte data MPU6050 */
        if (HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x3BU,
                             I2C_MEMADD_SIZE_8BIT, data_mentah, 14U, HAL_MAX_DELAY) == HAL_OK)
        {
            accX_raw  = (int16_t)((data_mentah[0] << 8) | data_mentah[1]);
            accY_raw  = (int16_t)((data_mentah[2] << 8) | data_mentah[3]);
            accZ_raw  = (int16_t)((data_mentah[4] << 8) | data_mentah[5]);
            temp_raw  = (int16_t)((data_mentah[6] << 8) | data_mentah[7]);
            gyroX_raw = (int16_t)((data_mentah[8] << 8) | data_mentah[9]);
            gyroY_raw = (int16_t)((data_mentah[10] << 8) | data_mentah[11]);
            gyroZ_raw = (int16_t)((data_mentah[12] << 8) | data_mentah[13]);

            accX_g = accX_raw / 16384.0f;
            accY_g = accY_raw / 16384.0f;
            accZ_g = accZ_raw / 16384.0f;
            suhu_c = (temp_raw / 340.0f) + 36.53f;
            gyroX_dps = gyroX_raw / 131.0f;
            gyroY_dps = gyroY_raw / 131.0f;
            gyroZ_dps = gyroZ_raw / 131.0f;

            /* Complementary Filter */
            sudut_accX = atan2f(accY_g, accZ_g) * RAD_TO_DEG;
            sudut_accY = atan2f(-accX_g, accZ_g) * RAD_TO_DEG;

            compX = 0.98f * (compX + (gyroX_dps * dt)) + 0.02f * sudut_accX;
            compY = 0.98f * (compY + (gyroY_dps * dt)) + 0.02f * sudut_accY;
            compZ += gyroZ_dps * dt;

            /* Kalman Filter */
            kalmanX += gyroX_dps * dt;
            err_gyroX += Q_drift;

            kalmanY += gyroY_dps * dt;
            err_gyroY += Q_drift;

            kalmanZ += gyroZ_dps * dt;

            kalman_gainX = err_gyroX / (err_gyroX + E_acc);
            kalmanX += kalman_gainX * (sudut_accX - kalmanX);
            err_gyroX = (1.0f - kalman_gainX) * err_gyroX;

            kalman_gainY = err_gyroY / (err_gyroY + E_acc);
            kalmanY += kalman_gainY * (sudut_accY - kalmanY);
            err_gyroY = (1.0f - kalman_gainY) * err_gyroY;
            /* ---------------- KENDALI PID STABILISASI SERVO ---------------- */
                        /* 1. Outer Loop Barometer: tentukan target sudut Pitch */
                        if (enable_alt_hold != 0U)
                        {
                            float err_alt = target_alt - ketinggian_filter;
                            target_pitch = clamp_f((kp_alt * err_alt) - (kd_alt * climb_mps), -10.0f, 10.0f);
                        }
                        else
                        {
                            target_pitch = 0.0f;
                        }

                        /* 2. Inner Loop IMU: hitung koreksi mikrodetik PWM tiap sumbu */
                        float out_roll  = PID_Compute(&pid_roll,  0.0f,         kalmanX, gyroX_dps, dt);
                        float out_pitch = PID_Compute(&pid_pitch, target_pitch, kalmanY, gyroY_dps, dt);
                        float out_yaw   = clamp_f(-kd_yaw * gyroZ_dps, -350.0f, 350.0f);

                        /* 3. Konversi ke lebar pulsa 1000 - 2000 us dan kirim ke TIM3 */
                        pwm_aileron  = (uint16_t)(clamp_f(1500.0f + trim_roll  + (dir_roll  * out_roll),  1000.0f, 2000.0f) + 0.5f);
                        pwm_elevator = (uint16_t)(clamp_f(1500.0f + trim_pitch + (dir_pitch * out_pitch), 1000.0f, 2000.0f) + 0.5f);
                        pwm_rudder   = (uint16_t)(clamp_f(1500.0f + trim_yaw   + (dir_yaw   * out_yaw),   1000.0f, 2000.0f) + 0.5f);

                        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_aileron);  /* PA6 */
                        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_elevator); /* PA7 */
                        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm_rudder);   /* PB0 */

            /* Paket MAVLink */
            uint16_t total_len = 0U;

            if ((waktu_sekarang - waktu_hb) >= 1000U)
            {
                waktu_hb = waktu_sekarang;
                mavlink_msg_heartbeat_pack(
                    SYS_ID, COMP_ID_KALMAN, &msg_hb,
                    MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC,
                    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0U, MAV_STATE_ACTIVE);
                total_len += (uint16_t)mavlink_msg_to_send_buffer(&tx_buf[total_len], &msg_hb);
            }

            float gx_rad = gyroX_dps * DEG_TO_RAD;
            float gy_rad = gyroY_dps * DEG_TO_RAD;
            float gz_rad = gyroZ_dps * DEG_TO_RAD;

            mavlink_msg_attitude_pack(
                SYS_ID, COMP_ID_KALMAN, &msg_kalman, waktu_sekarang,
                kalmanX * DEG_TO_RAD, kalmanY * DEG_TO_RAD, kalmanZ * DEG_TO_RAD,
                gx_rad, gy_rad, gz_rad);
            total_len += (uint16_t)mavlink_msg_to_send_buffer(&tx_buf[total_len], &msg_kalman);

            mavlink_msg_attitude_pack(
                SYS_ID, COMP_ID_COMP, &msg_comp, waktu_sekarang,
                compX * DEG_TO_RAD, compY * DEG_TO_RAD, compZ * DEG_TO_RAD,
                gx_rad, gy_rad, gz_rad);
            total_len += (uint16_t)mavlink_msg_to_send_buffer(&tx_buf[total_len], &msg_comp);

            float heading_deg_f = fmodf(kalmanZ, 360.0f);
            if (heading_deg_f < 0.0f) heading_deg_f += 360.0f;
            int16_t heading_deg = (int16_t)heading_deg_f;

            /* Data barometer dikirim melalui field alt (meter) & climb (m/s) pada VFR_HUD */
            mavlink_msg_vfr_hud_pack(
                SYS_ID, COMP_ID_KALMAN, &msg_vfr,
                0.0f, 0.0f, heading_deg, 0U,
                ketinggian_filter, climb_mps);
            total_len += (uint16_t)mavlink_msg_to_send_buffer(&tx_buf[total_len], &msg_vfr);

            if (total_len > 0U)
            {
                (void)CDC_Transmit_FS(tx_buf, total_len);
            }

            /* Kedipkan LED PC13 setiap 250 ms sebagai tanda sensor berjalan normal */
            if ((waktu_sekarang - waktu_led) >= 250U)
            {
                waktu_led = waktu_sekarang;
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            }
        }

        HAL_Delay(10U);
        waktu_sebelumnya = waktu_sekarang;
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

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
  hi2c1.Init.ClockSpeed = 100000;
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
  htim3.Init.Prescaler = 0;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  huart1.Init.BaudRate = 115200;
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
