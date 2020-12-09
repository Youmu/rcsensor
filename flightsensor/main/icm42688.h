/*
 * icm42688.h
 *
 *  Created on: Nov 27, 2020
 *      Author: tengda
 */

#ifndef MAIN_ICM42688_H_
#define MAIN_ICM42688_H_



#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c.h"
#include "i2c_bus.h"

#define ICM42688_I2C_ADDRESS         0x68    /*!< slave address for ICM42688 sensor */

/* ICM42688 register */
//#define ICM42688_SELF_TEST_X         0x0D
//#define ICM42688_SELF_TEST_Y         0x0E
//#define ICM42688_SELF_TEST_Z         0x0F
//#define ICM42688_SELF_TEST_A         0x10
//#define ICM42688_SMPLRT_DIV          0x19
//#define ICM42688_CONFIG              0x1A
#define ICM42688_GYRO_CONFIG0         0x4F
#define ICM42688_ACCEL_CONFIG0        0x50
//#define ICM42688_FIFO_EN             0x23
//#define ICM42688_I2C_MST_CTRL        0x24
//#define ICM42688_I2C_SLV0_ADDR       0x25
//#define ICM42688_I2C_SLV0_REG        0x26
//#define ICM42688_I2C_SLV0_CTRL       0x27
//#define ICM42688_I2C_SLV1_ADDR       0x28
//#define ICM42688_I2C_SLV1_REG        0x29
//#define ICM42688_I2C_SLV1_CTRL       0x2A
//#define ICM42688_I2C_SLV2_ADDR       0x2B
//#define ICM42688_I2C_SLV2_REG        0x2C
//#define ICM42688_I2C_SLV2_CTRL       0x2D
//#define ICM42688_I2C_SLV3_ADDR       0x2E
//#define ICM42688_I2C_SLV3_REG        0x2F
//#define ICM42688_I2C_SLV3_CTRL       0x30
//#define ICM42688_I2C_SLV4_ADDR       0x31
//#define ICM42688_I2C_SLV4_REG        0x32
//#define ICM42688_I2C_SLV4_DO         0x33
//#define ICM42688_I2C_SLV4_CTRL       0x34
//#define ICM42688_I2C_SLV4_DI         0x35
//#define ICM42688_I2C_MST_STATUS      0x36
//#define ICM42688_INT_PIN_CFG         0x37
//#define ICM42688_INT_ENABLE          0x38
//#define ICM42688_DMP_INT_STATUS      0x39
//#define ICM42688_INT_STATUS          0x3A
#define ICM42688_ACCEL_XOUT_1        0x1F
#define ICM42688_ACCEL_XOUT_0        0x20
#define ICM42688_ACCEL_YOUT_1        0x21
#define ICM42688_ACCEL_YOUT_0        0x22
#define ICM42688_ACCEL_ZOUT_1        0x23
#define ICM42688_ACCEL_ZOUT_0        0x24
//#define ICM42688_TEMP_OUT_H          0x41
//#define ICM42688_TEMP_OUT_L          0x42
#define ICM42688_GYRO_XOUT_1         0x25
#define ICM42688_GYRO_XOUT_0         0x26
#define ICM42688_GYRO_YOUT_1         0x27
#define ICM42688_GYRO_YOUT_0         0x28
#define ICM42688_GYRO_ZOUT_1         0x29
#define ICM42688_GYRO_ZOUT_0         0x2A
//#define ICM42688_EXT_SENS_DATA_00    0x49
//#define ICM42688_EXT_SENS_DATA_01    0x4A
//#define ICM42688_EXT_SENS_DATA_02    0x4B
//#define ICM42688_EXT_SENS_DATA_03    0x4C
//#define ICM42688_EXT_SENS_DATA_04    0x4D
//#define ICM42688_EXT_SENS_DATA_05    0x4E
//#define ICM42688_EXT_SENS_DATA_06    0x4F
//#define ICM42688_EXT_SENS_DATA_07    0x50
//#define ICM42688_EXT_SENS_DATA_08    0x51
//#define ICM42688_EXT_SENS_DATA_09    0x52
//#define ICM42688_EXT_SENS_DATA_10    0x53
//#define ICM42688_EXT_SENS_DATA_11    0x54
//#define ICM42688_EXT_SENS_DATA_12    0x55
//#define ICM42688_EXT_SENS_DATA_13    0x56
//#define ICM42688_EXT_SENS_DATA_14    0x57
//#define ICM42688_EXT_SENS_DATA_15    0x58
//#define ICM42688_EXT_SENS_DATA_16    0x59
//#define ICM42688_EXT_SENS_DATA_17    0x5A
//#define ICM42688_EXT_SENS_DATA_18    0x5B
//#define ICM42688_EXT_SENS_DATA_19    0x5C
//#define ICM42688_EXT_SENS_DATA_20    0x5D
//#define ICM42688_EXT_SENS_DATA_21    0x5E
//#define ICM42688_EXT_SENS_DATA_22    0x5F
//#define ICM42688_EXT_SENS_DATA_23    0x60
//#define ICM42688_I2C_SLV0_DO         0x63
//#define ICM42688_I2C_SLV1_DO         0x64
//#define ICM42688_I2C_SLV2_DO         0x65
//#define ICM42688_I2C_SLV3_DO         0x66
//#define ICM42688_I2C_MST_DELAY_CTRL  0x67
//#define ICM42688_SIGNAL_PATH_RESET   0x68
//#define ICM42688_USER_CTRL           0x6A
#define ICM42688_PWR_MGMT_0          0x4E
//#define ICM42688_PWR_MGMT_2          0x6C
//#define ICM42688_FIFO_COUNTH         0x72
//#define ICM42688_FIFO_COUNTL         0x73
//#define ICM42688_FIFO_R_W            0x74
#define ICM42688_WHO_AM_I            0x75

typedef enum {
    ACCE_FS_16G  = 0,     /*!< Accelerometer full scale range is +/- 16g */
    ACCE_FS_8G   = 1,     /*!< Accelerometer full scale range is +/- 8g */
    ACCE_FS_4G   = 2,     /*!< Accelerometer full scale range is +/- 4g */
    ACCE_FS_2G   = 3,     /*!< Accelerometer full scale range is +/- 2g */
} icm42688_acce_fs_t;

typedef enum {
    GYRO_FS_2000DPS  = 0,     /*!< Gyroscope full scale range is +/- 2000 degree per sencond */
    GYRO_FS_1000DPS  = 1,     /*!... */
    GYRO_FS_500DPS = 2,
    GYRO_FS_250DPS = 3,
    GYRO_FS_125DPS = 4,
    GYRO_FS_62_5DPS = 5,
    GYRO_FS_31_25DPS = 6,
    GYRO_FS_15_625DPS = 7,
} icm42688_gyro_fs_t;

typedef enum {
	PWR_TEMP_ON = 0x20,
	PWR_GYRO_STB = 0x04,
	PWR_GYRO_LN  = 0x0C,
	PWR_ACCE_LP  = 0x01,
	PWR_ACCE_LN  = 0x03
} icm42688_pwr_t;

//
//typedef enum{
//	FIFO_EN_XG = 0x40,
//	FIFO_EN_YG = 0x20,
//	FIFO_EN_ZG = 0x10
//
//} icm42688_fifo_en_t;
//
//typedef enum{
//	DLPF_DISABLE = 0,
//	DLPF_1 = 1,
//	DLPF_2 = 2,
//	DLPF_3 = 3,
//	DLPF_4 = 4
//} icm42688_dlpf_config;
//
typedef struct {
    int16_t raw_acce_x;
    int16_t raw_acce_y;
    int16_t raw_acce_z;
} icm42688_raw_acce_value_t;

typedef struct {
    int16_t raw_gyro_x;
    int16_t raw_gyro_y;
    int16_t raw_gyro_z;
} icm42688_raw_gyro_value_t;
//
//typedef struct {
//    float acce_x;
//    float acce_y;
//    float acce_z;
//} ICM42688_acce_value_t;
//
//typedef struct {
//    float gyro_x;
//    float gyro_y;
//    float gyro_z;
//} ICM42688_gyro_value_t;
//
//typedef struct {
//    float roll;
//    float pitch;
//} complimentary_angle_t;

typedef void* icm42688_handle_t;

/**
 * @brief Create and init sensor object and return a sensor handle
 *
 * @param bus I2C bus object handle
 * @param dev_addr I2C device address of sensor
 *
 * @return
 *     - NULL Fail
 *     - Others Success
 */
icm42688_handle_t iot_icm42688_create(i2c_bus_handle_t bus, uint16_t dev_addr);

/**
 * @brief Delete and release a sensor object
 *
 * @param sensor object handle of ICM42688
 * @param del_bus Whether to delete the I2C bus
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_icm42688_delete( i2c_bus_handle_t sensor, bool del_bus);

/**
 * @brief Get device identification of ICM42688
 *
 * @param sensor object handle of ICM42688
 * @param deviceid a pointer of device ID
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_icm42688_get_deviceid(icm42688_handle_t sensor, uint8_t* deviceid);

/**
 * @brief Wake up ICM42688
 *
 * @param sensor object handle of ICM42688
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_icm42688_pwr_mgmt(icm42688_handle_t sensor, icm42688_pwr_t val);

/**
 * @brief Enter sleep mode
 *
 * @param sensor object handle of ICM42688
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
//esp_err_t iot_ICM42688_sleep(ICM42688_handle_t sensor);

/**
 * @brief Set accelerometer full scale range
 *
 * @param sensor object handle of ICM42688
 * @param acce_fs accelerometer full scale range
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_icm42688_set_acce_fs(icm42688_handle_t sensor, icm42688_acce_fs_t acce_fs);

/**
 * @brief Set gyroscope full scale range
 *
 * @param sensor object handle of ICM42688
 * @param gyro_fs gyroscope full scale range
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_icm42688_set_gyro_fs(icm42688_handle_t sensor, icm42688_gyro_fs_t gyro_fs);

/**
 * @brief Get accelerometer full scale range
 *
 * @param sensor object handle of ICM42688
 * @param acce_fs accelerometer full scale range
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_icm42688_get_acce_fs(icm42688_handle_t sensor, icm42688_acce_fs_t *acce_fs);

/**
 * @brief Get gyroscope full scale range
 *
 * @param sensor object handle of ICM42688
 * @param gyro_fs gyroscope full scale range
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_icm42688_get_gyro_fs(icm42688_handle_t sensor, icm42688_gyro_fs_t *gyro_fs);

/**
 * @brief Get accelerometer sensitivity
 *
 * @param sensor object handle of ICM42688
 * @param acce_sensitivity accelerometer sensitivity
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
//esp_err_t iot_ICM42688_get_acce_sensitivity(ICM42688_handle_t sensor, float *acce_sensitivity);

/**
 * @brief Get gyroscope sensitivity
 *
 * @param sensor object handle of ICM42688
 * @param gyro_sensitivity gyroscope sensitivity
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
//esp_err_t iot_ICM42688_get_gyro_sensitivity(icm42688_handle_t sensor, float *gyro_sensitivity);

/**
 * @brief Read raw accelerometer measurements
 *
 * @param sensor object handle of ICM42688
 * @param acce_value raw accelerometer measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_icm42688_get_raw_acce(icm42688_handle_t sensor, icm42688_raw_acce_value_t *raw_acce_value);

/**
 * @brief Read raw gyroscope measurements
 *
 * @param sensor object handle of ICM42688
 * @param gyro_value raw gyroscope measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_icm42688_get_raw_gyro(icm42688_handle_t sensor, icm42688_raw_gyro_value_t *raw_gyro_value);

/**
 * @brief Read accelerometer measurements
 *
 * @param sensor object handle of ICM42688
 * @param acce_value accelerometer measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
//esp_err_t iot_ICM42688_get_acce(icm42688_handle_t sensor, icm42688_acce_value_t *acce_value);

/**
 * @brief Read gyro values
 *
 * @param sensor object handle of ICM42688
 * @param gyro_value gyroscope measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
//esp_err_t iot_ICM42688_get_gyro(icm42688_handle_t sensor, icm42688_gyro_value_t *gyro_value);

/**
 * @brief use complimentory filter to caculate roll and pitch
 *
 * @param acce_value accelerometer measurements
 * @param gyro_value gyroscope measurements
 * @param complimentary_angle complimentary angle
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
//esp_err_t iot_ICM42688_complimentory_filter(ICM42688_handle_t sensor, ICM42688_acce_value_t *acce_value,
//                        ICM42688_gyro_value_t *gyro_value, complimentary_angle_t *complimentary_angle);


//esp_err_t iot_ICM42688_get_fifo_data(ICM42688_handle_t sensor, size_t len, uint8_t *buffer);
//
//
//esp_err_t iot_ICM42688_get_fifo_len(ICM42688_handle_t sensor, uint16_t *len);
//
//esp_err_t iot_ICM42688_set_dlpf(ICM42688_handle_t sensor, uint8_t flags);
//
//esp_err_t iot_ICM42688_set_smplrt_div(ICM42688_handle_t sensor, uint8_t val);
//
//esp_err_t iot_ICM42688_set_fifo_en(ICM42688_handle_t sensor, uint8_t flags);

#ifdef __cplusplus
}
#endif



#endif /* MAIN_ICM42688_H_ */
