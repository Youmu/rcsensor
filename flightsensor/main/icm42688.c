// ICM 42688
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "esp_system.h"
#include "driver/i2c.h"
#include "i2c_bus.h"
#include "icm42688.h"

#define WRITE_BIT  I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT   I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL    0x0         /*!< I2C ack value */
#define NACK_VAL   0x1         /*!< I2C nack value */

#define ALPHA 0.99             /*!< Weight for gyroscope */
#define RAD_TO_DEG 57.27272727 /*!< Radians to degrees */

typedef struct {
    i2c_bus_handle_t bus;
    uint16_t dev_addr;
    uint32_t counter;
    float dt;  /*!< delay time between twice measurement, dt should be small (ms level) */
    struct timeval *timer;
} icm42688_dev_t;

esp_err_t iot_icm42688_write_byte(icm42688_handle_t sensor, uint8_t reg_addr, uint8_t data)
{
    icm42688_dev_t* sens = (icm42688_dev_t*) sensor;
    esp_err_t  ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (sens->dev_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    ret = iot_i2c_bus_cmd_begin(sens->bus, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL) {
        return ret;
    }
    return ESP_OK;
}

esp_err_t iot_icm42688_write(icm42688_handle_t sensor, uint8_t reg_start_addr, uint8_t reg_num, uint8_t *data_buf)
{
    uint32_t i = 0;
    if (data_buf != NULL) {
        for(i=0; i<reg_num; i++) {
            iot_icm42688_write_byte(sensor, reg_start_addr+i, data_buf[i]);
        }
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t iot_icm42688_read_byte(icm42688_handle_t sensor, uint8_t reg, uint8_t *data)
{
	icm42688_dev_t* sens = (icm42688_dev_t*) sensor;
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (sens->dev_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = iot_i2c_bus_cmd_begin(sens->bus, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL) {
        return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (sens->dev_addr << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data, NACK_VAL);
    i2c_master_stop(cmd);
    ret = iot_i2c_bus_cmd_begin(sens->bus, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t iot_icm42688_read(icm42688_handle_t sensor, uint8_t reg_start_addr, uint8_t reg_num, uint8_t *data_buf)
{
    uint32_t i = 0;
    uint8_t data_t = 0;
    if (data_buf != NULL) {
        for(i=0; i<reg_num; i++){
            iot_icm42688_read_byte(sensor, reg_start_addr+i, &data_t);
            data_buf[i] = data_t;
        }
        return ESP_OK;
    } 
    return ESP_FAIL;  
}

icm42688_handle_t iot_icm42688_create(i2c_bus_handle_t bus, uint16_t dev_addr)
{
	icm42688_dev_t* sensor = (icm42688_dev_t*) calloc(1, sizeof(icm42688_dev_t));
    sensor->bus = bus;
    sensor->dev_addr = dev_addr;
    sensor->counter = 0;
    sensor->dt = 0;
    sensor->timer = (struct timeval *) calloc(1, sizeof(struct timeval));
    return (icm42688_handle_t) sensor;
}

esp_err_t iot_icm42688_delete(icm42688_handle_t sensor, bool del_bus)
{
	icm42688_dev_t* sens = (icm42688_dev_t*) sensor;
    if(del_bus) {
        iot_i2c_bus_delete(sens->bus);
        sens->bus = NULL;
    }
    free(sens);
    return ESP_OK;
}

esp_err_t iot_icm42688_get_deviceid(icm42688_handle_t sensor, uint8_t* deviceid)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = iot_icm42688_read_byte(sensor, ICM42688_WHO_AM_I, &tmp);
    *deviceid = tmp;
    return ret;
}

esp_err_t iot_icm42688_pwr_mgmt(icm42688_handle_t sensor, icm42688_pwr_t val)
{
    esp_err_t ret;
    ret = iot_icm42688_write_byte(sensor, ICM42688_PWR_MGMT_0, val);
    return ret;
}
/*
esp_err_t iot_icm42688_sleep(mpu6050_handle_t sensor)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = iot_mpu6050_read_byte(sensor, MPU6050_PWR_MGMT_1, &tmp);
    if (ret == ESP_FAIL) {
        return ret;
    }
    tmp |= BIT6;
    ret = iot_mpu6050_write_byte(sensor, MPU6050_PWR_MGMT_1, tmp);
    return ret;
}
*/

esp_err_t iot_icm42688_set_acce_fs(icm42688_handle_t sensor, icm42688_acce_fs_t acce_fs)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = iot_icm42688_read_byte(sensor, ICM42688_ACCEL_CONFIG0, &tmp);
    if (ret == ESP_FAIL) {
        return ret;
    }
    tmp &= 0x1F; // 0001 1111b
    tmp |= (acce_fs << 5);
    ret = iot_icm42688_write_byte(sensor, ICM42688_ACCEL_CONFIG0, tmp);
    return ret;
}

esp_err_t iot_icm42688_set_gyro_fs(icm42688_handle_t sensor, icm42688_gyro_fs_t gyro_fs)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = iot_icm42688_read_byte(sensor, ICM42688_GYRO_CONFIG0, &tmp);
    if (ret == ESP_FAIL) {
        return ret;
    }
    tmp &= 0x1F; // 0001 1111b
    tmp |= (gyro_fs << 5);
    ret = iot_icm42688_write_byte(sensor, ICM42688_GYRO_CONFIG0, tmp);
    return ret;
}

esp_err_t iot_icm42688_get_acce_fs(icm42688_handle_t sensor, icm42688_acce_fs_t *acce_fs)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = iot_icm42688_read_byte(sensor, ICM42688_ACCEL_CONFIG0, &tmp);
    tmp = (tmp >> 5) & 0x07;
    *acce_fs = tmp;
    return ret;
}

esp_err_t iot_icm42688_get_gyro_fs(icm42688_handle_t sensor, icm42688_gyro_fs_t *gyro_fs)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = iot_icm42688_read_byte(sensor, ICM42688_GYRO_CONFIG0, &tmp);
    tmp = (tmp >> 5) & 0x07;
    *gyro_fs = tmp;
    return ret;
}

esp_err_t iot_icm42688_get_raw_acce(icm42688_handle_t sensor, icm42688_raw_acce_value_t *raw_acce_value)
{
    uint8_t data_rd[6] = {0};
    icm42688_dev_t* sens = (icm42688_dev_t*) sensor;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( ICM42688_I2C_ADDRESS << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ICM42688_ACCEL_XOUT_1, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( ICM42688_I2C_ADDRESS << 1 ) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data_rd, 5, ACK_VAL);
    i2c_master_read(cmd, data_rd + 5, 1, NACK_VAL);
    i2c_master_stop(cmd);
    int ret = iot_i2c_bus_cmd_begin(sens->bus, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    raw_acce_value->raw_acce_x = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
    raw_acce_value->raw_acce_y = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
    raw_acce_value->raw_acce_z = (int16_t)((data_rd[4] << 8) + (data_rd[5]));
    return ret;
}

esp_err_t iot_icm42688_get_raw_gyro(icm42688_handle_t sensor, icm42688_raw_gyro_value_t *raw_gyro_value)
{
    uint8_t data_rd[6] = {0};
    icm42688_dev_t* sens = (icm42688_dev_t*) sensor;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( ICM42688_I2C_ADDRESS << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ICM42688_GYRO_XOUT_1, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( ICM42688_I2C_ADDRESS << 1 ) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data_rd, 5, ACK_VAL);
    i2c_master_read(cmd, data_rd + 5, 1, NACK_VAL);
    i2c_master_stop(cmd);
    int ret = iot_i2c_bus_cmd_begin(sens->bus, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    raw_gyro_value->raw_gyro_x = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
    raw_gyro_value->raw_gyro_y = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
    raw_gyro_value->raw_gyro_z = (int16_t)((data_rd[4] << 8) + (data_rd[5]));
    return ret;
}

//esp_err_t iot_icm42688_get_fifo_data(icm42688_handle_t sensor, size_t len, uint8_t *buffer)
//{
//    mpu6050_dev_t* sens = (mpu6050_dev_t*) sensor;
//    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//    i2c_master_start(cmd);
//    i2c_master_write_byte(cmd, ( MPU6050_I2C_ADDRESS << 1 ) | WRITE_BIT, ACK_CHECK_EN);
//    i2c_master_write_byte(cmd, MPU6050_FIFO_R_W, ACK_CHECK_EN);
//    i2c_master_start(cmd);
//    i2c_master_write_byte(cmd, ( MPU6050_I2C_ADDRESS << 1 ) | READ_BIT, ACK_CHECK_EN);
//    if(len > 1){
//        i2c_master_read(cmd, buffer, len - 1, ACK_VAL);
//    }
//    i2c_master_read(cmd, buffer + len - 1, 1, NACK_VAL);
//    i2c_master_stop(cmd);
//    esp_err_t ret = iot_i2c_bus_cmd_begin(sens->bus, cmd, 1000 / portTICK_RATE_MS);
//    i2c_cmd_link_delete(cmd);
//    return ret;
//}
//
//esp_err_t iot_icm42688_get_fifo_len(icm42688_handle_t sensor, uint16_t *len)
//{
//    esp_err_t ret;
//    uint8_t datard[2];
//    ret = iot_mpu6050_read(sensor, MPU6050_FIFO_COUNTH, 2,  datard);
//    *len = ((int16_t)datard[0] << 8) + datard[1];
//    return ret;
//}
//
//
//esp_err_t iot_icm42688_set_dlpf(icm42688_handle_t sensor, uint8_t flags)
//{
//    esp_err_t ret;
//    ret = iot_mpu6050_write_byte(sensor, MPU6050_CONFIG, flags);
//    return ret;
//}
//
//esp_err_t iot_icm42688_set_smplrt_div(icm42688_handle_t sensor, uint8_t val)
//{
//    esp_err_t ret;
//    ret = iot_mpu6050_write_byte(sensor, MPU6050_SMPLRT_DIV, val);
//    return ret;
//}
//

//esp_err_t iot_icm42688_set_fifo_en(icm42688_handle_t sensor, uint8_t flags)
//{
//    esp_err_t ret;
//    ret = iot_mpu6050_write_byte(sensor, MPU6050_FIFO_EN, flags);
//    ret = iot_mpu6050_write_byte(sensor, MPU6050_USER_CTRL, 0x44);
//    return ret;
//}
//
//
