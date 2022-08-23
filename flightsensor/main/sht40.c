#include "sht40.h"

#define WRITE_BIT  I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT   I2C_MASTER_READ  /*!< I2C master read */
#define SHT40_ADDR   0x44     /*!< I2C master will check ack from slave*/
#define CMD_MEASURE_HIGHPRE 0xFD

#define ACK_VAL    0x0         /*!< I2C ack value */
#define NACK_VAL   0x1         /*!< I2C nack value */
#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/

typedef struct {
    i2c_bus_handle_t bus;
    uint16_t dev_addr;
} sht40_dev_t;

sht40_handle_t iot_sht40_create(i2c_bus_handle_t bus)
{
	sht40_dev_t* sensor = (sht40_dev_t*) calloc(1, sizeof(sht40_dev_t));
    sensor->bus = bus;
    sensor->dev_addr = SHT40_ADDR;
    return (sht40_handle_t*) sensor;
}

int read_sensor(sht40_handle_t dev, double *temp){
    sht40_dev_t *sensor = (sht40_dev_t*)dev;
    uint8_t data[6];
    int ret = 0;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (sensor->dev_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, CMD_MEASURE_HIGHPRE, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = iot_i2c_bus_cmd_begin(sensor->bus, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL) {
        esp_rom_printf("MPL3115 Read error");
        return ret;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (sensor->dev_addr << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data, ACK_VAL);
    i2c_master_read_byte(cmd, data+1, ACK_VAL);
    i2c_master_read_byte(cmd, data+2, ACK_VAL);
    i2c_master_read_byte(cmd, data+3, ACK_VAL);
    i2c_master_read_byte(cmd, data+4, ACK_VAL);
    i2c_master_read_byte(cmd, data+5, NACK_VAL);
    i2c_master_stop(cmd);
    ret = iot_i2c_bus_cmd_begin(sensor->bus, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_FAIL) {
        esp_rom_printf("SHT40 Read error2");
        return ret;
    }

    uint32_t t_ticks = data[0] * 256 + data[1];
    *temp = -45.0 + 175. * t_ticks/65535.0;

    return ret;
}

