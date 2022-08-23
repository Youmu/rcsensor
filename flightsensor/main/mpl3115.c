#include "mpl3115.h"

#define WRITE_BIT  I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT   I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL    0x0         /*!< I2C ack value */
#define NACK_VAL   0x1         /*!< I2C nack value */
#define Pa_OS128_STBY 0x70
#define Pa_OS128_ACT  0x71
typedef struct {
    i2c_bus_handle_t bus;
    uint16_t dev_addr;
} mpl3115_dev_t;

mpl3115_handle_t iot_mpl3115_create(i2c_bus_handle_t bus, uint16_t dev_addr)
{
	mpl3115_dev_t* sensor = (mpl3115_dev_t*) calloc(1, sizeof(mpl3115_dev_t));
    sensor->bus = bus;
    sensor->dev_addr = dev_addr;
    return (mpl3115_handle_t*) sensor;
}

void write_byte(mpl3115_dev_t *dev, uint8_t reg_addr, uint8_t data){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->dev_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = iot_i2c_bus_cmd_begin(dev->bus, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL) {
        printf("ERROR");
    }
}


esp_err_t read_byte(mpl3115_handle_t sensor, uint8_t reg, uint8_t *data)
{
	mpl3115_dev_t* sens = (mpl3115_dev_t*) sensor;
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (sens->dev_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    //i2c_master_stop(cmd);
    ret = iot_i2c_bus_cmd_begin(sens->bus, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL) {
        esp_rom_printf("MPL3115 Read error");
        return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (sens->dev_addr << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data, NACK_VAL);
    i2c_master_stop(cmd);
    ret = iot_i2c_bus_cmd_begin(sens->bus, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_FAIL) {
        esp_rom_printf("MPL3115 Read error2");
        return ret;
    }
    return ret;
}

void iot_mpl3115_start(mpl3115_handle_t sensor){
    mpl3115_dev_t* sens = (mpl3115_dev_t*) sensor;
    write_byte(sens, 0x26, 0x4 );
    write_byte(sens, 0x26, Pa_OS128_STBY );
    write_byte(sens, 0x13, 0x07 );
    write_byte(sens, 0x26, Pa_OS128_ACT );

}

uint32_t iot_mp3115_read(mpl3115_handle_t sensor){
    uint32_t data = 0;
    uint8_t *buf = (unsigned char*)&data;
    read_byte(sensor, 0x0, buf);
    if(!(buf[0] & 0x08) ) return 0xffffffff;
    read_byte(sensor, 0x1, (unsigned char*)buf + 3);
    read_byte(sensor, 0x2, (unsigned char*)buf + 2);
    read_byte(sensor, 0x3, (unsigned char*)buf + 1);
    *buf = 0;

    //read_byte(sensor, 0x4, buf + 4);

    //read_byte(sensor, 0x5, buf + 5);

    return data >> 13;

}


double iot_mp3115_readT(mpl3115_handle_t sensor){
    uint8_t data = 0;
    uint8_t data1 = 0;
    read_byte(sensor, 0x4, &data);
    read_byte(sensor, 0x5, &data1);

    //read_byte(sensor, 0x4, buf + 4);

    //read_byte(sensor, 0x5, buf + 5);
    int16_t t = (int16_t)((data <<8) | data1);
    return t / 256.0;

}


float iot_mp3115_readf(mpl3115_handle_t sensor){
    int16_t meter = 0;
    float val = 0;
    uint8_t buf[3] = {0x00,0x00,0x00};
    read_byte(sensor, 0x0, buf);
    if(!(buf[0] & 0x08) ) return 0xffffffff;
    read_byte(sensor, 0x1, buf + 2);
    read_byte(sensor, 0x2, buf + 1);
    read_byte(sensor, 0x3, buf);

    //read_byte(sensor, 0x4, buf + 4);

    //read_byte(sensor, 0x5, buf + 5);
    meter = buf[2];
    meter <<= 8;
    meter += buf[1];
    val = 1.0/16 * (buf[0]>>4) ;
    if(meter < 0)val = -val;
    val += meter;
    return val;

}