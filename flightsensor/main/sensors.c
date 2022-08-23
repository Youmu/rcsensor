#include "sensors.h"
#include <math.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "mpl3115.h"
#include "sht40.h"
static i2c_bus_handle_t i2c_bus = NULL;
static icm42688_handle_t icm42688_dev = NULL;
static mpl3115_handle_t mpl3115_dev = NULL;
static sht40_handle_t sht40_dev = NULL;

void dev_init()
{
    i2c_bus_init();
    icm42688_init();
	//mpl3115_init();
	sht40_dev = iot_sht40_create(i2c_bus);
    //uart_init();
}

#define I2C_MASTER_SCL_IO           26          /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO           25          /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_1   /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE   0           /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0           /*!< I2C master do not need buffer */
#define I2C_MASTER_FREQ_HZ          800000      /*!< I2C master clock frequency */


void i2c_bus_init(){
	esp_rom_printf("zzzzz");
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
	conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;
    i2c_bus = iot_i2c_bus_create(I2C_MASTER_NUM, &conf);
}

void icm42688_init()
{
	icm42688_dev = iot_icm42688_create(i2c_bus, ICM42688_I2C_ADDRESS);
	iot_icm42688_pwr_mgmt(icm42688_dev, PWR_GYRO_LN | PWR_ACCE_LN);
    iot_icm42688_set_acce_fs(icm42688_dev, ACCE_FS_8G);
    iot_icm42688_set_gyro_fs(icm42688_dev, GYRO_FS_250DPS);
}

void mpl3115_init()
{
	esp_rom_printf("MPL3115 Init");
	mpl3115_dev = iot_mpl3115_create(i2c_bus, 0x60);
	esp_rom_printf("MPL3115 Inited");
	iot_mpl3115_start(mpl3115_dev);
	esp_rom_printf("MPL3115 started");
}

void sht40_init()
{
	sht40_dev = iot_sht40_create(i2c_bus);
}

void showAcce()
{
	icm42688_raw_acce_value_t val;
    iot_icm42688_get_raw_acce(icm42688_dev, &val);
	float g = sqrt( (float)val.raw_acce_x * val.raw_acce_x +
	                (float)val.raw_acce_y * val.raw_acce_y +
					(float)val.raw_acce_z * val.raw_acce_z);
	esp_rom_printf("Accel: %d, %d, %d --- ", val.raw_acce_x, val.raw_acce_y, val.raw_acce_z);
	esp_rom_printf("val = %d \n", (int)(g * 1000 / 4096));
	int16_t temp;
	iot_icm42688_get_temp(icm42688_dev, &temp);
	esp_rom_printf("temp = %d \n", temp);
	uint32_t p = iot_mp3115_read(mpl3115_dev);
	printf("P = %d \n", p);
	double t2 = iot_mp3115_readT(mpl3115_dev);
	printf("t1 = %.2f \n", t2);
	double shttemp;
	read_sensor(sht40_dev, &shttemp);
	printf("t2 = %.2f \n", shttemp);
	fflush(stdout);
}