
#include "driver/i2c.h"
#include "i2c_bus.h"


typedef void* mpl3115_handle_t;

mpl3115_handle_t iot_mpl3115_create(i2c_bus_handle_t bus, uint16_t dev_addr);
void iot_mpl3115_start(mpl3115_handle_t sensor);
uint32_t iot_mp3115_read(mpl3115_handle_t sensor);
float iot_mp3115_readf(mpl3115_handle_t sensor);