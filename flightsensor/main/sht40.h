
#include "driver/i2c.h"
#include "i2c_bus.h"


typedef void* sht40_handle_t;

sht40_handle_t iot_sht40_create(i2c_bus_handle_t bus);

int read_sensor(sht40_handle_t dev, double *temp);