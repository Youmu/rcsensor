
set(COMPONENT_INCLUDE_DIRS
.
)

idf_component_register(
    SRCS "sht40.c" "mpl3115.c" "main.c" "i2c_bus.c" "icm42688.c" "sensors.c" 
    INCLUDE_DIRS "."
    REQUIRES freertos nvs_flash coreMQTT azure-sdk-for-c azure-iot-middleware-freertos
)
