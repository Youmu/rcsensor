
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "zigbee.h"

const uart_port_t z_uart_num = UART_NUM_1;
const int z_uart_buffer_size = 512;

void zigbee_init()
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(z_uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(z_uart_num, GPIO_NUM_23, GPIO_NUM_21, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // Setup UART buffered IO with event queue
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(z_uart_num, z_uart_buffer_size, z_uart_buffer_size, 10, NULL, 0));
}

#define escape(x,buf,i) if((x)==0xff){buf[i++]=0xFE;buf[i++]=0xFD;}\
                        else if((x)==0xfe){buf[i++]=0xFE;buf[i++]=0xFC;}\
                        else {buf[i++] = x;}

void zigbee_send(uint16_t addr, uint8_t from_port, uint8_t to_port, uint8_t len, uint8_t *data)
{
    int escape_count = 0;
    uint8_t addr_l = addr &0xff;
    uint8_t addr_h = (addr >> 8) & 0xff;
    for (int i = 0; i < len; i++)
    {
        if (data[i] == 0xff || data[i] == 0xfe)
            escape_count++;
    }
    int datalen_max = len + escape_count + 9; // Header (2) + Port_s(1) + Port_d(1) + addr(4) + data + Fin(1)
    uint8_t *pkg = malloc(datalen_max);
    pkg[0] = 0xFE;
    pkg[1] = 4 + len;
    pkg[2] = from_port;
    pkg[3] = to_port;
    int data_index = 4;
    escape(addr_h, pkg, data_index);
    escape(addr_l, pkg, data_index);
    for(int i = 0;i<len;i++){
        escape((data[i]), pkg, data_index); // Calculate escape
    }
    // Send 

	uart_write_bytes(z_uart_num, (const char*)pkg, data_index);
}
