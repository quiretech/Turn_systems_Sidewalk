#ifndef U_UART_TURN_H
#define U_UART_TURN_H

#include <zephyr/kernel.h>

#define RECEIVE_BUFF_SIZE   256
#define TRANSMIT_BUFF_SIZE  256
#define RECEIVE_TIMEOUT     2000     // U_SEC
#define PACKAGE_TIMEOUT     5       // M_SEC


typedef int err_return;

typedef enum
{
    Uart_0 = 0,
    Uart_1
}Uart_num;

err_return u_uart_init();
err_return u_uart_send(const struct device *dev, uint8_t *tx_buffer, size_t buff_len);
void u_uart_send_data(uint8_t, uint8_t *tx_buffer, size_t buff_len);
err_return u_uart0_get_rx_buf(uint8_t * data_buff, size_t data_len);
err_return u_uart1_get_rx_buf(uint8_t * data_buff, size_t data_len);
err_return get_module_info();
err_return uhf_single_poll();
err_return write_power(uint8_t pow);


#endif //U_UART_TURN_H