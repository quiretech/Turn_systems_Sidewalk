#ifndef U_UART_TURN_H
#define U_UART_TURN_H

#include <zephyr/kernel.h>

#define RECEIVE_BUFF_SIZE   256
#define TRANSMIT_BUFF_SIZE  256
#define RECEIVE_TIMEOUT     2000     // U_SEC
#define PACKAGE_TIMEOUT     5       // M_SEC


typedef int err_return;

err_return u_uart_init();
err_return u_uart_send(uint8_t *tx_buffer, size_t buff_len);
err_return u_uart_get_rx_buf(uint8_t * data_buff, size_t data_len);
err_return get_module_info();
err_return single_poll();
err_return write_power(uint8_t pow);


#endif //U_UART_TURN_H