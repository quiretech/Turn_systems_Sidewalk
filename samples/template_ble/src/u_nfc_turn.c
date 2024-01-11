

#if defined(CONFIG_TURN_APP)

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>

#include <u_uart_turn.h>

uint16_t        data_len;
static uint8_t  data_buf[32];
bool            new_nfc_tag;


//extern struct   k_msgq u_uart1_received_msgq;


// void check_new_nfc_tag(uint8_t *data)
// {   
//     int ret;
//     new_nfc_tag = false;

//     if(!k_msgq_get(&u_uart1_received_msgq, &data_len, K_NO_WAIT))
//     {
//         if(data_len > 4)
//         {
//             ret = u_uart1_get_rx_buf(data_buf, data_len);
//             memcpy(data, data_buf, data_len);
//             new_nfc_tag = true;
//         }       
//         //printk("got UART0 msg queue");
//     }

//     return new_nfc_tag;
// }

// void nfc_single_poll(void)
// {
//         //uint8_t send_single[7] = {0xbb,0x00,0x22,0x00,0x00,0x22,0x7e}; //single poll commmand

//     err_return ret = 0;
//     int data_len;
//     static uint8_t send_nfc_scan[] = {0x0a,0x55,0x0d};
//     ret = u_uart_send_data(Uart_0, send_nfc_scan, sizeof send_nfc_scan);

//     return ret;
// }



#endif // (CONFIG_TURN_APP)