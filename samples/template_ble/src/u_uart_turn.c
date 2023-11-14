/*
 * Copyright (c) 2023 Quiretech
 *
 * 
 * 
 * To use this module in the project
 * 1. Include the following in the prj.config
 *      CONFIG_SERIAL=y
 *      CONFIG_UART_ASYNC_API=y     // If inturrpt services need to be used
 * 
 * 
 * 
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>

#include <u_uart_turn.h>

// UART
const struct device *uart           = DEVICE_DT_GET(DT_NODELABEL(uart0));
const struct device *uart1           = DEVICE_DT_GET(DT_NODELABEL(uart1));
//static uint8_t      tx_buf[]        = {0x0a,0x55,0x0d};//{"Test String\n\r"};
//static uint8_t uhf_sample[] =  {"UU3000E280689000004006D0B55C1125E711"};//{"Test String\n\r"};
static uint8_t rx_buf[RECEIVE_BUFF_SIZE] = {0};

uint16_t received_msg_length;

K_MSGQ_DEFINE(u_uart_received_msgq, sizeof(received_msg_length), 10, 4);

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{

printk("UART Callback\n");
switch (evt->type) {

	case UART_RX_RDY:
        //printk("uart receive event\n");
        received_msg_length =  evt->data.rx.len;

        k_msgq_put(&u_uart_received_msgq, (void *)&received_msg_length, K_NO_WAIT);
        
        uart_rx_disable(uart);

        //printk("%s\n",rx_buf);
        //printk("%i\n",received_msg_length);
        received_msg_length = 0;
        break;

    case UART_RX_STOPPED:
        //printk("uart stopped event\n");
        break;

	case UART_RX_DISABLED:
		//printk("uart disable event\n");
        uart_rx_enable(dev ,rx_buf,sizeof rx_buf,RECEIVE_TIMEOUT);
		break;
    case UART_TX_DONE:
        //printk("TX Done %s\n", evt->data.tx.buf);
        //printk("TX Done %i\n", evt->data.tx.len);
        break;
		
	default:
		break;
	}
}

err_return u_uart_init()
{
    int ret;

    if (!device_is_ready(uart)){
		printk("UART device not ready\r\n");
		return 1 ;
	}
    else{
        printk("UART device ready\r\n");
    }

	ret = uart_callback_set(uart, uart_cb, NULL);
	if (ret) {
        printk("UART callback not set %i\r\n", ret);
		return ret;
	}
    else{
        printk("UART callback set\r\n");
    }
    
    ret = uart_rx_enable(uart ,rx_buf,sizeof rx_buf,RECEIVE_TIMEOUT);
	if (ret) {
        printk("UART rx not enabled %i\r\n", ret);
		return ret;
	}
    else{
        printk("UART rx enabled\r\n");
    }
}

err_return u_uart_send(uint8_t *tx_buffer, size_t buff_len)
{
    err_return ret = 0;

    //static uint8_t send_power[] = {0x0a,0x4e,0x31,0x2c,0x32,0x30,0x0d};
    //ret = uart_tx(uart, send_power, sizeof(send_power), SYS_FOREVER_MS);
    ret = uart_tx(uart, tx_buffer, buff_len, SYS_FOREVER_MS);
    if(ret)
    {
        printk("Error sending Uart data\r\n%i", ret);
        return ret;
    }

    return ret;
}

err_return u_uart_get_rx_buf(uint8_t *data_buff, size_t data_len)
{
    int ret = 0;
    memcpy(data_buff,rx_buf,data_len);

    return ret;
}


err_return get_module_info()
{
 //   uint8_t send_module[8] = {0xBB, 0x00, 0x03, 0x00, 0x01, 0x00, 0x04, 0x7E};//Get module info

    err_return ret = 0;
    //uint16_t msg_len;
    static uint8_t send_module[] = {0x0a,0x56,0x0d};
    ret = u_uart_send(send_module, sizeof(send_module));

    return ret;
}

err_return single_poll()
{
    //uint8_t send_single[7] = {0xbb,0x00,0x22,0x00,0x00,0x22,0x7e}; //single poll commmand

    err_return ret = 0;
    int data_len;
    static uint8_t send_single[] = {0x0a,0x55,0x0d};
    ret = u_uart_send(send_single, sizeof send_single);

    // if(k_msgq_get(&u_uart_received_msgq, &data_len, K_NO_WAIT))
    //     {
    //         //uhf_data_received = true;
    //         printk("got msg queue%i",data_len);
    //     }

    return ret;
}


//Ranges between 0 to 25
err_return write_power(uint8_t pow)
{
    err_return ret = 0;
    static uint8_t send_power[] = {0x0a,0x4e,0x31,0x2c,0x32,0x30,0x0d};
    send_power[4] = (pow/10)+48;
    send_power[5] = (pow%10)+48;
    ret = u_uart_send(send_power, sizeof(send_power));
    return 0;
}