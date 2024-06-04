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


#if defined(CONFIG_TURN_APP)

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>

#include <u_uart_turn.h>



// UART
const struct device *uart0           = DEVICE_DT_GET(DT_NODELABEL(uart0));
const struct device *uart1           = DEVICE_DT_GET(DT_NODELABEL(uart1));
//static uint8_t      tx_buf[]        = {0x0a,0x55,0x0d};//{"Test String\n\r"};
//static uint8_t uhf_sample[] =  {"UU3000E280689000004006D0B55C1125E711"};//{"Test String\n\r"};
static uint8_t rx0_buf[RECEIVE_BUFF_SIZE] = {0};
static uint8_t rx1_buf[RECEIVE_BUFF_SIZE] = {0};

uint16_t received_msg_length;

K_MSGQ_DEFINE(u_uart0_received_msgq, sizeof(received_msg_length), 10, 4);
K_MSGQ_DEFINE(u_uart1_received_msgq, sizeof(received_msg_length), 10, 4);

static void uart0_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{

//printk("UART0 Callback\n");
switch (evt->type) {

	case UART_RX_RDY:
        //printk("uart0 receive event\n");
        received_msg_length =  evt->data.rx.len;

        if(received_msg_length > 10) // set the message queue only if the message has ID
        {
            k_msgq_put(&u_uart0_received_msgq, (void *)&received_msg_length, K_NO_WAIT);
            //printk("%s\n",rx0_buf);
            //printk("%i\n",received_msg_length);
        }
        
        uart_rx_disable(uart0);

        received_msg_length = 0;
        break;

    case UART_RX_STOPPED:
        //printk("uart0 stopped event\n");
        break;

	case UART_RX_DISABLED:
		//printk("uart0 disable event\n");
        uart_rx_enable(dev ,rx0_buf,sizeof rx0_buf,RECEIVE_TIMEOUT);
		break;
    case UART_TX_DONE:
        //printk("TX Done %s\n", evt->data.tx.buf);
        //printk("TX Done %i\n", evt->data.tx.len);
        break;
		
	default:
		break;
	}
}

static void uart1_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{

//printk("UART1 Callback\n");
switch (evt->type) {

	case UART_RX_RDY:
        //printk("uart1 receive event\n");
        received_msg_length =  evt->data.rx.len;

        if(received_msg_length > 10) // set the message queue only if the message has ID
        {
            k_msgq_put(&u_uart1_received_msgq, (void *)&received_msg_length, K_NO_WAIT);
        }
        
        uart_rx_disable(uart1);

        //printk("%s\n",rx1_buf);
        
        //printk("%i\n",received_msg_length);
        received_msg_length = 0;
        break;

    case UART_RX_STOPPED:
        //printk("uart1 stopped event\n");
        break;

	case UART_RX_DISABLED:
		//printk("uart1 disable event\n");
        uart_rx_enable(dev ,rx1_buf,sizeof rx1_buf,RECEIVE_TIMEOUT);
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

    if (!device_is_ready(uart0)){
		printk("UART0 device not ready\r\n");
		return 1 ;
	}
    else{
        printk("UART0 device ready\r\n");
    }

	ret = uart_callback_set(uart0, uart0_cb, NULL);
	if (ret) {
        printk("UART0 callback not set %i\r\n", ret);
		return ret;
	}
    else{
        printk("UART0 callback set\r\n");
    }
    
    ret = uart_rx_enable(uart0 ,rx0_buf,sizeof rx0_buf,RECEIVE_TIMEOUT);
	if (ret) {
        printk("UART0 rx not enabled %i\r\n", ret);
		return ret;
	}
    else{
        printk("UART0 rx enabled\r\n");
    }

    if (!device_is_ready(uart1)){
    printk("UART1 device not ready\r\n");
    return 1 ;
	}
    else{
        printk("UART1 device ready\r\n");
    }

	ret = uart_callback_set(uart1, uart1_cb, NULL);
	if (ret) {
        printk("UART1 callback not set %i\r\n", ret);
		return ret;
	}
    else{
        printk("UART1 callback set\r\n");
    }
    
    ret = uart_rx_enable(uart1 ,rx1_buf,sizeof rx1_buf,RECEIVE_TIMEOUT);
	if (ret) {
        printk("UART1 rx not enabled %i\r\n", ret);
		return ret;
	}
    else{
        printk("UART1 rx enabled\r\n");
    }
}

err_return u_uart_send(const struct device *dev, uint8_t *tx_buffer, size_t buff_len)
{
    err_return ret = 0;

    //static uint8_t send_power[] = {0x0a,0x4e,0x31,0x2c,0x32,0x30,0x0d};
    //ret = uart_tx(uart0, send_power, sizeof(send_power), SYS_FOREVER_MS);
    ret = uart_tx(dev, tx_buffer, buff_len, SYS_FOREVER_MS);
    if(ret)
    {
        printk("Error sending Uart data\r\n%i", ret);
        return ret;
    }

    return ret;
}

err_return u_uart0_get_rx_buf(uint8_t *data_buff, size_t data_len)
{
    int ret = 0;

    //printk("%s\n",rx0_buf);
    memcpy(data_buff,rx0_buf,data_len);
    memset(rx0_buf, 0, RECEIVE_BUFF_SIZE); // Reset buffer data

    return ret;
}

err_return u_uart1_get_rx_buf(uint8_t *data_buff, size_t data_len)
{
    int ret = 0;

    //printk("%s\n",rx1_buf);
    memcpy(data_buff,rx1_buf,data_len);
    memset(rx1_buf, 0, RECEIVE_BUFF_SIZE); // reset buffer data 

    return ret;
}


err_return get_module_info()
{
 //   uint8_t send_module[8] = {0xBB, 0x00, 0x03, 0x00, 0x01, 0x00, 0x04, 0x7E};//Get module info

    err_return ret = 0;
    //uint16_t msg_len;
    static uint8_t send_module[] = {0x0a,0x56,0x0d};
    ret = u_uart_send(uart0, send_module, sizeof(send_module));

    return ret;
}

err_return uhf_single_poll()
{
    //uint8_t send_single[7] = {0xbb,0x00,0x22,0x00,0x00,0x22,0x7e}; //single poll commmand

    err_return ret = 0;
    int data_len;
    static uint8_t send_single[] = {0x0a,0x55,0x0d};
    ret = u_uart_send(uart0, send_single, sizeof send_single);

    // if(k_msgq_get(&u_uart_received_msgq, &data_len, K_NO_WAIT))
    //     {
    //         //uhf_data_received = true;
    //         printk("got msg queue%i",data_len);
    //     }

    return ret;
}


// void u_uart_send_data(uint8_t, uint8_t *tx_buffer, size_t buff_len)
// {
//     int ret;
//     ret = u_uart_send(uart0, tx_buffer, buff_len);
// }

//Ranges between 0 to 25
err_return ufh_power_set(uint8_t pow)
{
    err_return ret = 0;
    static uint8_t send_power[] = {0x0a,0x4e,0x31,0x2c,0x32,0x30,0x0d};
    send_power[4] = (pow/10)+48;
    send_power[5] = (pow%10)+48;
    ret = u_uart_send(uart0, send_power, sizeof(send_power));
    return 0;
}

#endif //#if defined(CONFIG_TURN_APP)