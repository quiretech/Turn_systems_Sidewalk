#include <stdio.h>
#include <zephyr/zephyr.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <u_i2c_turn.h>

#define TWI_INSTANCE_ID     1
#define TWI_ADDRESSES      127

bool twi_addr[TWI_ADDRESSES+1];

const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

uint8_t i2c_ready(void)
{
    if (i2c_dev == NULL || !device_is_ready(i2c_dev))
    {
        printk("Could not get I2C device\n");
        return -1;
    }
    else
    {
        printk("I2C device Ready\n");
        return 0;
    }
}

uint8_t i2c_scan(void)
{
    uint8_t err_code;
    uint8_t address;
    uint8_t sample_data[2];
    int ret = -1;

    for (address = 1; address <= TWI_ADDRESSES; address++)
    {
        //m_xfer_done = false;
        twi_addr[address] = false;
        err_code = i2c_read(i2c_dev,sample_data,1,address);
        //APP_ERROR_CHECK(err_code);
        // while (m_xfer_done == false); //wait for the handler to be done

        if (err_code == 0)
        {
            //detected_device = true;
            twi_addr[address] = true;
            //printk("TWI device detected at address 0x%x\n.", address);
            ret = 0;
        }
        
    }

    return ret;
}

bool check_i2c_address(uint8_t addr) {
    return twi_addr[addr];
}

uint8_t u_i2c_write_read(uint16_t addr, uint8_t *tx_data, uint8_t tx_data_len, uint8_t *rx_data, uint8_t rx_data_len)
{
    uint8_t err_code;
    err_code = i2c_write_read(i2c_dev, addr, tx_data, tx_data_len, rx_data, rx_data_len);

    if(err_code == 0)
    {
        //printk("i2c write read success\n");
    }
    else
    {
        printk("i2c write read non\n");
    }

    return err_code;
}