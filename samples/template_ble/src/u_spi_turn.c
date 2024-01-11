#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#include <u_spi_turn.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

// #define SPI1_NODE   DT_NODELABEL(spi1)
// static const struct device *spi1_dev = DEVICE_DT_GET(SPI1_NODE);
// #define MY_GPIO1 DT_NODELABLE(gpoi1)
// #define GPIO_1_CS   7
// //const struct device *gpoi1_dev = DEVICE_DT_GET(MY_GPIO1);


// // SPI master functionconst struct device * spi_dev;

// static const struct spi_config spi_cfg = {
// 	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
// 		     SPI_MODE_CPOL | SPI_MODE_CPHA,
// 	.frequency = 4000000,
// 	.slave = 0,
// };

// static void u_spi_init(void)
// {
// 	const char* const spiName = "SPI_1";
// 	spi1_dev = device_get_binding(spiName);

// 	if (spi1_dev == NULL) {
// 		printk("Could not get %s device\n", spiName);
// 		return;
// 	}
// }


// void u_spi_init(void)
// {
	
// 	if(!device_is_ready(spi1_dev)) {
// 		printk("SPI master device not ready!\n");
// 	}
//     else{
//         printk("SPI master device is ready!\n");
//     }
// 	// if(!device_is_ready(spim_cs.gpio.port)){
// 	// 	printk("SPI master chip select device not ready!\n");
// 	// }
// }
