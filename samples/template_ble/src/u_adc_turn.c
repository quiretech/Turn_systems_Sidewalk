#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <u_adc_turn.h>

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};

int u_adc_init(void){

    int err = 0;
	//uint32_t count = 0;
	

	/* Configure channels individually prior to sampling. */
	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		if (!device_is_ready(adc_channels[i].dev)) {
			printk("ADC controller device %s not ready\n", adc_channels[i].dev->name);
			return err;
		}

		err = adc_channel_setup_dt(&adc_channels[i]);
		if (err < 0) {
			printk("Could not setup channel #%d (%d)\n", i, err);
			return err;
		}
	}
    return err;

}

int32_t u_adc_bin_level(void){

    int err;
    uint16_t buf;
    int32_t val_mv;

	struct adc_sequence sequence = {
		.buffer = &buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(buf),
	};

    (void)adc_sequence_init_dt(&adc_channels[1], &sequence);

    err = adc_read(adc_channels[1].dev, &sequence);
    if (err < 0) {
        printk("Could not read (%d)\n", err);
        return err;
    }

    val_mv = (int32_t)buf;
	//printk("%"PRId32, val_mv);
	err = adc_raw_to_millivolts_dt(&adc_channels[1], &val_mv);

    if (err < 0) {
        printk(" (value in mV not available)\n");
        return err;
    } else {
        //printk(" = %"PRId32" mV\n", val_mv);
        return val_mv;
    }

}

int32_t u_adc_batt_volt(void){
    
    int err;
    uint16_t buf;
    int32_t val_mv;

	struct adc_sequence sequence = {
		.buffer = &buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(buf),
	};

    (void)adc_sequence_init_dt(&adc_channels[0], &sequence);

    err = adc_read(adc_channels[0].dev, &sequence);
    if (err < 0) {
        printk("Could not read (%d)\n", err);
        return err;
    }

    val_mv = (int32_t)buf;
	//printk("%"PRId32, val_mv);
	err = adc_raw_to_millivolts_dt(&adc_channels[0], &val_mv);

    if (err < 0) {
        printk(" (value in mV not available)\n");
        return err;
    } else {
        //printk(" = %"PRId32" mV\n", val_mv);
        return val_mv;
    }

}