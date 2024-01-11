#if defined(CONFIG_TURN_APP)

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>
#include <u_led_turn.h>

#define I2S_TX_NODE  DT_NODELABEL(i2s_tx)
const struct device *const i2s_dev_tx = DEVICE_DT_GET(I2S_TX_NODE);


#define NUM_LEDS            180
#define NUMBER_OF_LEDS      ((NUM_LEDS+1) * 4)
#define SAMPLE_FREQUENCY    100000//44100
#define SAMPLE_BIT_WIDTH    16
#define BYTES_PER_SAMPLE    sizeof(int16_t)
#define NUMBER_OF_CHANNELS  2
/* Such block length provides an echo with the delay of 100 ms. */
#define SAMPLES_PER_BLOCK   ((SAMPLE_FREQUENCY / 1) * NUMBER_OF_CHANNELS)
#define INITIAL_BLOCKS      2
#define TIMEOUT             1000

#define BLOCK_SIZE  (NUMBER_OF_LEDS*4)//(BYTES_PER_SAMPLE * SAMPLES_PER_BLOCK)
#define BLOCK_COUNT (INITIAL_BLOCKS + 2)
K_MEM_SLAB_DEFINE_STATIC(tx_mem_slab, BLOCK_SIZE, BLOCK_COUNT, 4);

static K_SEM_DEFINE(toggle_transfer, 1, 1);

#define DATA_BYTES_PER_RGBW	    4	// 32-bit GRBW data structure
#define DATA_BYTES_PER_RGB	    3	

#define I2S_SK6812_ZERO		    0x8		//0b'1000 (0.3us; 0.9us)
#define I2S_SK6812_ONE		    0xC		//0b'1100 (0.6us; 0.6us)
#define I2S_WS2812B_ZERO	    0x8		//0b'1000 (0.4us; 0.85us)
#define I2S_WS2812B_ONE		    0xE		//0b'1110 (0.8us; 0.45us)
#define TYPE_RBGW               true

typedef uint32_t ret_code_t;

typedef struct {
    uint8_t g;
    uint8_t r;
    uint8_t b;
    uint8_t w;
} led_t_rgbw;

typedef struct {
    uint8_t g;
    uint8_t r;
    uint8_t b;
} led_t_rgb;

static uint32_t	    m_i2s_led_buffer_tx[NUM_LEDS*4];
led_t_rgbw          m_led_buffer_tx_rgbw[NUM_LEDS];
led_t_rgb           *m_led_buffer_tx_rgb;
uint32_t            leds_data_byte_size = NUM_LEDS*4;
uint32_t            i2s_leds_frame_word_size;
uint16_t            num_leds = NUM_LEDS;
bool                rgb_w = TYPE_RBGW;

uint8_t pepsi_data_set[180][4] = {  {0,0,0,0},{},{},{},{},{0,0,0,25},{},{},{},{},{},{},{0,0,0,25},{},{},{},{},{0,0,50,0},{0,0,50,0},{0,0,50,0},{},{0,0,0,25},{0,0,0,25},{0,0,0,25},{},{50,0,0,0},{50,0,0,0},{50,0,0,0},{},{0,0,0,25},{0,0,0,25},{0,0,0,25},{},{0,0,50,0},{0,0,50,0},{0,0,50,0}, \
                                    {0,0,0,0},{},{},{},{},{0,0,0,25},{},{},{},{},{},{},{0,0,0,25},{},{},{},{},{},{0,0,50,0},{},{},{0,0,0,25},{},{},{},{50,0,0,0},{},{50,0,0,0},{},{},{},{0,0,0,25},{},{0,0,50,0},{},{0,0,50,0}, \
                                    {0,0,0,0},{},{},{},{},{0,0,0,25},{},{},{},{},{},{},{0,0,0,25},{},{},{},{},{},{0,0,50,0},{},{},{0,0,0,25},{0,0,0,25},{0,0,0,25},{},{50,0,0,0},{50,0,0,0},{50,0,0,0},{},{},{0,0,0,25},{0,0,0,25},{},{0,0,50,0},{0,0,50,0},{0,0,50,0}, \
                                    {0,0,0,0},{},{},{},{},{0,0,0,25},{},{},{},{},{},{},{0,0,0,25},{},{},{},{},{},{0,0,50,0},{},{},{},{},{0,0,0,25},{},{},{},{50,0,0,0},{},{},{},{0,0,0,25},{},{},{},{0,0,50,0}, \
                                    {0,0,0,0},{},{},{},{},{0,0,0,25},{},{},{},{},{},{},{0,0,0,25},{},{},{},{},{0,0,50,0},{0,0,50,0},{0,0,50,0},{},{0,0,0,25},{0,0,0,25},{0,0,0,25},{},{},{},{50,0,0,0},{},{0,0,0,25},{0,0,0,25},{0,0,0,25},{},{},{},{0,0,50,0}};





int u_led_init(void)
{
	
    struct i2s_config tx_config;

    int ret = 0;

    if (!device_is_ready(i2s_dev_tx)) {
		printk("%s is not ready\n", i2s_dev_tx->name);
		return -1;
	}

    tx_config.word_size = SAMPLE_BIT_WIDTH;
	tx_config.channels = NUMBER_OF_CHANNELS;
	tx_config.format = I2S_FMT_DATA_FORMAT_I2S;
	tx_config.options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER;
	tx_config.frame_clk_freq = SAMPLE_FREQUENCY;
	tx_config.mem_slab = &tx_mem_slab;
	tx_config.block_size = BLOCK_SIZE;
	tx_config.timeout = TIMEOUT;

    ret = i2s_configure(i2s_dev_tx, I2S_DIR_TX, &tx_config);
    if (ret < 0) {
        printk("Failed i2s_configure(i2s_dev_tx) %i\n", ret);
		return ret;
	}

}

ret_code_t i2s_init_mem() 
{
    // Reset data buffers
    if(rgb_w)
    {
      memset(m_led_buffer_tx_rgbw, 0, leds_data_byte_size);
    }
    else
    {
      memset(m_led_buffer_tx_rgb, 0, leds_data_byte_size);
    }
//    memset(m_i2s_led_buffer_tx, 0, sizeof(m_i2s_led_buffer_tx));
    memset(m_i2s_led_buffer_tx, 0, sizeof(m_i2s_led_buffer_tx)*i2s_leds_frame_word_size);
    return 0;
}

uint32_t convert_byte_to_i2s_bits(uint8_t data_byte) {
    uint32_t data_bits = 0;
    
    // Set data_bits based on MSB, then left-shift data_byte
    for (int ii=0; ii < 4; ii++) {
//	data_bits |= ((data_byte & 0x80) ? I2S_SK6812_ONE : I2S_SK6812_ZERO) << ((8-1-ii) * 4);
	data_bits |= ((data_byte & 0x80) ? I2S_WS2812B_ONE : I2S_WS2812B_ZERO) << ((4-1-ii) * 4);
	data_byte = data_byte << 1;
    }
    for (int ii=4; ii < 8; ii++) {
//	data_bits |= ((data_byte & 0x80) ? I2S_SK6812_ONE : I2S_SK6812_ZERO) << ((12-1-ii) * 4);
	data_bits |= ((data_byte & 0x80) ? I2S_WS2812B_ONE : I2S_WS2812B_ZERO) << ((12-1-ii) * 4);
	data_byte = data_byte << 1;
    }

    return data_bits;
}

void set_led_pixel_RGB(uint16_t pos, uint8_t r, uint8_t g, uint8_t b) {
    m_led_buffer_tx_rgb[pos].r = r;
    m_led_buffer_tx_rgb[pos].g = g;
    m_led_buffer_tx_rgb[pos].b = b;
}

void set_led_pixel_RGBW(uint16_t pos, uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
    m_led_buffer_tx_rgbw[pos].r = r;
    m_led_buffer_tx_rgbw[pos].g = g;
    m_led_buffer_tx_rgbw[pos].b = b;
    m_led_buffer_tx_rgbw[pos].w = w;

    //printk("RGBW, %i,%i,%i,%i,\n",r,g,b,w);
    //turn_leds_on();
}

void set_i2s_led_data() {
    uint16_t jj = 0;
    for (uint16_t ii=0; ii < num_leds; ii++) {
	if(rgb_w == TYPE_RBGW){
        m_i2s_led_buffer_tx[jj]     = convert_byte_to_i2s_bits(m_led_buffer_tx_rgbw[ii].g);
	    m_i2s_led_buffer_tx[jj+1]   = convert_byte_to_i2s_bits(m_led_buffer_tx_rgbw[ii].r);
	    m_i2s_led_buffer_tx[jj+2]   = convert_byte_to_i2s_bits(m_led_buffer_tx_rgbw[ii].b);
        m_i2s_led_buffer_tx[jj+3]   = convert_byte_to_i2s_bits(m_led_buffer_tx_rgbw[ii].w);
	    jj += 4;
        }
        else
        {
          m_i2s_led_buffer_tx[jj] = convert_byte_to_i2s_bits(m_led_buffer_tx_rgb[ii].g);
          m_i2s_led_buffer_tx[jj+1] = convert_byte_to_i2s_bits(m_led_buffer_tx_rgb[ii].r);
          m_i2s_led_buffer_tx[jj+2] = convert_byte_to_i2s_bits(m_led_buffer_tx_rgb[ii].b);
          jj +=3;
        }
    }
}

void send_i2s_led_data() {

    int ret = 0;
    void *tx_mem_block;

    k_mem_slab_alloc(&tx_mem_slab, &tx_mem_block, K_NO_WAIT);
    memcpy(tx_mem_block, m_i2s_led_buffer_tx, BLOCK_SIZE);


    //ret = i2s_write(i2s_dev_tx, m_i2s_led_buffer_tx, (BLOCK_SIZE));
    ret = i2s_write(i2s_dev_tx, tx_mem_block, (BLOCK_SIZE));
    if (ret < 0) {
        printk("Failed to write dataddd: %d\n", ret);
        //break;
    }
    else{
        //printk("Success writing I2S \n");
    }
    ret = i2s_trigger(i2s_dev_tx, I2S_DIR_TX, I2S_TRIGGER_START);
    if (ret < 0) {
        printk("Failed to trigger command %d on TX: %d\n", I2S_TRIGGER_START, ret);
        return false;
    }
    ret = i2s_trigger(i2s_dev_tx, I2S_DIR_TX, I2S_TRIGGER_STOP);
    if (ret < 0) {
        printk("Failed to trigger command %d on TX: %d\n", I2S_TRIGGER_STOP, ret);
        return false;
    }

}

void turn_leds_on()
{
    set_i2s_led_data();
    send_i2s_led_data();
}

void color_wipe(uint8_t r, uint8_t g, uint8_t b, uint8_t w, uint32_t ms_delay) 
{
    i2s_init_mem();

    for(uint16_t ii=0; ii < num_leds; ii++) 
    {
    if(TYPE_RBGW)
        {
          set_led_pixel_RGBW(ii, r, g, b, w);
        }
        else
        {
          set_led_pixel_RGB(ii, r, g, b);
        }
    }
    turn_leds_on();
}

void all_on(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
    
    i2s_init_mem();
    
    for(uint16_t ii=0; ii < num_leds; ii++) {
	if(rgb_w)
        {
          set_led_pixel_RGBW(ii, r, g, b, w);
        }
        else
        {
          set_led_pixel_RGB(ii, r, g, b);
        }
    }
    turn_leds_on();
}


void position_on(uint16_t pos,uint8_t r, uint8_t g, uint8_t b, uint8_t w, bool keep_state)
{
    if(!keep_state)
    {
      i2s_init_mem();
    }
	if(rgb_w)
        {
          set_led_pixel_RGBW(pos, r, g, b, w);
        }
        else
        {
          set_led_pixel_RGB(pos, r, g, b);
        }
     turn_leds_on();
}

void position_on_range(uint8_t pos1, uint8_t pos2, uint8_t r, uint8_t g, uint8_t b, uint8_t w, bool keep_state)
{
   if(!keep_state)
    {
      i2s_init_mem();
    }
    for(int i = pos1; i<=pos2;i++)
    {
      if(rgb_w)
        {
          set_led_pixel_RGBW(i, r, g, b, w);
        }
        else
        {
          set_led_pixel_RGB(i, r, g, b);
        }
    }
    turn_leds_on();
}


void display_pepsi(void)
{
    for(int pos = 0; pos<180; pos++)
    {
        //set_led_pixel_RGBW(i, pepsi_data_set[i][0], pepsi_data_set[i][1], pepsi_data_set[i][2], pepsi_data_set[i][3]);
        m_led_buffer_tx_rgbw[pos].r = pepsi_data_set[pos][0];
        m_led_buffer_tx_rgbw[pos].g = pepsi_data_set[pos][1];
        m_led_buffer_tx_rgbw[pos].b = pepsi_data_set[pos][2];
        m_led_buffer_tx_rgbw[pos].w = pepsi_data_set[pos][3];
    }
    turn_leds_on();
}

void display_lid_jam(void)
{
    for(int pos = 0; pos<180; pos++)
    {
        //set_led_pixel_RGBW(i, pepsi_data_set[i][0], pepsi_data_set[i][1], pepsi_data_set[i][2], pepsi_data_set[i][3]);
        m_led_buffer_tx_rgbw[pos].r = lid_jam_data_set[pos][0];
        m_led_buffer_tx_rgbw[pos].g = lid_jam_data_set[pos][1];
        m_led_buffer_tx_rgbw[pos].b = lid_jam_data_set[pos][2];
        m_led_buffer_tx_rgbw[pos].w = lid_jam_data_set[pos][3];
    }
    turn_leds_on();
}

void display_thanks(void)
{
    for(int pos = 0; pos<180; pos++)
    {
        //set_led_pixel_RGBW(i, pepsi_data_set[i][0], pepsi_data_set[i][1], pepsi_data_set[i][2], pepsi_data_set[i][3]);
        m_led_buffer_tx_rgbw[pos].r = thanks_data_set[pos][0];
        m_led_buffer_tx_rgbw[pos].g = thanks_data_set[pos][1];
        m_led_buffer_tx_rgbw[pos].b = thanks_data_set[pos][2];
        m_led_buffer_tx_rgbw[pos].w = thanks_data_set[pos][3];
    }
    turn_leds_on();
}

#endif //#if defined(CONFIG_TURN_APP)