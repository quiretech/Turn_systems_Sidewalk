/*
 * Copyright (c) 2023 Quiretech
 *
 * 
 */
#if defined(CONFIG_SIDEWALK)
    #include <sid_api.h>
    #include <sid_error.h>
    #include <sid_hal_reset_ifc.h>

    #include <state_notifier.h>
    #if defined(CONFIG_SIDEWALK_CLI)
    #include <sid_shell.h>
    #endif

    #include <application_thread.h>
#endif // endif defined(CONFIG_SIDEWALK)



#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/drivers/uart.h>



#include <u_app_turn.h>
#include <u_turn_config.h>
#include <u_uart_turn.h>
#include <u_adc_turn.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <inttypes.h>
#include <zephyr/drivers/pwm.h>



#if defined(CONFIG_TURN_APP)

#define TURN_THREAD_PRIORITY        14
#define TURN_THREAD_START_DELAY_MS  2000

extern struct k_msgq u_uart_received_msgq;

static const struct pwm_dt_spec pwm_led0 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0));

#define NUM_STEPS	50U
#define SLEEP_MSEC	25U
#define MAX_PULSE_WIDTH 2500000 // In nsec
#define MIN_PULSE_WIDTH  500000	// In nsec
#define MID_PULSE_WIDTH 1500000 // In nsec
#define STEP 		          10000	// In nsec


/*
 * Get button configuration from the devicetree doww_open alias. This is mandatory.
 */
#define DOOR_OPEN	DT_ALIAS(dooropen)
#if !DT_NODE_HAS_STATUS(DOOR_OPEN, okay)
#error "Unsupported board: dooropen devicetree alias is not defined"
#endif
static const struct gpio_dt_spec door_open = GPIO_DT_SPEC_GET_OR(DOOR_OPEN, gpios,
							      {0});
static struct gpio_callback door_open_cb_data;

// UART
// const struct device *uart= DEVICE_DT_GET(DT_NODELABEL(uart0));
// static uint8_t tx_buf[] =  {0x0a,0x55,0x0d};//{"Test String\n\r"};
// static uint8_t uhf_sample[] =  {"UU3000E280689000004006D0B55C1125E711"};//{"Test String\n\r"};
// static uint8_t rx_buf[RECEIVE_BUFF_SIZE] = {0};

#define TURN_THREAD_STACK_SIZE 1024

//Definitions
#define M_1500_MSEC         1500              //1500 millisecond
#define M_1000_MSEC         1000              //1000 millisecond
#define M_100_MSEC          100               //100 millisecond
#define M_200_MSEC          200               //200 millisecond
#define M_50_MSEC           50                //50 millisecond
#define M_500_MSEC          500               //500 millisecond
#define M_25_MSEC           25                //25 millisecond
#define M_10_MSEC           10                //10 millisecond
#define M_3_SEC             (3*M_1000_MSEC)   //3 seconds
#define M_5_SEC             (5*M_1000_MSEC)   //5 seconds
#define M_7_SEC             (7*M_1000_MSEC)   //7 seconds
#define M_1_MIN             (60 * M_1000_MSEC)//1 minute
#define M_2_MIN             (2 * M_1_MIN)     //2 minutes
#define M_10_MIN            (10 * M_1_MIN)    //10 minutes
#define M_30_MIN            (30 * M_1_MIN)    //30 minutes
#define LID_JAM_RETRY       4                 //Number of retries if lid jam is triggered. The motor opens and closes the door.
#define LID_CLOSE_PWM       90                //90
#define PWD                 "12345"           //Password to reset nrf52 used in ble communication
#define ESP32_PWD           "54321"           //Password to reset esp32
#define SHTDWN              "Shutdown 1"      //AWS command to turn off 5 volt



//System variable
bool          toggle = false;         //To toggle leds for low battery indication
volatile bool ble_connected;          //bool to signify if ble is connected toggled in main.c
static volatile bool ready_flag;      // bool for pwm callback function
uint8_t       bin_full_counter = 0;   // counter to toggle leds/buzzer for bin full indication
uint8_t       lid_jam_counter = 0;    // counter to toggle leds/buzzer for lid jam indication
uint16_t      read_loc = 0;           // read location for memory read and transmit via ble
volatile bool mem_read;               //bool to start memory read
uint16_t      buzzer_freq = FREQ_IN_US; //Frequency in microseconds defined in config.h
uint8_t       buzzer_pwm = PWM_DUTY;  //Duty cycle for pwm
uint8_t       timer_counter = 0; 
bool          flag_50ms = false;      //50ms flag
bool          flag_200ms = false;     //200ms flag
bool          flag_10s = false;     //200ms flag
bool          door_is_open = false;
bool          door_state_change = false;
bool          uhf_data_received = false;
uint16_t      uhf_pacaket_len;
uint8_t       counter_uhf = 0; 
uint8_t       nfc_counter = 0;
uint8_t       eeprom_no = 1;
bool          chip_read= false;
uint8_t       servo_pwm = LID_CLOSE_PWM;
bool          Enable_5V = false;
uint8_t       clear_counter = 0;
bool          red_led = false;
uint8_t       lid_jam_retry = 0;
uint8_t       password[5];
uint8_t       shtdn[10];
bool          prox;                 // variable for proximity sensor
uint32_t      data_id_counter = 0;
bool cup_in = false;

//Different states for state machine
typedef enum
{
  Idle = 0,
  NFC,
  UHF,
  Mem_write,
  Mem_write_UHF,
  Mem_read,
  Sys_Check,
  BLE,
  default_state
}states;

//Different events stored in packets
typedef enum
{
  NFC_EVE = 1,
  UHF_EVE,
  LID_EVE,
  BIN_FULL,
  LOW_BATT,
  LID_JAM,
  SYS_CHECK,
  SYS_OFF
}events;

//Data structure to be stored in EEPROM
// struct data{
//   uint8_t header[2];
//   uint8_t data_id[3];
//   uint8_t time[4];
//   uint8_t bin_id[6];
//   uint8_t event_id;
//   uint8_t data[13];
//   uint8_t crc[2];
//   uint8_t footer;
// };

turn_data_packet turn_packet;


//this structure to simplify all the variable management
struct
{
  uint8_t uid[7]; //Variable to store NFC UID
  uint8_t header[2]; //Header of data packer
  uint8_t data_id[3]; // Unique data id for every event
  uint8_t bin_id[6]; // Bin ID different for every bin
  uint8_t adc_bin; // analog sensor reading value of bin full sensor
  uint8_t adc_batt; // analog sensor reading value of battery sensor
  uint8_t uhf_data[256]; // data recieved from uart communication with the UHF
  uint8_t uhf_ids[256]; // data packets with uhf id to be stored in memory
  uint8_t cup_id[12]; //cup id uhf data
  uint8_t cup_id_temp[24]; //Temp storage for uhf cup data (ASCII characters needed to be converted to hex)
  uint8_t cup; //cups read in a single read
  uint8_t time[4];//Set time variable
  uint8_t u_ble_rec_buf[4];
  uint8_t cup_no; //Total number of cups read since lid open
  bool uid_read;  // NFC read successful or not
  bool uid_read_prev; //Previous UID to stop same uid write multiple times 
  bool system_check; // System check
  bool lid_open; //LID open check
  bool lid_open_prev;
  bool lid_jam; //LID JAM
  bool ble_on; // Variable to prevent from calling advertsing start multiple times
  bool mem_write; // After making a packet called to write the packet to eeprom
  bool periodic_sys_check; // periodic system check flag
  bool bin_full; // bin full indicator
  bool uhf_timeout; // used when lid jam occurs
  bool uhf_mem_write; // used to write uhf data to eeprom
  bool batt_low;
  bool sys_off;

  states current_state; //current state of state machine
  states previous_state; //previous state of state machine
}this;


void init_data_packet(void){

  memset(&turn_packet,0,32);
  
	//turn_packet.header[0] 	= 0x31;
	//turn_packet.header[1] 	= 0x32;
	// turn_packet.data_id[0] = 0x33;
	// turn_packet.data_id[1] = 0x34;
	// turn_packet.data_id[2] = 0x35;

	//static struct sid_msg msg;
	//static struct sid_msg_desc desc;
	//sid_get_time(application_ctx->handle, SID_GET_GPS_TIME, &curr_time);	
	//data.time = curr_time.tv_sec;
	//uint8_t i;
	//uint32_t temp_time = 0;
  //printk("Current time second is:%i\n",turn_packet.time[0]);
	//LOG_INF("Current time second is:%u",temp_time);
	//for(i = 0; i < 4; i++)
	//{
		//turn_packet.time[3-i] = (uint8_t)(temp_time >> (i*8) & 0x000000ff);
		//LOG_INF("Current time second is:%i",(data.time[3-i]));
    //printk("Current time second is:%i\n",turn_packet.time[3-i]);
	//}


	//LOG_INF("Current time second is:%u",data.time);
	//LOG_INF("Size of data.time:%u",sizeof(data.time));
	//LOG_INF("Size of curr_time.sec:%u",sizeof(curr_time.tv_sec));
	//LOG_INF("Size of Data:%u",sizeof(data));
	
	strcpy(turn_packet.bin_id, BIN_ID);         // Set the bin ID from u_turn_config.h
	//turn_packet.event_id = 0xFF;                // Event ID set by the message handler
	//strcpy(turn_packet.data,"123456acbdefg");   // turn data, depends on the event type
	//turn_packet.crc[0] = 0x10;                  // need to implement
	//turn_packet.crc[1] = 0x01;                  // need to implement
	//turn_packet.footer = 0xFF;                  // End of package    


}
//struct data packet; // Packet based on data struct
//struct data packet2;


// #define DOOR_OPEN DT_ALIAS(dooropen)

// #if !DT_NODE_HAS_STATUS(DOOR_OPEN, okay)
// #error"Unsupported board: sw0 devicetree alias is not defined"
// #endif
// static struct gpio_dt_spec doorOpen = GPIO_DT_SPEC_GET_OR(DOOR_OPEN, gpios,
// 							      {0});
//static struct gpio_callback doorOpen_cb;



/* 
Function to convert 2 byte ascii characters to 1 byte hex character

for example 0x45('E') 0x32('2') gets stored as 0xE2 in hex data 
*/

void ascii_to_hex(uint8_t* ascii_dat,uint8_t* hex_data,uint8_t len)
{
    for(int i=0;i<len;i++)
    {
      if(ascii_dat[i]< 0x40){
      //printf("%x",ascii_dat[i]);
      ascii_dat[i] = (ascii_dat[i]&0x0f);
      }
      else
      {
       ascii_dat[i] = 0x0a + ((ascii_dat[i]&0x0f) -1);
      }
      if(i%2==0)
      {
        ascii_dat[i]=ascii_dat[i]<<4;
      }
      else
      {
        hex_data[i/2] = ascii_dat[i-1] + ascii_dat[i];
      }
    }
}

static void uhf_timer_cb(struct k_timer *timer_id);
K_TIMER_DEFINE(uhf_timer, uhf_timer_cb, NULL);

static void system_timer_cb(struct k_timer *timer_id);
K_TIMER_DEFINE(system_timer, system_timer_cb, NULL);

// Door section
void door_open_function(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{

  int i;
	//printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
  door_state_change = true;
  data_id_counter++;

  //printk("Val of pin %d\n",gpio_pin_get_dt(&door_open));
  if(gpio_pin_get_dt(&door_open)){
    door_is_open = true;
    turn_packet.event_id = 0x02;
    //printk("Door open\n");
  } else
  {
    door_is_open = false;
    turn_packet.event_id = 0x03;
    //printk("Door close\n");
  }

  //printk("Data ID %d\n",data_id_counter);
  //turn_packet.event_id = 0x02;


  for(i = 0; i < 3; i++)
	{
		turn_packet.data_id[2-i] = (uint8_t)(data_id_counter >> (i*8) & 0x000000ff);
		//LOG_INF("Current time second is:%i",(data.time[3-i]));
	}
  
}
// End Door section


// Timer section

static void uhf_timer_cb(struct k_timer *timer_id)
{
  ARG_UNUSED(timer_id);

  int ret;

  ret = pwm_set_pulse_dt(&pwm_led0, MAX_PULSE_WIDTH);
  cup_in = true;
  if (ret) {
    printk("Error %d: failed to set pulse width\n", ret);
  }
}

static void system_timer_cb(struct k_timer *timer_id)
{
	ARG_UNUSED(timer_id);

    int ret;
    timer_counter++;
    static int uhf_counter = 0;

    if(timer_counter == 200)
    {
        
        //uart_tx(uart, tx_buf, sizeof(tx_buf), SYS_FOREVER_MS);
        //printk("Timer OFF\n");
	
        timer_counter = 0;
        //flag_200ms = true;
        flag_10s = true;
        //uhf_data_received = true;
        //uhf_pacaket_len = 36;
        //memcpy(rx_buf,uhf_sample,36);

        //dk_toggle_led(3);
    }

    uhf_counter++;
    if(uhf_counter > 20)
    {
        uhf_counter = 0;

        ret = single_poll();    
        
        //cup_in = true; 



    }

    
    flag_50ms = true;
    //NRF_LOG_FLUSH();


	//k_timeout_t delay = K_MSEC(CONFIG_SM_TIMER_DEMO_CAPABILITY_PERIOD_MS);


	//enum event_type event = EVENT_NOTIFICATION_TIMER_FIRED;

	/* if (BUILT_IN_LM == SID_LINK_TYPE_1) {
		if (!sm_is_sidewalk_ready()) {
			delay = K_MSEC(CONFIG_SM_TIMER_CONNECT_LINK_TYPE_1_DELAY_MS);
			event = EVENT_CONNECT_LINK_TYPE_1;
		}
	}

	if (sm_app_state_get() == DEMO_APP_STATE_NOTIFY_SENSOR_DATA && sm_is_sidewalk_ready()) {
		delay = K_MSEC(CONFIG_SM_TIMER_DEMO_NOTIFY_SENSOR_DATA_PERIOD_MS);
	} */
	//k_timer_start(&cap_timer, delay, Z_TIMEOUT_NO_WAIT);

	//sm_main_task_msg_q_write(event);
}

void system_timer_set_and_run(k_timeout_t delay)
{
	k_timer_start(&system_timer, delay, K_MSEC(50));
}

void system_timer_stop()
{
    k_timer_stop(&system_timer);
}

turn_data_packet* get_data_packket_pointer(void)
{
  //printk("Address of turn_packet at get packet%p\n", &turn_packet);
  return &turn_packet;
}
// End Timer Section

//Uart

// static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
// {

//  //printk("UART Callback\n");
// 	switch (evt->type) {

// 	case UART_RX_RDY:
//     //printk("uart receive event\n");
//     uhf_data_received = true;
//     uart_rx_disable(uart);
//     uhf_pacaket_len = evt->data.rx.len;
//     //uart_tx(uart, tx_buf, sizeof(tx_buf), SYS_FOREVER_MS);

//     printk("%s\n",rx_buf);
//     //printk("%i\n",evt->data.rx.len);
//     //printk("First data%i\n",evt->data.rx.offset);
//     //memset(rx_buf,0,32);
// 	  break;

//   case UART_RX_STOPPED:
//     //printk("uart stopped event\n");
//     break;
// 	case UART_RX_DISABLED:
// 		//printk("uart disable event\n");
//     uart_rx_enable(dev ,rx_buf,sizeof rx_buf,RECEIVE_TIMEOUT);
// 		break;
//   case UART_TX_DONE:
//     //printk("TX Done\n");
//     break;
		
// 	default:
// 		break;
// 	}
// }

//End Uart

// Thread Section

static struct k_thread turn_app_thread;
K_THREAD_STACK_DEFINE(turn_app_thread_stack, TURN_THREAD_STACK_SIZE);

static void turn_app_entry(void *dummy0, void *dummy1, void *dummy2){
    printk("turn_app_entry \n");

    uint32_t pulse_width = 0U;
	uint32_t step = STEP;//pwm_led0.period / NUM_STEPS;
	uint8_t dir = 1U;
    static uint8_t data_buf[RECEIVE_BUFF_SIZE] = {0};
    uint16_t data_len;

#if defined(CONFIG_SIDEWALK)
    static uint32_t event0 = BUTTON_EVENT_CONNECTION_REQUEST;
    static uint32_t event1 = BUTTON_EVENT_SEND_HELLO;
#endif // endif defined(CONFIG_SIDEWALK)

	ARG_UNUSED(dummy0);
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);

    int ret;
	if (!gpio_is_ready_dt(&door_open)) {
		//printk("Error: button device %s is not ready\n",door_open.port->name);
		//return 0;
	}

	ret = gpio_pin_configure_dt(&door_open, GPIO_INPUT);
	if (ret != 0) {
		//printk("Error %d: failed to configure %s pin %d\n",ret, door_open.port->name, door_open.pin);
		//return 0;
	}



	ret = gpio_pin_interrupt_configure_dt(&door_open,
					      GPIO_INT_EDGE_BOTH);
	if (ret != 0) {
		//printk("Error %d: failed to configure interrupt on %s pin %d\n",ret, door_open.port->name, door_open.pin);
		//return 0;
	}

    gpio_init_callback(&door_open_cb_data, door_open_function, BIT(door_open.pin));
	gpio_add_callback(door_open.port, &door_open_cb_data);

	//printk("Set up button at %s pin %d\n", door_open.port->name, door_open.pin);
    //printk("Address of turn_packet%p\n", &turn_packet);
    init_data_packet();

    ret = u_uart_init();

    ret = u_adc_init();

    ret = u_led_init();

    // if (!device_is_ready(uart)){
	// 	printk("UART device not ready\r\n");
	// 	//return 1 ;
	// }

	// ret = uart_callback_set(uart, uart_cb, NULL);
	// if (ret) {
    //     printk("UART callback not set\r\n%i", ret);
	// 	//return 1;
	// }
    // ret = uart_rx_enable(uart ,rx_buf,sizeof rx_buf,RECEIVE_TIMEOUT);
	// if (ret) {
    //     printk("UART rx not enabled\r\n%i", ret);
	// 	//return 1;
	// }

    //app_event_send((app_event_t)event0); // Send request for connection

    //k_msleep(10000);

    if (!device_is_ready(pwm_led0.dev)) {
		printk("Error: PWM device %s is not ready\n",
		       pwm_led0.dev->name);
	}

    //write_power(20);

    uint8_t brt_level;
    
    int i;

	while(1)
    {
		if(flag_50ms & cup_in)
        {
            
            if(i%4 == 0)
            {
                color_wipe( 0, 0, 0, 0, 100);   
            }
            else
            {
                pepsi();                   
            }
                
            
            i++;
            if(i>25)
            {
                i = 0;
                cup_in = false;
                color_wipe( 0, 0, 0, 0, 100); 
            }
            // brt_level = (uint8_t)(u_adc_bin_level()/15)-17;
            // //color_wipe(0,0,0,brt_level,100);
            // printk("Bin level %i  %i\n",u_adc_bin_level(), (brt_level));
            flag_50ms = false;
            //cup_in = false;
        }
        else if(door_state_change)
        {
            
            //updata_msg_packet((turn_data_packet*)packet);
            if(door_is_open)
            {
                turn_packet.event_id = 1;
            } else
            {
                turn_packet.event_id = 2;
            }
            //memcpy(&rx_buf[19],turn_packet.data,13);
            //app_event_send((app_event_t)event1);

            door_state_change = false;
            printk("led toggle \n");            
	    } 
        else if(uhf_data_received)
        {
            printk("uhf_data_received \n");
            //printk("-----\n");
            //printk("%s\n",rx_buf);
            uhf_data_received = false;
            
            if(data_len > 4)
            {
                ret = u_uart_get_rx_buf(data_buf, data_len);

                pulse_width = MIN_PULSE_WIDTH;
                ret = pwm_set_pulse_dt(&pwm_led0, pulse_width);
                cup_in = true;
                if(ret) 
                {
                    printk("Error %d: failed to set pulse width\n", ret);
                }
                k_timer_start(&uhf_timer, K_MSEC(4000), K_NO_WAIT);
                data_len = 0;
                turn_packet.event_id = 3;
                printk("%s\n",data_buf);
                memcpy(turn_packet.data,&data_buf[19],13);
                
                //app_event_send((app_event_t)event1);
            }            

            memset(data_buf,0,256); // make all data in the buf 0

        }
        else if(flag_10s)
        {
            flag_10s = false;
            turn_packet.event_id = 4;
            memset(turn_packet.data,0,13);
            //app_event_send((app_event_t)event1);
        }

        if(!k_msgq_get(&u_uart_received_msgq, &data_len, K_NO_WAIT))
        {
            uhf_data_received = true;
            //printk("got msg queue");
        }

		k_yield();
		//printk("loop");
	}
	
}



void turn_app_thread_init(){

	(void)k_thread_create(&turn_app_thread, turn_app_thread_stack, 
					K_THREAD_STACK_SIZEOF(turn_app_thread_stack), turn_app_entry,
					NULL,NULL,NULL,TURN_THREAD_PRIORITY,0, K_MSEC(TURN_THREAD_START_DELAY_MS));

	k_thread_name_set(&turn_app_thread, "turn_application_thread");
}


// End Thread Section

void u_app_turn_init()
{
    system_timer_set_and_run(K_MSEC(500));
    turn_app_thread_init();
    init_data_packet();

    printk("u_app_turn_init \n");
    
}



#endif // CONFIG_TURN_APP