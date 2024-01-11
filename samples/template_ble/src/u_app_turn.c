/*
 * Copyright (c) 2023 Quiretech
 *
 * 
 */

#if defined(CONFIG_TURN_APP)

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
#include <u_spi_turn.h>
#include <u_i2c_turn.h>
#include <u_esp32_turn.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <inttypes.h>
#include <zephyr/drivers/pwm.h>

#define TURN_THREAD_PRIORITY        14
#define TURN_THREAD_START_DELAY_MS  25000

extern struct k_msgq u_uart0_received_msgq;
extern struct k_msgq u_uart1_received_msgq;

static const struct pwm_dt_spec pwm_led0 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0));
static const struct pwm_dt_spec buzzer_on = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led1));

#define NUM_STEPS	50U
#define SLEEP_MSEC	25U
//#define MAX_PULSE_WIDTH   2500000 // In nsec
#define MAX_PULSE_WIDTH     500000 // In nsec
//#define MIN_PULSE_WIDTH   500000	// In nsec
#define MIN_PULSE_WIDTH     2500000	// In nsec
#define MID_PULSE_WIDTH     1500000 // In nsec
#define STEP 		        10000	// In nsec

#define LID_JAM_CLOSE_DURATION  3000    //in ms, close the lid after this time
#define LID_JAM_OPEN_DURATION   6000    //in ms, open the lid after this time
#define LID_CLOSE_UHF_DURATION  4000    //in ms, close lid after NFC tag detection
#define LID_CLOSE_NFC_DURATION  4000    //in ms, close lid after NFC tag detection
#define UHF_TIMEOUT_DURATION    120000  //in ms, disable ufh after this time

#define OFF                     0
#define ON                      1


//Buzzer

#define BUZZER_FREQUENCY_HZ 2700
#define BUZZER_PERIOD_NS    (1000000/BUZZER_FREQUENCY_HZ)*1000
#define BUZZER_DUTY_100     50         
#define BUZZER_DUTY_NS      (BUZZER_PERIOD_NS * BUZZER_DUTY_100)/100
/*
 * Get button configuration from the devicetree doww_open alias. This is mandatory.
 */
#define DOOR_OPEN	    DT_ALIAS(dooropen)
//#define BUZZER_ON   DT_ALIAS(b1en)
#define UNLOCK_DOOR     DT_ALIAS(sol1en)
#define SERVO_POWER_ON  DT_ALIAS(sol2en)
#define LED_POWER_ON    DT_ALIAS(leden)
#define IR_SEN_POWER_ON DT_ALIAS(irsenen)
#define UHF_ENABLE      DT_ALIAS(uhfen)

#if !DT_NODE_HAS_STATUS(DOOR_OPEN, okay)
#error "Unsupported board: dooropen devicetree alias is not defined"
#endif
static const struct gpio_dt_spec door_open = GPIO_DT_SPEC_GET_OR(DOOR_OPEN, gpios, {0});
static struct gpio_callback door_open_cb_data;

//static const struct gpio_dt_spec buzzer_on      = GPIO_DT_SPEC_GET_OR(BUZZER_ON, gpios, {0});
static const struct gpio_dt_spec unlock_door    = GPIO_DT_SPEC_GET_OR(UNLOCK_DOOR, gpios, {0});
static const struct gpio_dt_spec servo_power_on = GPIO_DT_SPEC_GET_OR(SERVO_POWER_ON, gpios, {0});
static const struct gpio_dt_spec led_power_on   = GPIO_DT_SPEC_GET_OR(LED_POWER_ON, gpios, {0});
static const struct gpio_dt_spec ir_sen_power_on = GPIO_DT_SPEC_GET_OR(IR_SEN_POWER_ON, gpios, {0});
static const struct gpio_dt_spec uhf_enable     = GPIO_DT_SPEC_GET_OR(UHF_ENABLE, gpios, {0});

// UART
// const struct device *uart= DEVICE_DT_GET(DT_NODELABEL(uart0));
// static uint8_t tx_buf[] =  {0x0a,0x55,0x0d};//{"Test String\n\r"};
// static uint8_t uhf_sample[] =  {"UU3000E280689000004006D0B55C1125E711"};//{"Test String\n\r"};
// static uint8_t rx_buf[RECEIVE_BUFF_SIZE] = {0};

#define TURN_THREAD_STACK_SIZE 1024

//Local Definitions
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
#define SYSTIMER_START_DLY  M_500_MSEC        //This adds a delay to system timer starting
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
bool          flag_100ms  = false;      //100msflag
bool          flag_200ms = false;     //200ms flag
bool          flag_10s = false;     //200ms flag
bool          door_is_open = false;
bool          door_state_change = false;
bool          uhf_data_received = false;
bool          nfc_data_received = false;
bool          lid_jam_timer_expiered = false;
uint16_t      uhf_pacaket_len;
uint8_t       counter_uhf = 0; 
static uint8_t nfc_data_buf[32] = {0};
static uint8_t uhf_data_buf[32] = {0};
static uint16_t nfc_data_len;
static uint16_t uhf_data_len;
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
bool            cup_in = false;

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
  Batt_low,
  Batt_25p,
  Bin_full,
  Lid_jam,
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
// struct {
//     uint8_t header[2];
//     uint8_t data_id[3];
//     uint8_t time[4];
//     uint8_t bin_id[6];
//     uint8_t event_id;
//     uint8_t data[13];
//     uint8_t crc[2];
//     uint8_t footer;
// } mem_data;

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
  bool system_status_check; // periodic system check flag
  bool bin_full; // bin full indicator
  bool bin_75P_full;
  bool uhf_timeout; // used when lid jam occurs
  bool uhf_mem_write; // used to write uhf data to eeprom
  bool batt_low;
  bool batt_25P;
  bool sys_off;
  bool deposit_cup;
  bool display_wipe;
  bool display_thanks;

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





// Initialize system data with default values
void init_system_data(void){

    init_data_packet();     // Data packet for tranmission to cloud

}

/* Function to convert 2 byte ascii characters to 1 byte hex character  */
/* for example 0x45('E') 0x32('2') gets stored as 0xE2 in hex data      */

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


void deposit_cup_display()
{
    static int i = 0;
    static int line = 0;
    static int clr, red, grn, blu, wht = 100;

    if(flag_100ms)
    {
        display_power(ON);

        i = 0;
        flag_100ms = false;
        position_on_range((line*36), (line*36)+35, red, grn, blu, wht, 0); 
        line++;
        
        if(line >= 5)
        {
            if     (red == 100)     {red = 0; grn = 100; blu = 0; wht = 0;}
            else if(grn == 100)     {red = 0; grn = 0; blu = 100; wht = 0;}
            else if(blu == 100)     {red = 0; grn = 0; blu = 0; wht = 100;}
            else if(wht == 100)     {red = 100; grn = 0; blu = 0; wht = 0;}

            line = 0;
        }
    }
    // else if(!this.lid_open == false & i < 200)
    // {
    //     i++;
    //     display_thanks();
    //     //gpio_pin_set_dt(&servo_power_on, 1);
    //     //gpio_pin_set_dt(&unlock_door, 1);
    // }

}

//This function is called to close the lid.
void lid_open_pwm()
{

    int ret;
        
    ret = pwm_set_pulse_dt(&pwm_led0, (uint32_t)MIN_PULSE_WIDTH);
    if(ret) 
    {
        printk("Error %d: failed to set pulse width\n", ret);
    }
  
}


//This function is called to close the lid.
void lid_close_pwm()
{

    int ret;
    ret = pwm_set_pulse_dt(&pwm_led0, (uint32_t)MAX_PULSE_WIDTH);
    if(ret) 
    {
        printk("Error %d: failed to set pulse width\n", ret);
    }
  
}

void uhf_power(bool power)
{
    gpio_pin_set_dt(&uhf_enable, power);
}

void nfc_power(bool power)
{
    // send off or on message to nfc board
}

void lid_motor_power(bool power)
{
    gpio_pin_set_dt(&servo_power_on, power);
}

void ir_sensor_power(bool power)
{
    gpio_pin_set_dt(&ir_sen_power_on, power);
}

void display_power(bool power)
{
    gpio_pin_set_dt(&led_power_on, power);
}

void door_unlock(bool power)
{
    gpio_pin_set_dt(&unlock_door, power);
}

// Door section
void door_open_function(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{

    int i;

    door_state_change = true;
    data_id_counter++;

    if(gpio_pin_get_dt(&door_open)){
        door_is_open = true;
        this.lid_open = true;    
    } 
    else
    {
        door_is_open = false;
        this.lid_open = false;
    }

}
// End Door section


// Timer section

static void uhf_timer_cb(struct k_timer *timer_id);
K_TIMER_DEFINE(uhf_timer, uhf_timer_cb, NULL);

static void system_timer_cb(struct k_timer *timer_id);
K_TIMER_DEFINE(system_timer, system_timer_cb, NULL);

static void status_check_timer_cb(struct k_timer *timer_id);
K_TIMER_DEFINE(status_check_timer, status_check_timer_cb, NULL);

static void lid_jam_timer_cb(struct k_timer *timer_id);
K_TIMER_DEFINE(lid_jam_timer, lid_jam_timer_cb, NULL);

static void lid_close_timer_cb(struct k_timer *timer_id);
K_TIMER_DEFINE(lid_close_timer, lid_close_timer_cb, NULL);

static void uhf_off_timer_cb(struct k_timer *timer_id);
K_TIMER_DEFINE(uhf_off_timer, uhf_off_timer_cb, NULL);

static void display_off_timer_cb(struct k_timer *timer_id);
K_TIMER_DEFINE(display_off_timer, display_off_timer_cb, NULL);


static void uhf_timer_cb(struct k_timer *timer_id)
{
    ARG_UNUSED(timer_id);
    int ret;

    this.deposit_cup = false;
    this.display_thanks = true;

    lid_close_pwm();

}

static void system_timer_cb(struct k_timer *timer_id)
{
	ARG_UNUSED(timer_id);
    int ret;
    timer_counter++;
    static int  uhf_counter  = 0;
    static int  led_counter  = 0;

    flag_50ms = true;

    if(timer_counter == 200)
    {        
        timer_counter = 0;
        //flag_200ms = true;
        flag_10s = true;
    }

    uhf_counter++;
    if(uhf_counter > 20)
    {
        uhf_counter = 0;
        ret = uhf_single_poll();    
    }

    led_counter++;
    if(led_counter > 2)
    {
        led_counter = 0;
        flag_100ms = true;
    }
}

static void status_check_timer_cb(struct k_timer *timer_id)
{
    ARG_UNUSED(timer_id);
    this.system_status_check = true;     
}

static void lid_jam_timer_cb(struct k_timer *timer_id)
{
    ARG_UNUSED(timer_id);
    int ret;
    lid_jam_retry++;
    lid_jam_timer_expiered = true;
    // to_do Open Solenoid 2

    // open lide to release the jam
    lid_open_pwm();
    //printk("lid jam Open lid called\n");

    // timer to close lid in few seconds
    //k_timer_start(&lid_close_timer, K_MSEC(LID_JAM_CLOSE_DURATION), K_NO_WAIT);
    //k_timer_start(&lid_jam_timer, K_MSEC(LID_JAM_OPEN_DURATION), K_NO_WAIT);    
}

static void lid_close_timer_cb(struct k_timer *timer_id)
{
    ARG_UNUSED(timer_id);
    int ret;

    // Close lid
    lid_close_pwm();

}

static void uhf_off_timer_cb(struct k_timer *timer_id)
{
    ARG_UNUSED(timer_id);
    int ret;

    // turn off UHF 
    uhf_power(OFF);
    printk("uhf OFF called\n");


}

static void display_off_timer_cb(struct k_timer *timer_id)
{
    ARG_UNUSED(timer_id);
    int ret;

    //printk("Display OFF.\n");
    display_power(OFF);
    

}

void system_timer_set_and_run(k_timeout_t delay)
{
	k_timer_start(&system_timer, delay, K_MSEC(50));
    k_timer_start(&status_check_timer, K_MSEC(5000) , K_MSEC(1000));
}

void system_timer_stop()
{
    k_timer_stop(&system_timer);
    k_timer_stop(&status_check_timer);
}

// Timer section end

// This section implements the system operational logic

void check_system_status()
{
    int32_t bin_value;
    int32_t batt_value;
    uint16_t ret;
    uint8_t esp32_time[8];

    bin_value  = u_adc_bin_level();
    batt_value = u_adc_batt_volt();

    // printk("%i\n",bin_value);
    // printk("%i\n",batt_value);
    // printk("----\n");

    // ret = time_request_esp32(esp32_time);

    // if(ret == 0)
    // {
    //     printk("ESP32 time %x%x%x%x%x%x%x%x\n",esp32_time[0],esp32_time[1], esp32_time[2], esp32_time[3], esp32_time[4],esp32_time[5],esp32_time[6], esp32_time[6]);
    // }
    // else{
    //     printk("ESP32 time error\n");
    // }

    if(bin_value < BIN_FULL_75P_IN)
    {
        //printk("BIN_NOT_FULL\n");
        this.bin_75P_full = false;
        this.bin_full = false;
    }
    else if((bin_value < BIN_FULL_IN))
    {
        //printk("BIN_FULL_75_IN\n");
        this.bin_75P_full = true;
        this.bin_full = false;
    }
    else if(bin_value > BIN_FULL_IN)
    {
        //printk("BIN_FULL_IN\n");
        this.bin_75P_full = true;
        this.bin_full = true;
    }

    if(batt_value > BATT_25P_V)
    {
        this.batt_25P = false;
        this.batt_low = false;
        //printk("BATT_GOOD\n");
    }
    else if(batt_value > BATT_LOW_LEVEL_V)
    {
        this.batt_25P = true;
        this.batt_low = false;
        //printk("BATT_25P\n");
    }
    else if(batt_value < BATT_LOW_LEVEL_V)
    {
        this.batt_25P = true;
        this.batt_low = true;
        //printk("BATT_LOW\n");
    }
    
}

//checking if the status of nfc has changed since the last read, used to avoid multiple writes of the same nfc tag read at a time
bool nfc_check()
{
  if(this.uid_read_prev == this.uid_read)
  {
    return false;
  }
  else
  {
    return true;
  }
}

//led indication for nfc
void nfc_led()
{

// add code to display NFC read

}

/*
 * This function is called when lid jam is detected. It is used to turn off uhf after a timeout as uhf is always on when lid is open
 * Also to use nfc when uhf times out.
*/
// void lid_jam_function()
// {
//     // check is lid is still open
//     if(this.lid_open == true)
//     {
//         this.lid_jam = false;
//     }
//     else // if still jam
//     {
//         display_lid_jam();

//         // system checks the current state and then decides what to do based on that
//         switch (this.current_state)
//         {
//             case NFC:
//                 //nfc_check is used to avoid duplication. Since the nfc will read the same tag mulitple times
//                 //as it reads the tag every 100ms.
//                 //Here nfc_check returns true as there is no new data and hence it is different from the previous data
//                 if(nfc_check() == true)
//                 {
//                     clear_led_buzzer(); 
//                     this.current_state = UHF;
//                     if(this.uid_read == true)
//                     {
//                         //this.current_state = Mem_write;
//                         nfc_led();
//                     }
//                 }
//                 this.uid_read_prev = this.uid_read;
//                 break;

//             case UHF:
//                 //Lid jam handler timer is started this handles the uhf timeout to turn off uhf and start nfc.
//                 //It sets the uhf_timeout flag which makes the current state as NFC
                
//                 k_timer_start(&uhf_off_timer, K_NO_WAIT, K_MSEC(UHF_TIMEOUT_DURATION));
                

//                 // if(this.uid_read == true && nfc_check())// && !this.ble_on && (this.cup_no == 0))
//                 // {
//                 //     this.mem_write = true;
//                 //     if(!this.ble_on && (this.cup_no == 0))
//                 //     {
//                 //     //Uncomment when ble on has to be changed
//                 //         //ble_led();
//                 //         advertising_start();
//                 //         this.ble_on = true;
//                 //     }
//                 //     else if(this.cup_no > 0)
//                 //     {
//                 //         this.cup_no = 0;
//                 //         this.uhf_mem_write = true;
//                 //     }
//                 // }
//                 // this.uid_read_prev = this.uid_read;


//         case Mem_write:
//             //  if(this.uhf_mem_write)
//             //  {
//             //    this.current_state = Mem_write_UHF;
//             //    this.uhf_mem_write = false;
//             //  }
//             //  else if(this.uhf_timeout)
//             //  {
//             //    this.current_state = NFC;
//             //  }
//             //  else
//             //  {
//             //    this.current_state = UHF;
//             //  }
//              break;
//         case Mem_write_UHF:
//             //  uhf_led();
//             //  this.current_state = Sys_Check;
//              break;
//         case Sys_Check:
//             // if(this.system_check == false && !this.periodic_sys_check)
//             // {
//             //    clear_led_buzzer();
//             //    if(this.adc_batt < BATT_25)
//             //    {
//             //      app_timer_start(m_low_batt_timer,APP_TIMER_TICKS(M_5_SEC),NULL);
//             //    }
//             //    if(this.adc_bin < FULL_BIN)
//             //    {
//             //       this.bin_full = true;
//             //       make_packet(BIN_FULL,&this.bin_full,1);
//             //       this.mem_write = true;
//             //    }
//             //    else
//             //    {
//             //      this.bin_full = false;
//             //    }
//             //    if(this.uhf_timeout)
//             //    {
//             //      this.current_state = NFC;
//             //    }
//             //    else
//             //    {
//             //      this.current_state = UHF;
//             //    }
//             // }
//             break;
//         }
//     //   if(this.mem_write)
//     //   {
//     //      this.current_state = Mem_write;
//     //      this.mem_write = false;
//     //   }
//       state_machine();

//         }
//     }

// }

void state_machine()
{
    uint8_t da[32];
    int ret;

    #if defined(CONFIG_SIDEWALK)
    static uint32_t event0 = BUTTON_EVENT_CONNECTION_REQUEST;
    static uint32_t event1 = BUTTON_EVENT_SEND_HELLO;
    #endif // endif defined(CONFIG_SIDEWALK)

    int i;

    //printk("State Machine called");
    //printk("%i\n",this.current_state);

    switch(this.current_state)
    {
        case Idle:

            //printk("State Idle.\n");

            if(nfc_data_received)
            {
                nfc_data_received = false;
                this.current_state = NFC;
                //printk("nfc_data_received.\n");                                
            }
            else if(uhf_data_received)
            {
                uhf_data_received = false;
                this.current_state = UHF;
                //printk("uhf_data_received.\n");
            }
            break;
        
        case Bin_full:

            //display_bin_full;

            //printk("State Bin full.\n");

            check_system_status();
            //this.current_state = Idle;

            if(nfc_data_received)
            {
                nfc_data_received = false;
                this.current_state = NFC;                                
            }
            else if(uhf_data_received)
            {
                uhf_data_received = false;
                this.current_state = UHF;
            }

            //this.system_status_check = true;
            

            break;

        case NFC:

            //printk("State NFC.\n");

            ret = u_uart1_get_rx_buf(nfc_data_buf, nfc_data_len);
            
            if(this.lid_jam == false & this.bin_full == false) // try opening lid only if its not jammed
            {
                lid_open_pwm();
            } 
            this.deposit_cup = true;
            
            k_timer_start(&uhf_timer, K_MSEC(LID_CLOSE_NFC_DURATION), K_NO_WAIT);
            nfc_data_len = 0;
            turn_packet.event_id = NFC_EVE;
            printk("NFC id = %s\n",nfc_data_buf);
            memcpy(turn_packet.data,&nfc_data_buf[1],13);
            
            //app_event_send((app_event_t)event1);

            this.current_state = Idle;

            break;

        case UHF:

            //printk("State UHF.\n");
            
            ret = u_uart0_get_rx_buf(uhf_data_buf, uhf_data_len); // Get data from UHF UART buffer

            if(this.lid_jam == false & this.bin_full == false) // try opening lid only if its not jammed
            {
                lid_open_pwm();
            }
            
            this.deposit_cup = true;

            // Add buzzer noise
            
            k_timer_start(&uhf_timer, K_MSEC(LID_CLOSE_UHF_DURATION), K_NO_WAIT);

            uhf_data_len = 0;
            turn_packet.event_id = UHF_EVE;
            printk("UHF id = %s\n",uhf_data_buf);
            memcpy(turn_packet.data,&uhf_data_buf[19],13);

            //cup_in = true;
            
            //app_event_send((app_event_t)event1);

            this.current_state = Idle;
            break;

        case Mem_write:

            break;

        case Mem_write_UHF:

            break;

        case Mem_read:

            break;

        case Sys_Check:

            check_system_status();
            this.current_state = Idle;
            //k_timer_start(&status_check_timer, K_MSEC(10000) , K_NO_WAIT);

            break;

        case Batt_low:
            //standby_mode();
            //k_timer_start(&status_check_timer, K_MSEC(1000) , K_NO_WAIT);
            //this.system_status_check = true;
            check_system_status();
            this.current_state = Idle;
            break;

        case Lid_jam:

            lid_open_pwm();             // Leave lid in open state.

            printk("State Lid Jam.\n");

            if(nfc_data_received)
            {
                nfc_data_received = false;
                this.current_state = NFC;                                
            }
            else if(uhf_data_received)
            {
                uhf_data_received = false;
                this.current_state = UHF;
            }
            break;

        default:
            break;
    }
    //printk("%i\n", this.current_state);
}


void logic_machine()
{
    #if defined(CONFIG_SIDEWALK)
    static uint32_t event0 = BUTTON_EVENT_CONNECTION_REQUEST;
    static uint32_t event1 = BUTTON_EVENT_SEND_HELLO;
    #endif // endif defined(CONFIG_SIDEWALK)

    int i;

    // this.bin_full = false;
    // this.batt_low = false;
    // this.lid_jam = false;
    // this.batt_25P = false;

    //printk("Logic called");
    //printk("%i\n", this.current_state);



    if(this.batt_low == false) // Battery good to operate
    {
        if(this.batt_25P == true)
        {
            //display_low_batt();
            //printk("Batt_25P\n");
        }
        else
        {
            if(this.deposit_cup)
            {
                deposit_cup_display();
            }
            else
            {
                if(this.display_thanks)
                {
                    display_thanks();
                    this.display_thanks = false;
                    //this.display_wipe = true;
                    k_timer_start(&display_off_timer, K_MSEC(5000) , K_NO_WAIT);
                }
                
                if(this.display_wipe)
                {
                    color_wipe(0,0,0,0,0);
                    this.display_wipe = false;
                }
                
                //k_timer_start(&display_off_timer, K_MSEC(500) , K_NO_WAIT);
            }
        }

        if(this.bin_full == false) // Bin not full yet
        {                           
            //this.current_state = Idle;
            if(this.lid_open == true) // If lid is open
            {               
                //printk("Lid open");    

                if(door_state_change)
                {
                    door_state_change = false;
                    //printk("Lid open\n");
                                        
                    turn_packet.event_id = LID_EVE;
                    turn_packet.data[0] = this.lid_open;

                    for(i = 0; i < 3; i++)
                    {
                        turn_packet.data_id[2-i] = (uint8_t)(data_id_counter >> (i*8) & 0x000000ff);
                    }
                    
                    app_event_send((app_event_t)event1); // Send data to sidewalk

                    // This section check of the lid is jam due to mechincal malfunction of user error
                    k_timer_start(&lid_jam_timer, K_MSEC(LID_JAM_OPEN_DURATION), K_NO_WAIT);

                }

                // This section check of the lid is jam due to mechincal malfunction of user error
                if(lid_jam_timer_expiered == true)
                {
                    lid_jam_timer_expiered = false;
                    if(lid_jam_retry >= LID_JAM_RETRY) // check if lid close is tried for multiple times 
                    {
                        this.current_state = Lid_jam;
                        this.lid_jam = true;
                    }
                    else
                    {
                        k_timer_start(&lid_jam_timer, K_MSEC(LID_JAM_OPEN_DURATION), K_NO_WAIT);
                        k_timer_start(&lid_close_timer, K_MSEC(LID_JAM_CLOSE_DURATION), K_NO_WAIT);
                    }
                    printk("lid open tries %i\n",lid_jam_retry);
                }

                
            }
            else // if lid is close
            {
                //printk("lid closed");

                //this.current_state = Idle;

                
                this.lid_jam = false;
                
                if(door_state_change)
                {
                    door_state_change = false;

                    //printk("lid closed\n");
                    lid_jam_retry = 0; //   Reset count for detecting lid jam as lid is close
                    k_timer_stop(&lid_jam_timer); // stop the lid jam detection timer when lid closed

                    turn_packet.event_id = LID_EVE;
                    turn_packet.data[0] = this.lid_open;

                    for(i = 0; i < 3; i++)
                    {
                        turn_packet.data_id[2-i] = (uint8_t)(data_id_counter >> (i*8) & 0x000000ff);
                    }
                    
                    app_event_send((app_event_t)event1); // Send data to sidewalk
                }                
            }
        }
        else // If bin is full
        {
            //printk("%i\n", this.current_state);
            if(!(this.current_state == UHF)  & !(this.current_state == NFC))
            {
                //printk("*inside %i\n", this.current_state);
                this.current_state = Bin_full;                
            }
            printk("Bin full\n");       
        }
    }
    else    // if battery is below low level
    {
        this.current_state = Batt_low;
        //printk("Batt low\n");
        //this.system_status_check = true;

    }

    if(this.system_status_check)
    {
        this.current_state = Sys_Check;
        this.system_status_check = false;    
    }

    state_machine();
}






turn_data_packet* get_data_packket_pointer(void)
{
  //printk("Address of turn_packet at get packet%p\n", &turn_packet);
  return &turn_packet;
}
// End Timer Section

// Thread Section

static struct k_thread turn_app_thread;
K_THREAD_STACK_DEFINE(turn_app_thread_stack, TURN_THREAD_STACK_SIZE);

static void turn_app_entry(void *dummy0, void *dummy1, void *dummy2)
{
    printk("turn_app_entry \n");

    uint32_t pulse_width = 0U;
	uint32_t step = STEP;//pwm_led0.period / NUM_STEPS;
	uint8_t dir = 1U;
    static uint8_t data_buf[RECEIVE_BUFF_SIZE] = {0};

    static int count_10s = 0;


#if defined(CONFIG_SIDEWALK)
    static uint32_t event0 = BUTTON_EVENT_CONNECTION_REQUEST;
    static uint32_t event1 = BUTTON_EVENT_SEND_HELLO;
#endif // endif defined(CONFIG_SIDEWALK)

	ARG_UNUSED(dummy0);
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);

    int ret;

    //GPIO output


    if (!gpio_is_ready_dt(&unlock_door)) {
		printk("Error %d %d: LED device %s is not ready; ignoring it\n",
		       ret, unlock_door.pin, unlock_door.port->name);
		//unlock_door.port = NULL;
	}

    ret = gpio_pin_configure_dt(&unlock_door, GPIO_OUTPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",ret, unlock_door.port->name, unlock_door.pin);
		return 0;
	}

    //door_unlock(ON);

    if (!gpio_is_ready_dt(&servo_power_on)) {
		printk("Error %d %d: LED device %s is not ready; ignoring it\n",
		       ret, servo_power_on.pin, servo_power_on.port->name);
		//servo_power_on.port = NULL;
	}

    ret = gpio_pin_configure_dt(&servo_power_on, GPIO_OUTPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",ret, servo_power_on.port->name, servo_power_on.pin);
		return 0;
	}

    if (!gpio_is_ready_dt(&led_power_on)) {
		printk("Error %d %d: LED device %s is not ready; ignoring it\n",
		       ret, led_power_on.pin, led_power_on.port->name);
		//led_power_on.port = NULL;
	}

    ret = gpio_pin_configure_dt(&led_power_on, GPIO_OUTPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",ret, led_power_on.port->name, led_power_on.pin);
		return 0;
	}

    if (!gpio_is_ready_dt(&ir_sen_power_on)) {
		printk("Error %d %d: LED device %s is not ready; ignoring it\n",
		       ret, ir_sen_power_on.pin, ir_sen_power_on.port->name);
		//ir_sen_power_on.port = NULL;
	}

    ret = gpio_pin_configure_dt(&ir_sen_power_on, GPIO_OUTPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",ret, ir_sen_power_on.port->name, ir_sen_power_on.pin);
		return 0;
	}

    if (!gpio_is_ready_dt(&uhf_enable)) {
		printk("Error %d %d: UHF device %s is not ready; ignoring it\n",
		       ret, uhf_enable.pin, uhf_enable.port->name);
		//ir_sen_power_on.port = NULL;
	}

    ret = gpio_pin_configure_dt(&uhf_enable, GPIO_OUTPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",ret, uhf_enable.port->name, uhf_enable.pin);
		return 0;
	}


    // GPIO input
	if (!gpio_is_ready_dt(&door_open)) {
		//printk("Error: button device %s is not ready\n",door_open.port->name);
		//return 0;
	}

	ret = gpio_pin_configure_dt(&door_open, GPIO_INPUT);
	if (ret != 0) {
		//printk("Error %d: failed to configure %s pin %d\n",ret, door_open.port->name, door_open.pin);
		//return 0;
	}

	ret = gpio_pin_interrupt_configure_dt(&door_open, GPIO_INT_EDGE_BOTH);
	if (ret != 0) {
		//printk("Error %d: failed to configure interrupt on %s pin %d\n",ret, door_open.port->name, door_open.pin);
		//return 0;
	}

    gpio_init_callback(&door_open_cb_data, door_open_function, BIT(door_open.pin));
	gpio_add_callback(door_open.port, &door_open_cb_data);

    init_data_packet();

    ret = u_uart_init();

    ret = u_adc_init();

    ret = u_led_init();

    ret = i2c_ready();

    if(ret == 0)
    {
        ret = i2c_scan();
    }

    //u_spi_init();

    app_event_send((app_event_t)event0); // Send request for connection

    write_power(18);

    k_msleep(2000);

    // PWM device check

    if (!device_is_ready(pwm_led0.dev)) {
		printk("Error: PWM device %s is not ready\n",
		       pwm_led0.dev->name);
	}

    if (!device_is_ready(buzzer_on.dev)) {
		printk("Error: PWM device %s is not ready\n",
		       buzzer_on.dev->name);
	}

    uint8_t brt_level;
    

    // Init all output power control
    // gpio_pin_set_dt(&servo_power_on, 0);
    // gpio_pin_set_dt(&ir_sen_power_on, 0);
    // gpio_pin_set_dt(&unlock_door, 0);
    // gpio_pin_set_dt(&led_power_on, 0);

    lid_motor_power(ON);
    k_timer_start(&lid_jam_timer, K_MSEC(LID_JAM_OPEN_DURATION), K_NO_WAIT); // clear if anything jammed
    ir_sensor_power(ON);
    door_unlock(OFF);
    display_power(OFF);
    uhf_power(ON);

    this.current_state = Idle;

    this.bin_full = false;
    this.batt_low = false;
    this.lid_jam = false;
    this.batt_25P = false;

    if(gpio_pin_get_dt(&door_open)){
        door_is_open = true;
        this.lid_open = true;    
    } 
    else
    {
        door_is_open = false;
        this.lid_open = false;
    }
    uint8_t esp32_time[8];
    init_esp32();
    ret = time_request_esp32(esp32_time);

    if(ret == 0)
    {
        printk("ESP32 time %s\n",esp32_time);
    }
    else{
        printk("ESP32 time error\n");
    }

    
    while(1)
    {
		if(flag_50ms)
        {
            flag_50ms = false; // Reset flag for 50 ms

            if(!k_msgq_get(&u_uart0_received_msgq, &uhf_data_len, K_NO_WAIT))
            {
                uhf_data_received = true;
                //printk("got UART0 msg queue.\n");
            }
            else if(!k_msgq_get(&u_uart1_received_msgq, &nfc_data_len, K_NO_WAIT))
            {
                nfc_data_received = true;
                //printk("got UART1 msg queue.\n");
            }

            logic_machine(); 

            
        //     //brt_level = (uint8_t)(u_adc_bin_level()/15)-17;
        //     brt_level = u_adc_bin_level();
        //     //if(brt_level >= 150){ brt_level = 150;}
        //     //color_wipe(0,0,0,brt_level,100);
        //     printk("Bin level %i  %i\n",u_adc_batt_volt(), u_adc_bin_level());
        //     flag_50ms = false;
        //     //cup_in = false;
        // }
        // else if (this.bin_full)// | this.batt_low)
        // {
        //     display_pepsi();
        // }        
        // else if(door_state_change)
        // {
            
        //     //updata_msg_packet((turn_data_packet*)packet);
        //     if(door_is_open)
        //     {
        //         turn_packet.event_id = 1;
        //     } else
        //     {
        //         turn_packet.event_id = 2;
        //     }
        //     //memcpy(&rx_buf[19],turn_packet.data,13);
        //     //app_event_send((app_event_t)event1);

        //     door_state_change = false;
        //     printk("led toggle \n");            
	    // } 
        // else if(uhf_data_received)
        // {
        //     //printk("uhf_data_received \n");
        //     //printk("-----\n");
        //     //printk("%s\n",rx_buf);
        //     uhf_data_received = false;
            
        //     if(data_len > 4) //data_len is obtained for msg queue fn k_msgq_get
        //     {
        //         ret = u_uart0_get_rx_buf(data_buf, data_len);

        //         pulse_width = MIN_PULSE_WIDTH;
        //         ret = pwm_set_pulse_dt(&pwm_led0, pulse_width);
        //                         if(ret) 
        //         {
        //             printk("Error %d: failed to set pulse width\n", ret);
        //         }
        //         cup_in = true;
        //         //gpio_pin_set_dt(&buzzer_on, GPIO_OUTPUT_ACTIVE);

        //         k_timer_start(&uhf_timer, K_MSEC(4000), K_NO_WAIT);
        //         data_len = 0;
        //         turn_packet.event_id = 3;
        //         printk("UHF id = %s\n",data_buf);
        //         memcpy(turn_packet.data,&data_buf[19],13);
                
        //         app_event_send((app_event_t)event1);
        //     }            

        //     memset(data_buf,0,256); // make all data in the buf 0

        // }
        // else if(nfc_data_received)
        // {
        //     //printk("nfc_data_received \n");
        //     //printk("-----\n");
        //     //printk("%s\n",rx_buf);
        //     nfc_data_received = false;
            
        //     if(data_len > 4)  //data_len is obtained for msg queue fn k_msgq_get
        //     {
        //         ret = u_uart1_get_rx_buf(data_buf, data_len);

        //         pulse_width = MIN_PULSE_WIDTH;
        //         ret = pwm_set_pulse_dt(&pwm_led0, pulse_width);
        //         cup_in = true;
        //         if(ret) 
        //         {
        //             printk("Error %d: failed to set pulse width\n", ret);
        //         }
        //         k_timer_start(&uhf_timer, K_MSEC(4000), K_NO_WAIT);
        //         data_len = 0;
        //         turn_packet.event_id = 3;
        //         printk("NFC id = %s\n",data_buf);
        //         memcpy(turn_packet.data,&data_buf[1],13);
                
        //         app_event_send((app_event_t)event1);
        //     }            

        //     memset(data_buf,0,256); // make all data in the buf 0

        // }
        // else if(flag_10s)
        // {   
        //     flag_10s = false;

        //     color_wipe( 0, 0, 0, 0, 100);
        //     gpio_pin_set_dt(&servo_power_on, 0);
        //     gpio_pin_set_dt(&led_power_on, 0);
            
        //     static int count_10s = 0;
        //     count_10s++;

        //     if(count_10s >= 2){
        //         turn_packet.event_id = 4;
        //         memset(turn_packet.data,0,13);
        //         app_event_send((app_event_t)event1);
        //         count_10s = 0;
        //     }
        // }

        // nfc check 

        //uhf_data_received = check_new_uhf_tag(uhf_data_buf);
        //nfc_data_received = check_new_nfc_tag(nfc_data_buf);


        }

        if(flag_10s)
        {   
            flag_10s = false;
            count_10s++;
            if(count_10s >= 2)
            {
                turn_packet.event_id = 4;
                memset(turn_packet.data,0,13);
                //app_event_send((app_event_t)event1);
                count_10s = 0;
            }
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

// This app init will initialize the app system timer, app Threads and data sets
void u_app_turn_init()
{
    system_timer_set_and_run(K_MSEC(SYSTIMER_START_DLY));   // System heart beat, with delay 
    turn_app_thread_init();                                 // App thread init
    init_system_data();                                     // Initialize system data with default values

    printk("u_app_turn_init \n");    
}



#endif // CONFIG_TURN_APP