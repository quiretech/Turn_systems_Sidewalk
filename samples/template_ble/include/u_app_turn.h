#ifndef U_APP_TURN_H
#define U_APP_TURN_H

#define BATT_LOW_LEVEL      57      //Battery too low to deploy
#define BATT_25             59      //Low battery      
#define BUZZER_PIN_2        25      //Buzzer pin
#define FULL_BIN            15      //Bin full thresold
#define FREQ_IN_US          425     //pwm frequency for buzzer
#define PWM_DUTY            90      //pwm duty for buzzer
#define ADC_TO_CM(x)      (2100/x) - 3.21  //(183 - x)/3.59
#define ADC_TO_VOLT(x)    (x/2.2)-2
#define BIN_ID            "T3JMMV"
#define EN_5V               21
#define SOL_1               15
#define SOL_2               26
#define IR_SEN_EN           17

#define LID_OPEN_PIN        22
#define SERVO_PIN           40
#define PIN_IN              14
#define PROXIMITY           19

#define RECEIVE_BUFF_SIZE   256
//#define RECEIVE_TIMEOUT     2000

typedef struct turn_data{
    uint8_t header[2];
    uint8_t data_id[3];
    uint8_t time[4];
    uint8_t bin_id[6];
    uint8_t event_id;
    char data[13];
    uint8_t crc[2];
    uint8_t footer;
}turn_data_packet;




/**
 * @brief Set time period and run timer for capability notification.
 *
 * @param delay expected delay.
 */
void system_timer_set_and_run(k_timeout_t delay);

/**
 * @brief System timer stop.
 *
 */
void system_timer_stop(void);

void turn_app_thread_init(void);

void init_data_packet(void);

void u_app_turn_init(void);

turn_data_packet* get_data_packket_pointer(void);

#endif //U_APP_TURN_H