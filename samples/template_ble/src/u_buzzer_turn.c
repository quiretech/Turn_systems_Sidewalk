/*
 * Copyright (c) 2023 Quiretech
 *
 * 
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
//---------------------------
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <inttypes.h>
#include <zephyr/drivers/pwm.h>

#include <u_buzzer_turn.h>

static const struct pwm_dt_spec pwm_led1 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led1));


//Buzzer

#define BUZZER_FREQUENCY_HZ 2400
#define BUZZER_PERIOD_NS    (1000000/BUZZER_FREQUENCY_HZ)*1000
#define BUZZER_DUTY_100     50         
#define BUZZER_DUTY_NS      (BUZZER_PERIOD_NS * BUZZER_DUTY_100)/100
#define BUZZER_DUTY_OFF_NS  0

static void buzzer_off_timer_cb(struct k_timer *timer_id);
K_TIMER_DEFINE(buzzer_off_timer, buzzer_off_timer_cb, NULL);

static void buzzer_on_timer_cb(struct k_timer *timer_id);
K_TIMER_DEFINE(buzzer_on_timer, buzzer_on_timer_cb, NULL);

static uint8_t  repeat_count;
static uint16_t on_duration = 500;       // mili seconds
static uint16_t off_duration = 500;
//static uint8_t  duty_count = 5;    // 1 to 10 ( 10 -100 %)


static void buzzer_off_timer_cb(struct k_timer *timer_id)
{
    ARG_UNUSED(timer_id);
    int ret;
    buzzer_off_pwm();

    repeat_count--;

    if(repeat_count > 0){
        k_timer_start(&buzzer_on_timer, K_MSEC(off_duration) , K_NO_WAIT); // call to turn on after off_duration
    }
    
    //printk("Buzzer OFF.\n");
}

static void buzzer_on_timer_cb(struct k_timer *timer_id)
{
    ARG_UNUSED(timer_id);
    int ret;
    buzzer_on_pwm();

    //if(repeat_count > 0){
        k_timer_start(&buzzer_off_timer, K_MSEC(on_duration) , K_NO_WAIT); // call to turn off after on_duration
    //}

    //printk("Buzzer ON.\n");
}


void u_init_buzzer(){

    if (!device_is_ready(pwm_led1.dev)) {
		printk("Error: PWM device %s is not ready\n",
		       pwm_led1.dev->name);
	}
    else{
        printk("PWM device %s not ready\n",
		       pwm_led1.dev->name);
    }
}

void buzzer_on_pwm(){
    int ret;        
    ret = pwm_set_pulse_dt(&pwm_led1, (uint32_t)BUZZER_DUTY_NS);
    //ret = pwm_set_dt(&pwm_led1, (uint32_t)BUZZER_PERIOD_NS, (uint32_t)BUZZER_DUTY_NS);    
    if(ret){
        printk("Error %d: failed to set buzzer on pulse width\n", ret);
    }
}

void buzzer_off_pwm(){
    int ret;        
    ret = pwm_set_pulse_dt(&pwm_led1, (uint32_t)BUZZER_DUTY_OFF_NS);
    //ret = pwm_set_dt(&pwm_led1, (uint32_t)BUZZER_PERIOD_NS, (uint32_t)BUZZER_DUTY_OFF_NS);
    if(ret){
        printk("Error %d: failed to set buzzer off pulse width\n", ret);
    }
}

void buzzer_beep(uint8_t count, uint16_t interval, uint8_t duty){

    if(repeat_count <= 0 ){         // check if previous beeping is still running

        repeat_count    = count;
        on_duration       = (interval/10) * (duty/10);
        off_duration      = (interval/10) * ((100 - duty)/10);   

        buzzer_on_pwm();
        k_timer_start(&buzzer_off_timer, K_MSEC(on_duration) , K_NO_WAIT);  // call to turn off after on_duration      
    }

}