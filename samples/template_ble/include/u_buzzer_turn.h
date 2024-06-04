#ifndef U_BUZZER_TURN_H
#define U_BUZZER_TURN_H

#define COUNT_ONE       1
#define COUNT_TWO       2
#define COUNT_THREE     3
#define COUNT_FOUR      4
#define COUNT_FIVE      5
#define COUNT_SIX       6
#define COUNT_SEVEN     7
#define COUNT_EIGHT     8
#define COUNT_NINE      9
#define COUNT_TEN       10

#define MS_ONE_HUNDRED     100
#define MS_TWO_HUNDRED     200
#define MS_THREE_HUNDRED   300
#define MS_FOUR_HUNDRED    400
#define MS_FIVE_HUNDRED    500
#define MS_SIX_HUNDRED     600
#define MS_SEVEN_HUNDRED   700
#define MS_EIGHT_HUNDRED   900
#define MS_NINE_HUNDRED    900
#define MS_THOUSAND        1000



void u_init_buzzer();

void buzzer_on_pwm();

void buzzer_off_pwm();

void buzzer_beep(uint8_t count, uint16_t interval, uint8_t duty);



#endif //U_BUZZER_TURN_H