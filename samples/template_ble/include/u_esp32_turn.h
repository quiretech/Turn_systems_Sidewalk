#ifndef U_ESP32_TURN_H
#define U_ESP32_TURN_H

#include <zephyr/zephyr.h>

uint8_t init_esp32(void);
uint8_t time_request_esp32(uint8_t *aws_time);



#endif //U_ESP32_TURN_H