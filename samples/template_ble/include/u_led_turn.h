#ifndef U_LED_TURN_H
#define U_LED_TURN_H

#include <stdint.h>
#include <stdbool.h>

int u_led_init(void);
void position_on_range(uint8_t pos1, uint8_t pos2, uint8_t r, uint8_t g, uint8_t b, uint8_t w, bool keep_state);
void position_on(uint16_t pos,uint8_t r, uint8_t g, uint8_t b, uint8_t w, bool keep_state);
void all_on(uint8_t r, uint8_t g, uint8_t b, uint8_t w);
void color_wipe(uint8_t r, uint8_t g, uint8_t b, uint8_t w, uint32_t ms_delay);
void pepsi(void);

#endif // U_LED_TURN_H