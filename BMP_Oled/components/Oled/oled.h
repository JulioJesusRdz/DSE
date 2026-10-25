#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void oled_init(void);
void oled_clear(void);
void oled_write_text(uint8_t page, uint8_t column, const char *text);
void update_oled(float temperature, float pressure);

#ifdef __cplusplus
}
#endif