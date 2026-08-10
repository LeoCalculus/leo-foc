//
// Created by leoxu on 8/9/26.
//
#ifndef LEO_FOC_WS2812_H
#define LEO_FOC_WS2812_H

#include <spi.h>
#include <stdbool.h>

// reference to Li Wei ws2812 driver, but here using 7 bit for higher baud rate
#define WS2812_RESET_PERIOD 9
#define WS2812_BRIGHTNESS 1 // this is percentage
#define NUMBER_OF_LEDS 1
#define WS2812_HANDLER &hspi3
// define for bit 1 and bit 0 that recognize by led @ bd rate = 5.3125 MHz
// bit 0: 110_0000 -> 0x60
// bit 1: 111_1000 -> 0x78

typedef struct WS2812_BITS {
    uint8_t BIT_1 : 7;
    uint8_t BIT_0 : 7;
} WS2812_BITS;

// real ws2812 message has 8 bit green + 8 bit red + 8 bit blue
// each bit will be encoded to 7 bit
typedef struct ColorInfo {
    uint8_t g[7];
    uint8_t r[7];
    uint8_t b[7];
} ColorInfo;

// process should be: e.g. green is 0x88 which is 1000_1000
// loop 8 times, first loop: (1000_1000) & (1000_0000) if true => replace

extern ColorInfo WS2812RGB;
extern WS2812_BITS WS2812BINARY;
extern ColorInfo WS2812SPIBUFFER[WS2812_RESET_PERIOD + NUMBER_OF_LEDS + WS2812_RESET_PERIOD];

void WS2812_RESETBUFFER();
ColorInfo WS2812_GETRGB(uint8_t r, uint8_t g, uint8_t b);
void WS2812_SETPURE(uint8_t r, uint8_t g, uint8_t b);
void WS2812_REFRESH();


#endif //LEO_FOC_WS2812_H
