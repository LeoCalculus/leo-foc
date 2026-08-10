//
// Created by leoxu on 8/9/26.
//

#include "../Inc/ws2812.h"

#include <string.h>

ColorInfo WS2812RGB;
WS2812_BITS WS2812BINARY;
ColorInfo WS2812SPIBUFFER[WS2812_RESET_PERIOD + NUMBER_OF_LEDS + WS2812_RESET_PERIOD];

void WS2812_RESETBUFFER() {
    memset(&WS2812SPIBUFFER, 0, sizeof(WS2812SPIBUFFER));
}

ColorInfo WS2812_GETRGB(uint8_t r, uint8_t g, uint8_t b) {
    ColorInfo ColorData = {0};
    uint8_t ColorValues[3] = {g, r, b};
    uint8_t *ColorOutputs[3] = {ColorData.g, ColorData.r, ColorData.b};

    for (int color = 0; color < 3; color++) {
        uint64_t PackedColor = 0;
        uint8_t EncodedColor[7]; // 56 bits can be reinterpreted as 7 bytes

        // for each color pack all 56 bits information, determine each grb binary should be 1 or 0 in LED language
        for (int bit = 0; bit < 8; bit++) {
            uint8_t WS2812Bit = (ColorValues[color] & (0x80u >> bit) ? WS2812BINARY.BIT_1 : WS2812BINARY.BIT_0);
            PackedColor = (PackedColor << 7) | WS2812Bit; // move old first and use | to get new
        } // enc_g1 | enc_g2 ... | enc_g8 total 56 bit, each section is 7 bit

        // reinterpret as bytes
        for (int byte = 0; byte < 7; byte++) {
            EncodedColor[byte] = (uint8_t)(PackedColor >> (48 - 8 * byte));
        }

        memcpy(ColorOutputs[color], EncodedColor, sizeof(EncodedColor));
    }

    return ColorData;
}

void WS2812_SETPURE(uint8_t r, uint8_t g, uint8_t b) {
    WS2812_RESETBUFFER();
    for (int i = 0; i < NUMBER_OF_LEDS; i++) {
        WS2812SPIBUFFER[WS2812_RESET_PERIOD+i] = WS2812_GETRGB(r,g,b);
    }
}

void WS2812_REFRESH() {
    HAL_SPI_Transmit_DMA(WS2812_HANDLER, (uint8_t*)WS2812SPIBUFFER, sizeof(WS2812SPIBUFFER));
}
