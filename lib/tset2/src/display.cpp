#include <stdio.h>

#include <pico/stdio.h>
#include <pico/time.h>

#include "display.hpp"

void Screen::send_graph(const float data[], uint16_t length) {
    printf("addt s0.id,0,%d\xff\xff\xff", length);
    sleep_ms(100);
    for (uint16_t i = 0; i < length; i++) {
        const uint8_t data_casted = static_cast<uint8_t>(
            static_cast<int>(data[i] * stretch_ + offset_) 
        );
        printf("%c", data_casted);
    }
    printf("\x01\xff\xff\xff");
}

void Screen::send_text(const char *text) {
    printf("slt0.txt=\"%s\"\xff\xff\xff", text);
}
