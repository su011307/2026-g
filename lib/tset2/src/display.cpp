#include <stdio.h>

#include <pico/stdio.h>
#include <pico/time.h>

#include "display.hpp"

void Screen::send_wave_plot(
    const float data[], 
    uint16_t length
) {
    float min=600,max=0;
    for (uint16_t i = 0; i < length; i++){
        if(min>data[i]) min=data[i];
        if(max<data[i]) max=data[i];
    }
    stretch_=200.0/(max-min);
    printf("addt s0.id,0,%d\xff\xff\xff", length);
    sleep_ms(100);
    for (uint16_t i = length; i > 0; i--) {
        const uint8_t data_casted = static_cast<uint8_t>(
            static_cast<int>(data[i] * stretch_ + offset_) 
        );
        printf("%c", data_casted);
    }
    printf("\x01\xff\xff\xff");
}

void Screen::send_spectrum_plot(
    const float data[],
    uint16_t length
) {
    float min=600,max=0;
    for (uint16_t i = 0; i < length; i++){
        if(min>data[i]) min=data[i];
        if(max<data[i]) max=data[i];
    }
    stretch1_=200.0/(max-min);
    printf("addt s1.id,0,%d\xff\xff\xff", length);
    sleep_ms(100);
    for (uint16_t i = length; i > 0; i--) {
        const uint8_t data_casted = static_cast<uint8_t>(
            static_cast<int>(data[i] * stretch1_ + offset1_) 
        );
        printf("%c", data_casted);
    }
    printf("\x01\xff\xff\xff");
}

void Screen::send_text(const char *text) {
    printf("slt0.txt=\"%s\"\xff\xff\xff", text);
}

void Screen::send_graph_with_step(const float data[], uint16_t length, uint16_t step){
    printf("addt s0.id,0,%d\xff\xff\xff", length);
    sleep_ms(100);
    for (uint16_t i = 0; i < length; i+=step) {
        const uint8_t data_casted = static_cast<uint8_t>(
            static_cast<int>(data[i] * stretch_ + offset_) 
        );
        printf("%c", data_casted);
    }
    printf("\x01\xff\xff\xff");
}
