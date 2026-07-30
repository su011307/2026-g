#include "display.h"

Screen::Screen()
{
    //ctor
}

void Screen::send_graph(float data[], uint16_t length)
{
    printf("addt s0.id,0,%d\xff\xff\xff", length);
    sleep_ms(100);
    for(uint16_t i = 0; i < length; i++)
    {
        uint8_t data_casted = (int)(data[i] * strentch + offset) % 256;
        printf("%c", data_casted);
    }
    printf("\x01\xff\xff\xff");
}


void Screen::send_text(char t[]) {
    printf("slt0.txt=\"%s\"\xff\xff\xff", t);
}

Screen::~Screen()
{
    //dtor
}

uint8_t Screen::get_button() {
/*

    static int test_index = 0;
    uint8_t mock_buttons[] = {0x2B, 0x2D, 0x31, 0x33, 0x76}; // +, -, 1, 3, Spec
    Sleep(10);
    uint8_t cmd = mock_buttons[test_index];
    test_index = (test_index + 1) % 5;
    return cmd;
*/
 //Ƕ��ʽ��

/*    while (uart_is_readable(DISP_UART_PORT)) {
        uint8_t byte = uart_getc(DISP_UART_PORT);

        // ״̬������ [CMD, 0xFF, 0xFF, 0xFF]
        switch (rx_state) {
            case 0:
                if (byte == 0x2B || byte == 0x2D || byte == 0x31 || byte == 0x33 || byte == 0x76) {
                    last_cmd = byte;
                    rx_state = 1;
                }
                break;
            case 1:
                rx_state = (byte == 0xFF) ? 2 : 0;
                break;
            case 2:
                rx_state = (byte == 0xFF) ? 3 : 0;
                break;
            case 3:
                if (byte == 0xFF) {
                    rx_state = 0;
                    return last_cmd;
                } else {
                    rx_state = 0;
                }
                break;
        }
    }
    return 0;
*/
}
