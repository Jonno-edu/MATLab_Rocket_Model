// test_simple_echo.c
#include <stdio.h>
#include "pico/stdlib.h"

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    // LED setup
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, 1);  // Ready
    
    float input;
    uint8_t* input_ptr = (uint8_t*)&input;
    size_t bytes_received = 0;
    
    while (1) {
        int c = getchar_timeout_us(0);
        
        if (c != PICO_ERROR_TIMEOUT) {
            input_ptr[bytes_received++] = (uint8_t)c;
            
            if (bytes_received == 4) {  // Got 1 float
                gpio_put(25, 0);  // Processing
                
                float output = input * 2.0f;
                fwrite(&output, sizeof(float), 1, stdout);
                fflush(stdout);
                
                bytes_received = 0;
                gpio_put(25, 1);  // Ready
            }
        }
        sleep_us(10);
    }
}
