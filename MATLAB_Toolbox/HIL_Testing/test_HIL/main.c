#include "pico/stdlib.h"
#include <stdio.h>

int main() {
    stdio_init_all();
    sleep_ms(2000);


    const double MULTIPLIER = 20;
    double received_value;

    while (1) {
        if (scanf("%lf", &received_value) == 1) {

            double result_value = received_value * MULTIPLIER;

            // The actual data response for Simulink
            printf("%lf\\n", result_value);
        }
    }
    return 0;
}
