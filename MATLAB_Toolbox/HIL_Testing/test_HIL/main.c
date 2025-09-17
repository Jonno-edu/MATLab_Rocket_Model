#include "pico/stdlib.h"
#include <stdio.h>
#include "pico/cyw43_arch.h"

int main() {
    stdio_init_all();
    if (cyw43_arch_init()) {
        return -1;
    }
    sleep_ms(2000);


    const double kp = 20;
    const double kd = 5;
    double received_value, error, prev_error;
    prev_error = 0;
    bool led_state = false;

    while (1) {
        if (scanf("%lf", &received_value) == 1) {
            led_state = true;

            error = received_value;

            double result_value = error * kp + kd * (error - prev_error) / 0.01;
            // The actual data response for Simulink
            printf("%lf\\n", result_value);

            prev_error = error;
        }
        else {
            led_state = false;
        }
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);

    }
    return 0;
}
