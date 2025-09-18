#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "led.h"
#include <stdio.h>

void flash_led(int t) {
    printf("  led.c: LED ON\n");
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(t);
    printf("  led.c: LED OFF\n");
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(t);
}
