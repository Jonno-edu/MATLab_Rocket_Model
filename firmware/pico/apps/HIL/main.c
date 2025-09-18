/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include <stdio.h>
#include "led.h"

int main() {
    stdio_init_all();
    printf("HIL App: Initializing...\n");

    // For Pico W devices we need to initialise the driver
    if (cyw43_arch_init()) {
        printf("HIL App: WiFi driver failed to initialize.\n");
        return -1;
    }
    printf("HIL App: WiFi driver initialized successfully.\n");
    printf("HIL App: Starting main loop.\n");

    while (true) {
        // Call the shared LED flashing function
        flash_led(10);
    }
}