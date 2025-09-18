add_executable(pico_hil
  main.c
  ../../../../src/utils/led.c
)

target_include_directories(pico_hil PRIVATE
  ../../../../src/utils
)

target_link_libraries(pico_hil
  pico_stdlib
  pico_cyw43_arch_none
  hardware_gpio
)

pico_enable_stdio_usb(pico_hil 1)
pico_enable_stdio_uart(pico_hil 0)

pico_add_extra_outputs(pico_hil)