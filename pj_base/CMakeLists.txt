set(BinName "pj_base")

add_library(encoder encoder.cpp)
target_link_libraries(encoder pico_stdlib)

add_library(motor_driver motor_driver.cpp)
target_link_libraries(motor_driver hardware_gpio hardware_pwm pico_stdlib)

add_library(speed_controller speed_controller.cpp)
target_link_libraries(speed_controller encoder motor_driver)

add_executable(${BinName}
    Main.cpp
)

pico_enable_stdio_usb(${BinName} 1)
pico_enable_stdio_uart(${BinName} 1)

target_link_libraries(${BinName} pico_stdlib hardware_pwm hardware_adc hardware_i2c encoder motor_driver speed_controller)
pico_add_extra_outputs(${BinName})