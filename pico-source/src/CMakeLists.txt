# include_directories(${PROJECT_SOURCE_DIR})
include_directories(../include)
include_directories(../PicoQuickPID/src)

add_executable(controller
        controller.cpp
        motor.cpp
        ../PicoQuickPID/src/QuickPID.cpp
        )

# Enable stdio over the pico's USB connection
pico_enable_stdio_usb(controller 1)

# pull in common dependencies and additional pwm hardware support
target_link_libraries(controller pico_stdlib hardware_pwm QuickPID)

# create map/bin/hex file etc.
pico_add_extra_outputs(controller)
