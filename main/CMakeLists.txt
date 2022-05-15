set(srcs "main.c")

set(requires "bluepad32")

# See: https://gitlab.com/ricardoquesada/bluepad32/-/issues/9
if(DEFINED ENV{BLUEPAD32_ARDUINO})
    list(APPEND srcs)
    list(APPEND requires "bluepad32_arduino" "arduino" )
endif()

idf_component_register(SRCS "arduino_main.cpp" "${srcs}"
                    INCLUDE_DIRS "."
		    REQUIRES "${requires}" "UartRemote" "Adafruit_NeoPixel" "arduinoFFT" "ESP32Servo" "LPF2")
