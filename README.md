Studying C language project design flow with PlatformIO IDE, ESP-IDF API on VSCode
## Section 1 
Installing and configuring design environment
https://platformio.org/install/ide?install=vscode

Get started with ESP-IDF and ESP32-DevKitC: debugging, unit testing, project analysis
https://docs.platformio.org/en/latest/tutorials/espressif32/espidf_debugging_unit_testing_analysis.html

Exercise1 (Main Branch) - Creating initial version and Blinky led.

## Section 2 
Interrupt allocation
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/intr_alloc.html

Exercise2 (ESP32-interfaces branch) - Creating I/O interrupt allocation function and callback.


## Section 3 
General Purpose Timer
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/timer.html?highlight=timer

Exercise3 (ESP32-Exercicio-Timer Branch) - Creating Timer setup, function and callback.


## Section 4 
LED Control (LEDC)
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/ledc.html?highlight=pwm

Exercise4 (ESP32-Exercicio-LED-PWM Branch) - Creating PWM setup, function and callback.

The purpose of this activity is to synchronize the value read on the ADC channel in a proportional control of a certain desired parameter.
Let's assume that the fader axis can rotate from 0 to 180 degrees, and in that sense, 0 degrees will be 0% LED control brightness, and 180 degrees of the fader axis will be 100% LED brightness.
