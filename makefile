

### Mini-makefile for quad-gy80. This makefile requires Arduino-mk, but you don't hav eto use it unless you're using an
### external editor like emacs or vim. Using the official IDE is always an option


ARDUINO_LIBS 	= ADXL345 Wire Servo L3G4200D

MONITOR_PORT 	= /dev/ttyACM0
BOARD_TAG 	= mega
ISP_PROG	= stk500v2
ISP_PORT	= /dev/ttyACM0

USER_LIB_PATH 	= /home/david/Arduino/quad-gy80/libraries

BOARD_TAG 	= mega
BOARD_SUB 	= atmega2560

include $(ARDMK_DIR)/Arduino.mk
