ARDUINO_DIR = /usr/share/arduino
ARDMK_DIR = /usr/share/arduino
BOARD_TAG = uno
BOARD_SUB = atmega168
ARDUINO_PORT = /dev/ttyUSB0
ARDUINO_LIBS = Servo
SCREEN = screen

include $(ARDMK_DIR)/Arduino.mk

$(OBJDIR)/sainsmart.o: ../profile.hh ../path.hh ../controllerbase.hh ../calibration.hh

repl:
	$(SCREEN) $(ARDUINO_PORT) 115200
