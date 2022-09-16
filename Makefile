all:
	g++ Example.cpp LinSerial.cpp -o lin_serial_example -pthread -Wall -Wextra -Wpedantic -DLINSERDEBUG=3