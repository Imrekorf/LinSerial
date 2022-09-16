all:
	g++ Example.cpp LinSerial.cpp -o lin_serial_example -pthread -Wall -Wextra -Wpedantic -DLINSER_LOG_LEVEL=7
