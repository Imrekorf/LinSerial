all:
	g++ Example.cpp LinSerial.cpp -pthread -Wall -Wextra -Wpedantic -DLINSER_LOG_LEVEL=7