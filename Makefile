all:
	g++ Example.cpp LinSerial.cpp -pthread -Wall -Wextra -Wpedantic -DLINSERDEBUG=3