all:
	g++ Example.cpp LinSerial.cpp -pthread -Wall -Wextra -Wpedantic -DLINSERLOGLEVEL=7