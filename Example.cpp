#include "LinSerial.h"
#include <cstring>
#include <unistd.h>

int main() {
    linSerLogInfo("Debug: %s", (LINSER_LOG_LEVEL & LINSER_LOG_DEBUG ? "on" : "off"));
    
    linSer::serial port("/dev/tty10");
    // expands to: 
	// LinSer::serial port("/dev/ttyACM0", LinSer::serParam(
    //     LinSer::serParam::baudrate::b9600,
    //     LinSer::serParam::parity::PAR_NONE,
    //     LinSer::serParam::stopBits::ONE_STOP,
    //     LinSer::serParam::bitCount::BIT_COUNT_8
    // ), LinSer::serTimeout(0, 100));

    for (int i = 0; i < 2; i++){
        std::cout << "writing hello to port" << std::endl;
        // port.writeStr("Hello!");
        port << "Hello!";
        while (!port.available()){
            usleep(10000);
        }
        std::cout << "Received: \"" << port.readLine() << "\"" << std::endl;
    }

	return 0;
}