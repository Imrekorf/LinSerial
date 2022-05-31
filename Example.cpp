#include "LinSerial.h"
#include <cstring>
#include <unistd.h>

int main() {
    std::cout << LINSERDEBUG << std::endl;
    LinSer::Serial port("/dev/ttyACM1");
    // expands to: 
	// LinSer::Serial port("/dev/ttyACM0", LinSer::SerParam(
    //     LinSer::SerParam::Baudrate::Baud9600,
    //     LinSer::SerParam::Parity::PAR_NONE,
    //     LinSer::SerParam::StopBits::ONE_STOP,
    //     LinSer::SerParam::BitCount::ByteSize8
    // ), LinSer::SerTimeOut(0, 100));

    for (int i = 0; i < 2; i++){
        std::cout << "writing hello to port" << std::endl;
        port.writeStr("Hello!");
        while (!port.available()){
            usleep(10000);
        }
        std::cout << "Received: \"" << port.readLine() << "\"" << std::endl;
    }

	return 0;
}