
// Linux headers
#include <fcntl.h> 		// Contains file controls like O_RDWR
#include <errno.h>		// errno getter
#include <termios.h>	// Contains POSIX terminal control definitions
#include <unistd.h> 	// write(), read(), close()
#include <sys/ioctl.h>	// ioctl()

#include "LinSerialc.h"

unsigned int initSerial(SerLinuxData* SD, const char* Port) {
	SD->hSerial = open(Port, O_RDWR);
	// Check for errors
	if (SD->hSerial < 0) {
		// todo: log error
		throw SerialException(
			"Error " + std::to_string(errno) +
			" from open(): " + std::strerror(errno),
			SerialException::openError);
	}
}