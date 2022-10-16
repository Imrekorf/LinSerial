#include <iostream>
#include <chrono>
#include <cstring>

// Linux headers
#include <fcntl.h> 		// Contains file controls like O_RDWR
#include <cerrno>		// errno getter
#include <termios.h>	// Contains POSIX terminal control definitions
#include <unistd.h> 	// write(), read(), close()
#include <sys/ioctl.h>	// ioctl()
#include "LinSerial.h"

// based on: https://web.archive.org/web/20180127160838/http://bd.eduweb.hhs.nl/micprg/pdf/serial-win.pdf
// Linux version rewrite based on: https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/

using namespace linSer;

std::map<serParam::baudrate, int> serParam::baudrateString  = {
	{serParam::b0, 		0}, 	 {serParam::b50, 		50}, 
	{serParam::b75, 	75}, 	 {serParam::b110, 		110},
	{serParam::b134, 	14}, 	 {serParam::b150, 		150}, 
	{serParam::b200, 	200},	 {serParam::b300, 		300},
	{serParam::b600, 	600},	 {serParam::b1200, 		1200}, 
	{serParam::b1800, 	1800}, 	 {serParam::b2400, 		2400},
	{serParam::b4800, 	4800}, 	 {serParam::b9600, 		9600}, 
	{serParam::b19200, 	19200},  {serParam::b38400, 	38400},
	{serParam::b57600, 	57600},  {serParam::b115200, 	115200}, 
	{serParam::b230400, 230400}, {serParam::b460800, 	460800}
};

/**=========================== buffer::_buffer functions =========================== **/
void buffer::buffer::push(char c) {
	// Check if buffer is full
	if (count >= SERIAL_BUFFER_SIZE) { 
		linSerLogError("buffer overflow!\n");
		throw serialBufferOverflowException(c); 
	}
	buff[count + front] = c;
	count++;
}

char buffer::buffer::pop(void) {
	if (!count) { 
		linSerLogError("buffer underflow!\n");
		throw SerialBufferUnderflowException(0);
	}
	front++;
	count--;
	return buff[front-1];
}

void buffer::buffer::flushbuffer() {
	count = 0;
	front = 0;
}

inline unsigned int buffer::buffer::getBufferSize() const {
	return count;
}


serialBase::serialBase(const char* Port, const serParam& SP, const serTimeout& ST, const std::string eolString) {
	this->eolString = eolString;

	// Setup handle
	hSerial = open(Port, O_RDWR);

	// Check for errors
	if (hSerial < 0) {
		linSerLogError("%s from open(): %s\n", std::to_string(errno).c_str(), std::strerror(errno));
		throw serialException(
			"Error " + std::to_string(errno) +
			" from open(): " + std::strerror(errno),
			serialException::openError);
	}

	// Create new termios struct, we call it 'tty' for convention
	// No need for "= {0}" at the end as we'll immediately write the existing
	// config to this struct
	struct termios tty;

	// Read in existing settings, and handle any error
	// NOTE: This is important! POSIX states that the struct passed to tcsetattr()
	// must have been initialized with a call to tcgetattr() overwise behaviour
	// is undefined
	if (tcgetattr(hSerial, &tty) != 0) {
		linSerLogError("%s from tcgetattr(): %s\n", std::to_string(errno).c_str(), std::strerror(errno));
		throw serialException(
			"Error " + std::to_string(errno) +
			"from tcgetattr(): " + std::strerror(errno),
			serialException::tcgetattrError);
	}

	// Control modes
	// Parity
	if(SP.parity_ == serParam::parity::PAR_NONE){
		tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity
		tty.c_cflag &= ~PARODD; // clear parity odd bit, disabling odd parity
		linSerLogInfo("Using no parity\n");
	}
	if(SP.parity_ == serParam::parity::PAR_EVEN){
		tty.c_cflag |=  PARENB;  // Set parity bit, enabling even parity
		tty.c_cflag &= ~PARODD;  // clear parity odd bit, disabling odd parity
		linSerLogInfo("Using even parity\n");
	}
	if(SP.parity_ == serParam::parity::PAR_ODD){
		tty.c_cflag |= PARENB;  // Set parity bit, enabling parity
		tty.c_cflag |= PARODD;  // Set parity odd bit, enabling odd parity
		linSerLogInfo("Using odd parity\n");
	}
	// Stop bits
	if(SP.stopBits_ == serParam::stopBits::ONE_STOP){
		tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication
		linSerLogInfo("Using one stop bit\n");
	}
	if(SP.stopBits_ == serParam::stopBits::TWO_STOP){
		tty.c_cflag |= CSTOPB;  // Set stop field, two stop bits used in communication
		linSerLogInfo("Using two stop bits\n");
	}
	tty.c_cflag &= ~CSIZE; // Clear all the size bits
	tty.c_cflag |= (tcflag_t)SP.bitCount_; // Set byte size
	tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

	// Local modes
	tty.c_lflag &= ~ICANON; // disable canonical mode
	// Disable due to disabled canonical mode
	tty.c_lflag &= ~ECHO; // Disable echo
	tty.c_lflag &= ~ECHOE; // Disable erasure
	tty.c_lflag &= ~ECHONL; // Disable new-line echo
	tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

	// Input modes
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off software flow ctrl
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

	// Output modes
	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
	// tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT IN LINUX)
	// tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT IN LINUX)

	// Set timeout
	tty.c_cc[VTIME] = ST.__VTIME;
	tty.c_cc[VMIN]  = ST.__VMIN;

	linSerLogDebug("VTime set to: %d\n", (int)ST.__VTIME*100);
	linSerLogDebug("VMIN set to:  %d\n", (int)ST.__VMIN);

	linSerLogInfo("Baudrate set to: %d\n", serParam::baudrateString[SP.rate]);

	cfsetspeed(&tty, (speed_t)SP.rate);

	// Save tty settings, also checking for error
	if (tcsetattr(hSerial, TCSANOW, &tty) != 0) {
		linSerLogError("%s from tcsetattr(): %s\n", std::to_string(errno).c_str(), std::strerror(errno));
		throw serialException(
			"Error " + std::to_string(errno) +
			"from tcsetattr(): " + std::strerror(errno),
			serialException::tcsetattrError);
	}

	usleep(10000); // Required to make flush work, for some reason
  	tcflush(hSerial,TCIOFLUSH);

	
}



serialBase::~serialBase() {
	
}



void serialBase::setTimeout(const serTimeout& ST) {
	std::lock_guard<std::mutex> lock(serialHandleMutex);

	// Create new termios struct, we call it 'tty' for convention
	// No need for "= {0}" at the end as we'll immediately write the existing
	// config to this struct
	struct termios tty;

	// Read in existing settings, and handle any error
	if (tcgetattr(hSerial, &tty) != 0) {
		linSerLogError("%s from tcgetattr(): %s\n", std::to_string(errno).c_str(), std::strerror(errno));
		throw serialException(
			"Error " + std::to_string(errno) +
			"from tcgetattr: " + std::strerror(errno),
			serialException::tcgetattrError);
	}

	// Set timeout
	tty.c_cc[VTIME] = ST.__VTIME;
	tty.c_cc[VMIN] = ST.__VMIN;

	linSerLogDebug("VTime set to: %d\n", (int)ST.__VTIME*100);
	linSerLogDebug("VMIN set to:  %d\n", (int)ST.__VMIN);


	// Save tty settings, also checking for error
	if (tcsetattr(hSerial, TCSANOW, &tty) != 0) {
		linSerLogError("%s from tcsetattr(): %s\n", std::to_string(errno).c_str(), std::strerror(errno));
		throw serialException(
			"Error " + std::to_string(errno) +
			"from tcsetattr: " + std::strerror(errno),
			serialException::tcsetattrError);
	}
}

char serialBase::readByte() {
	char temp;
	serRead(&temp, 1);
	return temp;
}

unsigned int serialBase::readBytes(char* buffer, unsigned int length) {
	return serRead(buffer, length);
}

unsigned int serialBase::readBytesUntil(char* buffer, const char terminator, unsigned int length) {
	return serReadUntil(buffer, terminator, length);
}

size_t serialBase::readBytesUntil(char* buffer, size_t bufferLength, const char* substr, size_t substrLength) {
	const size_t avail = available();
	size_t read = 0, c = 0;
	read += serReadUntil(buffer, substr[0], avail);
	while(c < substrLength && read < bufferLength) {
		if (buffer[read-1] == substr[c++]) { // continue
			size_t r = serRead(buffer + read, 1); // read 1 more
			if (r <= 0)
				break; // no more bytes available;
			read += r;
		} else {
			c = 0;
		}
	}

	return read;
}

std::string serialBase::readString() {
	const size_t avail = available();
	std::unique_ptr<char> buffer = std::make_unique<char>(avail);
	const size_t read = serRead(buffer.get(), avail);
	return std::string(buffer.get(), read); // check if this does not create problems with memory deallocation.
}

std::string serialBase::readStringUntil(const char terminator) {
	const size_t avail = available();
	std::unique_ptr<char> buffer = std::make_unique<char>(avail);
	const size_t read = serReadUntil(buffer.get(), terminator, avail);
	return std::string(buffer.get(), read); // check if this does not create problems with memory deallocation.
}

std::string serialBase::readStringUntil(std::string& substr) {
	const size_t avail = available();
	std::unique_ptr<char> buffer = std::make_unique<char>(avail);
	const size_t read = readBytesUntil(buffer.get(), avail, substr.c_str(), substr.length());
	return std::string(buffer.get(), read); // check if this does not create problems with memory deallocation.
}

std::string serialBase::readLine() {
	return readStringUntil(eolString);
}

void serialBase::writeByte(const char val) {
	char buf[1] = {val};
	writeBytes(buf, 1);
}

void serialBase::writeStr(const std::string str) {
	writeBytes(str.c_str(), str.length());
}

void serialBase::writeBytes(const char* buf, const unsigned int len) {
	std::unique_lock<std::mutex> lock(serialHandleMutex);
	int n = write(hSerial, buf, len);
	lock.unlock();
	if (n < 0) {
		linSerLogError("during write: %s\n", std::strerror(errno));
		throw serialException("during write: " + std::string(std::strerror(errno)), serialException::writeError);
	}
	if (n < 0) // Error occurred. Inform user
		linSerLogError("during write: %s\n", std::strerror(errno));
}

int serialAsync::_readThreadFunc(serialAsync& self) {
	while (!self.stopThread) {
		std::this_thread::sleep_for(std::chrono::milliseconds(SERIAL_READ_SLEEP_TIME));
		char szBuff[SERIAL_BUFFER_SIZE + 1] = {0};
		// Read bytes. The behaviour of read() (e.g. does it block?,
		// how long does it block for?) depends on the configuration
		// settings above, specifically VMIN and VTIME

		int n = 0; // acts as a buffer to hold byte counts

		// Set buffer to all 0
		memset(&szBuff, '\0', sizeof(szBuff));

		// Lock and read data
		std::unique_lock<std::mutex> lock(self.serialHandleMutex);
		ioctl(self.hSerial, FIONREAD, &n);
		if (n != 0)
			n = read(self.hSerial, &szBuff, sizeof(szBuff));
		lock.unlock();
		// Read nothing
		if (!n)
			continue; // Read returned 0 or ioctl returned 0 thus continue the loop
		// Error occurred. Inform user
		if (n < 0) 
			linSerLogError("during read: %s\n", std::strerror(errno));
		// Read something
		if (n > 0) {
			try {
				std::lock_guard<std::mutex> lock(self.incomingBufferMutex);
				for(int i = 0; i < n && !self.stopThread; i++)
					self.incomingBuffer.push(szBuff[i]);
			}
			catch (buffer::serialBufferException const &e) {
				linSerLogError("Stopping reading thread due to exception %s\n", e.what());
				while(!self.stopThread){}
				return -1;
			}
		}
	}
	linSerLogDebug("Stopping reading thread\n");

	return 0;
}

serialSync::serialSync(const char* port, const serParam& serPar, const serTimeout& serTim, const std::string eolString) : serialBase(port, serPar, serTim, eolString) {

}

serialSync::~serialSync() {

}

size_t serialSync::serRead(char* buffer, size_t size) {
	int n = 0; // acts as a buffer to hold byte counts
	std::unique_lock<std::mutex> lock(serialHandleMutex);
	if (ioctl(hSerial, FIONREAD, &n) == -1) {
		linSerLogError("during ioctl: %s\n", std::strerror(errno));
		serialException("Error during read: " + std::string(std::strerror(errno)), serialException::ioctlError);
	}
	if (n != 0)
		n = read(hSerial, &buffer, sizeof(size));
	lock.unlock();
	if (n < 0) {
		linSerLogError("during read: %s\n", std::strerror(errno));
		memset(&buffer, '\0', sizeof(buffer));
		serialException("Error during read: " + std::string(std::strerror(errno)), serialException::readError);
	}
	return n;
}

size_t serialSync::available() {
	int n = 0; // acts as a buffer to hold byte counts
	std::unique_lock<std::mutex> lock(serialHandleMutex);
	if (ioctl(hSerial, FIONREAD, &n) == -1) {
		linSerLogError("during ioctl: %s\n", std::strerror(errno));
		serialException("Error during read: " + std::string(std::strerror(errno)), serialException::ioctlError);
	}
	return n;
}

size_t serialSync::serReadUntil(char* buffer, char terminator, size_t length) {
	size_t i = 0;
	size_t buffer_size = available();
	if (length == 0)
		length = buffer_size;
	for (; i < buffer_size && i < length; i++) {
		serRead(buffer+i, 1);
		if (buffer[i] == terminator)
			break;
	}
	return i;
}

serialAsync::serialAsync(const char* port, const serParam& serPar, const serTimeout& serTim, const std::string eolString) : serialBase(port, serPar, serTim, eolString) {
	// Start threads
	incomingThread = std::thread(_readThreadFunc, std::ref(*this));
}

serialAsync::~serialAsync() {
	linSerLogInfo("Closing serial thread...\n");
	stopThread = true;
	incomingThread.join();
	std::unique_lock<std::mutex> lock(serialHandleMutex);
	
	close(hSerial);
	lock.unlock();
	hSerial = 0;

	#if LINSER_LOGLEVEL >= 3
	linSerLogInfo("closed!\n");
	#endif
}

size_t serialAsync::serRead(char* buffer, size_t length) {
	size_t i = 0;
	const std::lock_guard<std::mutex> lock(incomingBufferMutex);
	size_t buffer_size = incomingBuffer.getBufferSize();
	if (length == 0)
		length = buffer_size;
	for (; i < buffer_size && i < length; i++) // todo: make this into 1 function call
		buffer[i] = incomingBuffer.pop();
	return i;
}

size_t serialAsync::serReadUntil(char* buffer, char terminator, size_t length) {
	unsigned int i = 0;
	const std::lock_guard<std::mutex> lock(incomingBufferMutex);
	if (length == 0)
		length = incomingBuffer.getBufferSize();
	for (; i < incomingBuffer.getBufferSize() && i < length; i++) {
		buffer[i] = incomingBuffer.pop();
		if (buffer[i] == terminator)
			break;
	}
	return i;
}

size_t serialAsync::available() {
	const std::lock_guard<std::mutex> lock(incomingBufferMutex);
	unsigned int size = incomingBuffer.getBufferSize();
	return size;
}

void serialAsync::clearBuffer(){
	const std::lock_guard<std::mutex> lock(incomingBufferMutex);
	incomingBuffer.flushbuffer();
}