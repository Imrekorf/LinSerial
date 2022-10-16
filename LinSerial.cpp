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
void buffer::buffer::push(char c){
	// Check if buffer is full
	if (count >= SERIAL_BUFFER_SIZE) { 
		linSerLogError("buffer overflow!\n");
		throw serialBufferOverflowException(c); 
	}
	buff[count + front] = c;
	count++;
}

char buffer::buffer::pop(void){
	if (!count) { 
		linSerLogError("buffer underflow!\n");
		throw SerialBufferUnderflowException(0);
	}
	front++;
	count--;
	return buff[front-1];
}

void buffer::buffer::flushbuffer(){
	count = 0;
	front = 0;
}

inline unsigned int buffer::buffer::getBufferSize() const {
	return count;
}


serial::serial(const char* Port, const serParam& SP, const serTimeout& ST){
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
			serialException::tcgetarrtError);
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

	// Start threads
	incomingThread = std::thread(_readThreadFunc, std::ref(*this));
}



serial::~serial(){
	linSerLogInfo("Closing serial thread...\n");
	stopThread = true;
	incomingThread.join();
	
	serialHandleMutex.lock();
	close(hSerial);
	serialHandleMutex.unlock();
	hSerial = 0;

	#if LINSER_LOGLEVEL >= 3
	linSerLogInfo("closed!\n");
	#endif
}

unsigned int serial::available() {
	incomingBufferMutex.lock();
	unsigned int size = incomingBuffer.getBufferSize();
	incomingBufferMutex.unlock();
	return size;
}

void serial::clearBuffer(){
	incomingBufferMutex.lock();
		incomingBuffer.flushbuffer();
	incomingBufferMutex.unlock();
}

void serial::setTimeout(const serTimeout& ST){
	incomingBufferMutex.lock();
	serialHandleMutex.lock();

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
			serialException::tcgetarrtError);
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

	serialHandleMutex.unlock();
	incomingBufferMutex.unlock();
}

char serial::readByte(){
	incomingBufferMutex.lock();
	char temp = incomingBuffer.pop();
	incomingBufferMutex.unlock();
	return temp;
}

unsigned int serial::readBytes(char* buffer, unsigned int length){
	unsigned int i = 0;
	incomingBufferMutex.lock();
	if (length == 0)
		length = incomingBuffer.getBufferSize();
	for (; i < length && incomingBuffer.getBufferSize(); i++) {
		char temp = incomingBuffer.pop();
		buffer[i] = temp;
	}
	incomingBufferMutex.unlock();
	return i;
}

unsigned int serial::readBytesUntil(char* buffer, const char terminator, unsigned int length){
	unsigned int i = 0;
	incomingBufferMutex.lock();
	if (length == 0)
		length = incomingBuffer.getBufferSize();
	for (; i < incomingBuffer.getBufferSize() && i < length; i++) {
		buffer[i] = incomingBuffer.pop();
		if (buffer[i] == terminator)
			break;
	}
	incomingBufferMutex.unlock();
	return i;
}

std::string serial::readLine(const std::string& newline, size_t timeout_ms){
	std::string S = readStringUntil(newline);
	if (S.find(newline) == std::string::npos){
		// Newline not yet found, wait max time out untill newline character
		auto start = std::chrono::system_clock::now();
		while (true) {
			while (labs((std::chrono::system_clock::now() - start).count()) < timeout_ms * 1000 && incomingBuffer.getBufferSize() == 0) {}
			S += readStringUntil(newline);
			// Exit if new line was found or timeout was reached
			if(S.find(newline) != std::string::npos || labs((std::chrono::system_clock::now() - start).count()) >= timeout_ms * 1000)
				break;
		}
		// Check if newline was eventually found or not
		if (S.find(newline) == std::string::npos)
			throw noEOLTimeoutException(timeout_ms);
	}
	S.erase(S.end() - newline.length(), S.end());
	return S;
}

std::string serial::readString(){
	std::string S;
	incomingBufferMutex.lock();
	while (incomingBuffer.getBufferSize())
		S += incomingBuffer.pop();
	incomingBufferMutex.unlock();
	return S;
}

std::string serial::readStringUntil(const char terminator){
	std::string S;
	incomingBufferMutex.lock();
	while (incomingBuffer.getBufferSize()) {
		S += incomingBuffer.pop();
		if (S.back() == terminator)
			break;
	}
	incomingBufferMutex.unlock();
	return S;
}

std::string serial::readStringUntil(const std::string& substr){
	std::string S;
	incomingBufferMutex.lock();
	while (incomingBuffer.getBufferSize()) {
		S += incomingBuffer.pop();
		if (S.find(substr) != std::string::npos)
			break;
	}
	incomingBufferMutex.unlock();
	return S;
}

void serial::writeByte(const char val){
	char buf[1] = {val};
	writeBytes(buf, 1);
}

void serial::writeStr(const std::string str){
	writeBytes(str.c_str(), str.length());
}

void serial::writeBytes(const char* buf, const unsigned int len){
	serialHandleMutex.lock();
	int n = write(hSerial, buf, len);
	serialHandleMutex.unlock();
	if (n < 0) // Error occurred. Inform user
		linSerLogError("during write: %s\n", std::strerror(errno));
}

int serial::_readThreadFunc(serial& self){
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
		self.serialHandleMutex.lock();
		ioctl(self.hSerial, FIONREAD, &n);
		if (n != 0)
			n = read(self.hSerial, &szBuff, sizeof(szBuff));
		self.serialHandleMutex.unlock();
		// Read nothing
		if (!n)
			continue; // Read returned 0 or ioctl returned 0 thus continue the loop
		// Error occurred. Inform user
		if (n < 0) 
			linSerLogError("during read: %s\n", std::strerror(errno));
		// Read something
		if (n > 0) {
			try {
				self.incomingBufferMutex.lock();
				for(int i = 0; i < n && !self.stopThread; i++)
					self.incomingBuffer.push(szBuff[i]);
				self.incomingBufferMutex.unlock();
			}
			catch (buffer::serialBufferException const &e) {
				linSerLogError("Stopping reading thread due to exception %s\n", e.what());
				self.incomingBufferMutex.unlock();
				while(!self.stopThread){}
				return -1;
			}
		}
	}
	linSerLogDebug("Stopping reading thread\n");

	return 0;
}
