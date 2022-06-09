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

namespace LinSer {

/**=========================== Buffer::_buffer functions =========================== **/
void Buffer::__buffer::push(char c){
	// check if buffer is full
	if(count >= SERIALBUFFERSIZE){ 
		#if LINSERDEBUG >= 1
		std::cout << "[LinSer] Buffer overflow!" << std::endl;
		#endif
		throw SerialBufferOverflowException(c); 
	}
	buff[count + front] = c;
	count++;
}

char Buffer::__buffer::pop(void){
	if(!count){ 
		#if LINSERDEBUG >= 1
		std::cout << "[LinSer] Buffer underflow!" << std::endl;
		#endif
		throw SerialBufferUnderflowException(0);
	}
	front++;
	count--;
	return buff[front-1];
}

void Buffer::__buffer::flushbuffer(){
	count = 0;
	front = 0;
}

inline unsigned int Buffer::__buffer::getBufferSize() const {
	return count;
}

void Buffer::__buffer::lock(const char* indicator = nullptr){
	#if LINSERDEBUG >= 4
	if(indicator){
		std::cout << "[LinSer] locking: " << indicator << std::endl;
	}
	#else
	(void)indicator;
	#endif
	Mutex.lock();
}

void Buffer::__buffer::unlock(const char* indicator = nullptr){
	#if LINSERDEBUG >= 4
	if(indicator){
		std::cout << "[LinSer] unlocking: " << indicator << std::endl;
	}
	#else
	(void)indicator;
	#endif
	Mutex.unlock();
}


Serial::Serial(const char* Port, const SerParam& SP, const SerTimeOut& ST){
	// setup handle
	hSerial = open(Port, O_RDWR);

	// Check for errors
	if (hSerial < 0) {
		#if LINSERDEBUG >= 1
		std::cout << "[LinSer] Error " << std::to_string(errno) << " from open(): " << std::strerror(errno) << std::endl;
		#endif
		throw SerialException(
			"Error " + std::to_string(errno) +
			" from open(): " + std::strerror(errno),
			SerialException::openError);
	}

	// Create new termios struct, we call it 'tty' for convention
	// No need for "= {0}" at the end as we'll immediately write the existing
	// config to this struct
	struct termios tty;

	// Read in existing settings, and handle any error
	// NOTE: This is important! POSIX states that the struct passed to tcsetattr()
	// must have been initialized with a call to tcgetattr() overwise behaviour
	// is undefined
	if(tcgetattr(hSerial, &tty) != 0) {
		#if LINSERDEBUG >= 1
		std::cout << "[LinSer] Error " << std::to_string(errno) << " from tcgetattr(): " << std::strerror(errno) << std::endl;
		#endif
		throw SerialException(
			"Error " + std::to_string(errno) +
			"from tcgetattr(): " + std::strerror(errno),
			SerialException::tcgetarrtError);
	}

	// control modes
	// Parity
	if(SP.P == SerParam::Parity::PAR_NONE){
		tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity
		tty.c_cflag &= ~PARODD; // clear parity odd bit, disabling odd parity
		#if LINSERDEBUG >= 2
		std::cout << "[LinSer] Using no parity" << std::endl;
		#endif
	}
	if(SP.P == SerParam::Parity::PAR_EVEN){
		tty.c_cflag |=  PARENB;  // Set parity bit, enabling even parity
		tty.c_cflag &= ~PARODD;  // clear parity odd bit, disabling odd parity
		#if LINSERDEBUG >= 2
		std::cout << "[LinSer] Using even parity" << std::endl;
		#endif
	}
	if(SP.P == SerParam::Parity::PAR_ODD){
		#if LINSERDEBUG >= 2
		std::cout << "[LinSer] Using odd parity" << std::endl;
		#endif
		tty.c_cflag |= PARENB;  // Set parity bit, enabling parity
		tty.c_cflag |= PARODD;  // Set parity odd bit, enabling odd parity
	}
	// StopBits
	if(SP.SB == SerParam::StopBits::ONE_STOP){
		tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication
		#if LINSERDEBUG >= 2
		std::cout << "[LinSer] using one stop bit" << std::endl;
		#endif
	}
	if(SP.SB == SerParam::StopBits::TWO_STOP){
		tty.c_cflag |= CSTOPB;  // Set stop field, two stop bits used in communication
		#if LINSERDEBUG >= 2
		std::cout << "[LinSer] using two stop bits" << std::endl;
		#endif
	}
	tty.c_cflag &= ~CSIZE; // Clear all the size bits
	// tty.c_cflag |= (tcflag_t)SP.bytesize; // set size bits
	tty.c_cflag |= CS8;
	tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

	// local modes
	tty.c_lflag &= ~ICANON; // disable canonical mode
	// disable due to disabled canonical mode
	tty.c_lflag &= ~ECHO; // Disable echo
	tty.c_lflag &= ~ECHOE; // Disable erasure
	tty.c_lflag &= ~ECHONL; // Disable new-line echo
	tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

	// input modes
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off software flow ctrl
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

	// output modes
	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
	// tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT IN LINUX)
	// tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT IN LINUX)

	// set timeout
	tty.c_cc[VTIME] = ST.__VTIME;
	tty.c_cc[VMIN]  = ST.__VMIN;

	#if LINSERDEBUG >= 2
	std::cout << "[LinSer] VTime set to: " << (int)ST.__VTIME*100 << std::endl;
	std::cout << "[LinSer] VMIN set to: " << (int)ST.__VMIN << std::endl;
	#endif

	#if LINSERDEBUG >= 2
	// Set in/out baud rate to be baudrate
	std::cout << "[LinSer] Baudrate set to: ";
	switch (SP.rate)
	{
	case SerParam::Baud0:
		std::cout << "B0" << std::endl;
		break;
	case SerParam::Baud50:
		std::cout << "B50" << std::endl;
		break;
	case SerParam::Baud75:
		std::cout << "B75" << std::endl;
		break;
	case SerParam::Baud110:
		std::cout << "B110" << std::endl;
		break;
	case SerParam::Baud134:
		std::cout << "B134" << std::endl;
		break;
	case SerParam::Baud150:
		std::cout << "B150" << std::endl;
		break;
	case SerParam::Baud200:
		std::cout << "B200" << std::endl;
		break;
	case SerParam::Baud300:
		std::cout << "B300" << std::endl;
		break;
	case SerParam::Baud600:
		std::cout << "B600" << std::endl;
		break;
	case SerParam::Baud1200:
		std::cout << "B1200" << std::endl;
		break;
	case SerParam::Baud1800:
		std::cout << "B1800" << std::endl;
		break;
	case SerParam::Baud2400:
		std::cout << "B2400" << std::endl;
		break;
	case SerParam::Baud4800:
		std::cout << "B4800" << std::endl;
		break;
	case SerParam::Baud9600:
		std::cout << "B9600" << std::endl;
		break;
	case SerParam::Baud19200:
		std::cout << "B19200" << std::endl;
		break;
	case SerParam::Baud38400:
		std::cout << "B38400" << std::endl;
		break;
	case SerParam::Baud57600:
		std::cout << "B57600" << std::endl;
		break;
	case SerParam::Baud115200:
		std::cout << "B115200" << std::endl;
		break;
	case SerParam::Baud230400:
		std::cout << "B230400" << std::endl;
		break;
	case SerParam::Baud460800:
		std::cout << "B460800" << std::endl;
		break;
	default:
		break;
	}
	#endif

	cfsetispeed(&tty, (speed_t)SP.rate);
	cfsetospeed(&tty, (speed_t)SP.rate);
	cfsetspeed(&tty, (speed_t)SP.rate);

	// Save tty settings, also checking for error
	if (tcsetattr(hSerial, TCSANOW, &tty) != 0) {
		#if LINSERDEBUG >= 1
		std::cout << "[LinSer] Error " << std::to_string(errno) << "from tcsetattr(): " << std::strerror(errno) << std::endl;
		#endif
		throw SerialException(
			"Error " + std::to_string(errno) +
			"from tcsetattr(): " + std::strerror(errno),
			SerialException::tcsetattrError);
	}

	usleep(10000); //required to make flush work, for some reason
  	tcflush(hSerial,TCIOFLUSH);

	// start threads
	IncomingBuffer.ThreadFunc = std::thread(ReadThreadFunc, std::ref(IncomingBuffer), std::ref(hSerial), std::ref(SerialHandleMutex));
}



Serial::~Serial(){
	#if LINSERDEBUG >= 3
	std::cout << "[LinSer] Closing Serial thread" << std::endl;
	#endif
	IncomingBuffer.StopThread = true;
	IncomingBuffer.ThreadFunc.join();
	
	SerialHandleMutex.lock();
	close(hSerial);
	SerialHandleMutex.unlock();
	hSerial = 0;

	#if LINSERDEBUG >= 3
	std::cout << "[LinSer] Serial thread closed" << std::endl;
	#endif
}

unsigned int Serial::available() {
	IncomingBuffer.lock();
	unsigned int size = IncomingBuffer.getBufferSize();
	IncomingBuffer.unlock();
	return size;
}

// unsigned int Serial::availableForWrite(){
// 	OutGoingBuffer.lock();
// 	unsigned int size = OutGoingBuffer.getBufferSize();
// 	OutGoingBuffer.unlock();
// 	return SERIALBUFFERSIZE - size;
// }

// void Serial::flush(const unsigned int refreshrate){
// 	while(1){
// 		OutGoingBuffer.lock("flush");
// 		if(OutGoingBuffer.getBufferSize() == 0){
// 			OutGoingBuffer.unlock("flush"); // unlock buffer
// 			break;
// 		}
// 		OutGoingBuffer.unlock("flush");
// 		// keep mutex unlocked for refreshrate
// 		std::this_thread::sleep_for(std::chrono::milliseconds(refreshrate));
// 	}
// }

void Serial::clearBuffer(){
	// OutGoingBuffer.lock("clearbuffer");
	IncomingBuffer.lock("clearbuffer");
		IncomingBuffer.flushbuffer();
		// OutGoingBuffer.flushbuffer();
	// OutGoingBuffer.unlock("clearbuffer");
	IncomingBuffer.unlock("clearbuffer");
}

void Serial::setTimeout(const SerTimeOut& ST){
	// OutGoingBuffer.lock("SetTimeout");
	IncomingBuffer.lock("SetTimeout");
	SerialHandleMutex.lock();

	// Create new termios struct, we call it 'tty' for convention
	// No need for "= {0}" at the end as we'll immediately write the existing
	// config to this struct
	struct termios tty;

	// Read in existing settings, and handle any error
	if(tcgetattr(hSerial, &tty) != 0) {
		#if LINSERDEBUG >= 1
		std::cout << "[LinSer] Error " << std::to_string(errno) << " from tcgetattr(): " << std::strerror(errno) << std::endl;
		#endif
		throw SerialException(
			"Error " + std::to_string(errno) +
			"from tcgetattr: " + std::strerror(errno),
			SerialException::tcgetarrtError);
	}

	// set timeout
	tty.c_cc[VTIME] = ST.__VTIME;
	tty.c_cc[VMIN] = ST.__VMIN;

	// Save tty settings, also checking for error
	if (tcsetattr(hSerial, TCSANOW, &tty) != 0) {
		#if LINSERDEBUG >= 1
		std::cout << "[LinSer] Error " << std::to_string(errno) << " from tcsetattr(): " << std::strerror(errno) << std::endl;
		#endif
		throw SerialException(
			"Error " + std::to_string(errno) +
			"from tcsetattr: " + std::strerror(errno),
			SerialException::tcsetattrError);
	}

	SerialHandleMutex.unlock();
	// OutGoingBuffer.unlock("SetTimeout");
	IncomingBuffer.unlock("SetTimeout");
}

char Serial::readByte(){
	IncomingBuffer.lock("readbyte");
	char temp = IncomingBuffer.pop();
	IncomingBuffer.unlock("readbyte");
	return temp;
}

unsigned int Serial::readBytes(char* buffer, unsigned int length){
	unsigned int i = 0;
	IncomingBuffer.lock("readbytes");
	if(length == 0){
		length = IncomingBuffer.getBufferSize();
	}
	for(; i < length && IncomingBuffer.getBufferSize(); i++){
		char temp = IncomingBuffer.pop();
		buffer[i] = temp;
	}
	IncomingBuffer.unlock("readbytes");
	return i;
}

unsigned int Serial::readBytesUntil(char* buffer, const char terminator, unsigned int length){
	unsigned int i = 0;
	IncomingBuffer.lock("readbytesuntil");
	if(length == 0)
		length = IncomingBuffer.getBufferSize();
	for(; i < IncomingBuffer.getBufferSize() && i < length; i++){
		buffer[i] = IncomingBuffer.pop();
		if(buffer[i] == terminator){
			break;
		}
	}
	IncomingBuffer.unlock("readbytesuntil");
	return i;
}

std::string Serial::readLine(const std::string& newline, int64_t timeout_ms){
	std::string S = readStringUntil(newline);
	if(S.find(newline) == std::string::npos){
		// newline not yet found, wait max time out untill newline character
		auto start = std::chrono::system_clock::now();
		while(true){
			while((std::chrono::system_clock::now() - start).count() < timeout_ms * 1000 && IncomingBuffer.getBufferSize() == 0) {}
			S += readStringUntil(newline);
			// exit if new line was found or timeout was reached
			if(S.find(newline) != std::string::npos || (std::chrono::system_clock::now() - start).count() >= timeout_ms * 1000)
				break;
		}
		// check if newline was eventually found or not
		if(S.find(newline) == std::string::npos)
			throw NoEOLTimeoutException();
	}
	S.erase(S.end() - newline.length(), S.end());
	return S;
}

std::string Serial::readString(){
	std::string S;
	IncomingBuffer.lock("readstring");
	while(IncomingBuffer.getBufferSize()){
		S += IncomingBuffer.pop();
	}
	IncomingBuffer.unlock("readstring");
	return S;
}

std::string Serial::readStringUntil(const char terminator){
	std::string S;
	IncomingBuffer.lock("readstringuntil");
	while(IncomingBuffer.getBufferSize()){
		S += IncomingBuffer.pop();
		if(S.back() == terminator)
			break;
	}
	IncomingBuffer.unlock("readstringuntil");
	return S;
}

std::string Serial::readStringUntil(const std::string& substr){
	std::string S;
	IncomingBuffer.lock("readstringuntil");
	while(IncomingBuffer.getBufferSize()){
		S += IncomingBuffer.pop();
		if(S.find(substr) != std::string::npos)
			break;
	}
	IncomingBuffer.unlock("readstringuntil");
	return S;
}

void Serial::writeByte(const char val){
	char buf[1] = {val};
	writeBytes(buf, 1);
}

void Serial::writeStr(const std::string str){
	writeBytes(str.c_str(), str.length());
}

void Serial::writeBytes(const char* buf, const unsigned int len){
	SerialHandleMutex.lock();
	int n = write(hSerial, buf, len);
	SerialHandleMutex.unlock();
	if(n < 0){
		// error occurred. Inform user
		#if LINSERDEBUG >= 1
		std::cout << "[LinSer] Error writing: " << std::strerror(errno) << std::endl;
		#endif
	}
}

int Serial::ReadThreadFunc(Buffer::Buffer& IncomingBuffer, int& hSerial, std::mutex& SHMutex){
	while(!IncomingBuffer.StopThread){
		std::this_thread::sleep_for(std::chrono::milliseconds(SERIALREADSLEEPTIME));
		char szBuff[SERIALBUFFERSIZE + 1] = {0};
		// Read bytes. The behaviour of read() (e.g. does it block?,
		// how long does it block for?) depends on the configuration
		// settings above, specifically VMIN and VTIME

		int n = 0; // acts as a buffer to hold byte counts

		// set buffer to all 0
		memset(&szBuff, '\0', sizeof(szBuff));

		// lock and read data
		SHMutex.lock();
		ioctl(hSerial, FIONREAD, &n);
		if(n){
			n = read(hSerial, &szBuff, sizeof(szBuff));
		}
		SHMutex.unlock();
		// read nothing
		if(!n){
			continue; // read returned 0 or ioctl returned 0 thus continue the loop.
		}
		// error
		if (n < 0) {
			// error occurred. Inform user
			#if LINSERDEBUG >= 1
			std::cout << "[LinSer] Error reading: " << std::strerror(errno) << std::endl;
			#endif
		}
		// read something
		if(n > 0){
			try{
				IncomingBuffer.lock("readthread");
				for(int i = 0; i < n && !IncomingBuffer.StopThread; i++)
					IncomingBuffer.push(szBuff[i]);
				IncomingBuffer.unlock("readthread");
			}
			catch(Buffer::SerialBufferException const &e){
				std::cerr << e.what() << std::endl;
				#if LINSERDEBUG >= 1
				std::cout << "[LinSer] Stopping reading thread due to exception " << std::endl;
				#endif
				IncomingBuffer.unlock("readthread-crash");
				while(!IncomingBuffer.StopThread){}
				return -1;
			}
		}
	}

	#if LINSERDEBUG >= 3
	std::cout << "[LinSer] Stopped reading thread" << std::endl;
	#endif

	return 0;
}

} // end of LinSer namespace
