#include <iostream>
#include <LinSerial.h>
#include <chrono>
#include <cstring>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <cerrno>
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <sys/ioctl.h>	// ioctl()

// based on: https://web.archive.org/web/20180127160838/http://bd.eduweb.hhs.nl/micprg/pdf/serial-win.pdf
// Linux version rewrite based on: https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/

using namespace LinSer;


/**=========================== Buffer::_buffer functions =========================== **/
void Buffer::__buffer::push(char c){
	// check if buffer is full
	if(count >= SERIALBUFFERSIZE){ 
		std::cout << "Buffer overflow!" << std::endl;
		throw SerialBufferOverflowException(c); return; }
	buff[count + front] = c;
	count++;
}

char Buffer::__buffer::pop(void){
	if(!count){ 
		std::cout << "Buffer underflow!" << std::endl;
		throw SerialBufferUnderflowException(0); return 0;}
	front++;
	count--;
	return buff[front-1];
}

void Buffer::__buffer::flushbuffer(){
	count = 0;
	front = 0;
}

unsigned int Buffer::__buffer::getbuffersize(){
	return count;
}




Serial::Serial(const char* Port, SerParam SP, SerTimeOut ST){
	// setup handle
	hSerial = open(Port, O_RDWR);

	// Check for errors
	if (hSerial < 0) {
		throw SerialException(
			"Error " + std::to_string(errno) +
			" from open: " + std::strerror(errno));
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
		throw SerialException(
			"Error " + std::to_string(errno) +
			"from tcgetattr: " + std::strerror(errno));
	}

	// control modes
	// Parity
	if(SP.P == SerParam::Parity::PAR_NONE){
		tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity
		std::cout << "[Serial] Using no parity" << std::endl;
	}
	if(SP.P == SerParam::Parity::PAR_EVEN){
		tty.c_cflag |= PARENB;  // Set parity bit, enabling even parity
		std::cout << "[Serial] Using even parity" << std::endl;
	}
	if(SP.P == SerParam::Parity::PAR_ODD){
		std::cout << "[Serial] Using odd parity" << std::endl;
		tty.c_cflag |= PARENB;  // Set parity bit, enabling parity
		tty.c_cflag |= PARODD;  // Set parity odd bit, enabling odd parity
	}
	// StopBits
	if(SP.SB == SerParam::StopBits::ONE_STOP){
		tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication
		std::cout << "[Serial] using one stop bit" << std::endl;
	}
	if(SP.SB == SerParam::StopBits::TWO_STOP){
		tty.c_cflag |= CSTOPB;  // Set stop field, two stop bits used in communication
		std::cout << "[Serial] using two stop bits" << std::endl;
	}
	tty.c_cflag &= ~CSIZE; // Clear all the size bits
	tty.c_cflag |= (tcflag_t)SP.bytesize; // set size bits
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
	std::cout << "[Serial] VTime set to: " << (int)ST.__VTIME << std::endl;
	tty.c_cc[VTIME] = ST.__VTIME;
	std::cout << "[Serial] VMIN set to: " << (int)ST.__VMIN << std::endl;
	tty.c_cc[VMIN] = ST.__VMIN;

	// Set in/out baud rate to be baudrate
	std::cout << "[Serial] Baudrate set to: ";
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
		std::cout << "B0" << std::endl;
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
	cfsetispeed(&tty, (speed_t)SP.rate);
	cfsetospeed(&tty, (speed_t)SP.rate);

	// Save tty settings, also checking for error
	if (tcsetattr(hSerial, TCSANOW, &tty) != 0) {
		throw SerialException(
			"Error " + std::to_string(errno) +
			"from tcgetattr: " + std::strerror(errno));
	}

	// flush both buffers
	ioctl(hSerial, TCFLSH, 2); // flush both

	// start threads
	ReadThread = std::thread(ReadThreadFunc, std::ref(IncomingBuffer), std::ref(hSerial), std::ref(SerialHandleMutex));
	SendThread = std::thread(SendThreadFunc, std::ref(OutGoingBuffer), std::ref(hSerial), std::ref(SerialHandleMutex));
}

Serial::~Serial(){
	std::cout << "Closing Serial threads" << std::endl;
	OutGoingBuffer.StopThread = true;
	IncomingBuffer.StopThread = true;

	ReadThread.join();
	SendThread.join();

	SerialHandleMutex.lock();
	close(hSerial);
	SerialHandleMutex.unlock();
	hSerial = 0;

	std::cout << "Serial threads closed" << std::endl;
}

unsigned int Serial::available(){
	IncomingBuffer.lock();
	unsigned int size = IncomingBuffer.getbuffersize();
	IncomingBuffer.unlock();
	return size;
}

unsigned int Serial::availableForWrite(){
	OutGoingBuffer.lock();
	unsigned int size = OutGoingBuffer.getbuffersize();
	OutGoingBuffer.unlock();
	return SERIALBUFFERSIZE - size;
}

void Serial::flush(unsigned int refreshrate){
	while(1){
		OutGoingBuffer.lock("flush");
		if(OutGoingBuffer.getbuffersize() == 0){
			OutGoingBuffer.unlock("flush"); // unlock buffer
			break;
		}
		OutGoingBuffer.unlock("flush");
		// keep mutex unlocked for refreshrate
		std::this_thread::sleep_for(std::chrono::milliseconds(refreshrate));
	}
}

void Serial::clearBuffer(){
	OutGoingBuffer.lock("clearbuffer");
	IncomingBuffer.lock("clearbuffer");
		IncomingBuffer.flushbuffer();
		OutGoingBuffer.flushbuffer();
	OutGoingBuffer.unlock("clearbuffer");
	IncomingBuffer.unlock("clearbuffer");
}

void Serial::setTimeout(SerTimeOut ST){
	OutGoingBuffer.lock("SetTimeout");
	IncomingBuffer.lock("SetTimeout");
	SerialHandleMutex.lock();

	// Create new termios struct, we call it 'tty' for convention
	// No need for "= {0}" at the end as we'll immediately write the existing
	// config to this struct
	struct termios tty;

	// Read in existing settings, and handle any error
	if(tcgetattr(hSerial, &tty) != 0) {
		throw SerialException(
			"Error " + std::to_string(errno) +
			"from tcgetattr: " + std::strerror(errno));
	}

	// set timeout
	tty.c_cc[VTIME] = ST.__VTIME;
	tty.c_cc[VMIN] = ST.__VMIN;

	// Save tty settings, also checking for error
	if (tcsetattr(hSerial, TCSANOW, &tty) != 0) {
		throw SerialException(
			"Error " + std::to_string(errno) +
			"from tcgetattr: " + std::strerror(errno));
	}

	SerialHandleMutex.unlock();
	OutGoingBuffer.unlock("SetTimeout");
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
		length = IncomingBuffer.getbuffersize();
	}
	for(; i < length; i++){
		char temp = IncomingBuffer.pop();
		buffer[i] = temp;
	}
	IncomingBuffer.unlock("readbytes");
	return i;
}

unsigned int Serial::readBytesUntil(char terminator, char* buffer, unsigned int length){
	unsigned int i = 0;
	IncomingBuffer.lock("readbytesuntil");
	if(length == 0){
		length = IncomingBuffer.getbuffersize();
	}
	for(; i < length; i++){
		char temp = IncomingBuffer.pop();
		buffer[i] = temp;
		if(temp == terminator){
			break;
		}
	}
	IncomingBuffer.unlock("readbytesuntil");
	return i;
}

std::string Serial::readString(){
	std::string S;
	IncomingBuffer.lock("readstring");
	while(IncomingBuffer.getbuffersize()){
		S += IncomingBuffer.pop();
	}
	IncomingBuffer.unlock("readstring");
	return S;
}

std::string Serial::readStringUntil(char terminator){
	std::string S;
	char temp;
	IncomingBuffer.lock("readstringuntil");
	do{
		temp = IncomingBuffer.pop();
		S += temp;
	}
	while(temp != terminator && IncomingBuffer.getbuffersize());
	IncomingBuffer.unlock("readstringuntil");
	return S;
}

void Serial::writeByte(char val){
	OutGoingBuffer.lock("writebyte");
	try{
		OutGoingBuffer.push(val);
		OutGoingBuffer.unlock("writebyte");
	}
	catch(Buffer::SerialBufferException const &e){
		OutGoingBuffer.unlock("writebyte");
		for(int i = 0; (i <= 10) && !this->availableForWrite(); i++){
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
			if(i == 10){
				throw Buffer::WriteRetryTimeoutException(val);
			}
		}
	}
}

void Serial::writeStr(std::string str){
	OutGoingBuffer.lock("writestr");
	for(char const &c : str)
	{
		OutGoingBuffer.push(c);
	}
	OutGoingBuffer.unlock("writestr");
}

void Serial::writeBytes(char* buf, unsigned int len){
	OutGoingBuffer.lock("writebytes");
	for(unsigned int i = 0; i < len; i++)
	{
		OutGoingBuffer.push(buf[i]);
	}
	OutGoingBuffer.unlock("writebytes");
}

int Serial::ReadThreadFunc(Buffer::Buffer& IncomingBuffer, int& hSerial, std::mutex& SHMutex){
	while(!IncomingBuffer.StopThread){

		char szBuff[SERIALBUFFERSIZE + 1] = {0};
		// Read bytes. The behaviour of read() (e.g. does it block?,
		// how long does it block for?) depends on the configuration
		// settings above, specifically VMIN and VTIME

		int n = 0; // acts as a buffer to hold byte counts

		memset(&szBuff, '\0', sizeof(szBuff));

		SHMutex.lock();
		ioctl(hSerial, FIONREAD, &n);
		if(n){
			n = read(hSerial, &szBuff, sizeof(szBuff));
		}
		SHMutex.unlock();
		if(!n){
			continue; // read returned 0 or ioctl returned 0 thus continue the loop.
		}
		if (n < 0) {
			// error occurred. Inform user
			std::cout << "Error reading: " << std::strerror(errno) << std::endl;
			//throw SerialException("Error reading: " + std::strerror(errno));
		}
		if(n > 0){
			try{
				IncomingBuffer.lock("readthread");
				for(int i = 0; i < n && !IncomingBuffer.StopThread; i++){
					IncomingBuffer.push(szBuff[i]);
				}
				IncomingBuffer.unlock("readthread");
			}
			catch(Buffer::SerialBufferException const &e){
				std::cerr << e.what() << std::endl;
				std::cout << "Stopping reading thread due to exception " << std::endl;
				IncomingBuffer.unlock("readthread-crash");
				while(!IncomingBuffer.StopThread){}
				return -1;
			}
		}
	}

	return 0;
}

int Serial::SendThreadFunc(Buffer::Buffer& OutGoingBuffer, int& hSerial, std::mutex& SHMutex){
	while(!OutGoingBuffer.StopThread){
		char szBuff[SERIALBUFFERTEMPSIZE] = {0};

		memset(&szBuff, '\0', sizeof(szBuff));

		OutGoingBuffer.lock();
		unsigned int charinbuffer = OutGoingBuffer.getbuffersize();
		OutGoingBuffer.unlock();
		if(charinbuffer > 0){
			OutGoingBuffer.lock();
			try{
				for(unsigned int i = 0; i < charinbuffer && !OutGoingBuffer.StopThread; i++){
					szBuff[i] = OutGoingBuffer.pop();
				}
			}
			catch(Buffer::SerialBufferException const &e){
				std::cerr << e.what() << std::endl;
				std::cout << "Stopping writing thread due to exception " << std::endl;
				OutGoingBuffer.unlock();
				while(!OutGoingBuffer.StopThread){}
				return -1;
			}
			OutGoingBuffer.unlock();

			SHMutex.lock();
			int n = write(hSerial, szBuff, sizeof(szBuff));
			SHMutex.unlock();

			if(n < 0){
				// error occurred. Inform user
				std::cout << "Error writing: " << std::strerror(errno) << std::endl;
				//throw SerialException("Error reading: " << std::strerror(errno));
			}
		}
	}

	return 0;
}