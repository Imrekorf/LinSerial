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
	Mutex.lock();
	if(count >= SERIALBUFFERSIZE){ throw SerialBufferOverflowException(c); return; }
	buff[count + front] = c;
	count++;
	Mutex.unlock();
}

char Buffer::__buffer::pop(void){
	Mutex.lock();
	if(!count){ throw SerialBufferUnderflowException(0); return 0;}
	front++;
	count--;
	return buff[front-1];
	Mutex.unlock();
}

void Buffer::__buffer::flushbuffer(){
	Mutex.lock();
	count = 0;
	front = 0;
	Mutex.unlock();
}

unsigned int Buffer::__buffer::getbuffersize(){
	unsigned int size = 0;
	Mutex.lock();
	size = count;
	Mutex.unlock();
	return size;
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
	if(SP.P == SerParam::Parity::NONE)
		tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity
	if(SP.P == SerParam::Parity::EVEN)
		tty.c_cflag |= PARENB;  // Set parity bit, enabling even parity
	if(SP.P == SerParam::Parity::ODD){
		tty.c_cflag |= PARENB;  // Set parity bit, enabling parity
		tty.c_cflag |= PARODD;  // Set parity odd bit, enabling odd parity
	}
	// StopBits
	if(SP.SB == SerParam::StopBits::ONE)
		tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication
	if(SP.SB == SerParam::StopBits::TWO)
		tty.c_cflag |= CSTOPB;  // Set stop field, two stop bits used in communication
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
	tty.c_cc[VTIME] = ST.__VTIME;
	tty.c_cc[VMIN] = ST.__VMIN;

	// Set in/out baud rate to be baudrate
	cfsetispeed(&tty, (speed_t)SP.rate);
	cfsetospeed(&tty, (speed_t)SP.rate);

	// Save tty settings, also checking for error
	if (tcsetattr(hSerial, TCSANOW, &tty) != 0) {
		throw SerialException(
			"Error " + std::to_string(errno) +
			"from tcgetattr: " + std::strerror(errno));
	}

	// start threads
	ReadThread = std::thread(ReadThreadFunc, std::ref(IncomingBuffer), std::ref(hSerial));
	SendThread = std::thread(SendThreadFunc, std::ref(OutGoingBuffer), std::ref(hSerial));
}

Serial::~Serial(){
	OutGoingBuffer.StopThread = true;
	IncomingBuffer.StopThread = true;

	ReadThread.join();
	SendThread.join();

	SerialHandleMutex.lock();
	close(hSerial);
	SerialHandleMutex.unlock();
	hSerial = 0;
}

unsigned int Serial::available(){
	return IncomingBuffer.count;
}

unsigned int Serial::availableForWrite(){
	return SERIALBUFFERSIZE - OutGoingBuffer.count;
}

void Serial::flush(unsigned int refreshrate){
	while(1){
		OutGoingBuffer.Mutex.lock();
		if(OutGoingBuffer.count == 0){
			break;
		}
		OutGoingBuffer.Mutex.unlock();
		// keep mutex unlocked for refreshrate
		std::this_thread::sleep_for(std::chrono::milliseconds(refreshrate));
	}
}

void Serial::clearBuffer(){
	IncomingBuffer.flushbuffer();
	OutGoingBuffer.flushbuffer();
}

void Serial::setTimeout(SerTimeOut ST){
	OutGoingBuffer.Mutex.lock();
	IncomingBuffer.Mutex.lock();
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
	OutGoingBuffer.Mutex.unlock();
	IncomingBuffer.Mutex.unlock();
}

char Serial::readByte(){
	char temp = IncomingBuffer.pop();
}

unsigned int Serial::readBytes(char* buffer, unsigned int length){
	unsigned int i = 0;
	if(length == 0){
		length = IncomingBuffer.getbuffersize();
	}
	for(; i < length; i++){
		char temp = this->readByte();
		buffer[i] = temp;
	}
	return i;
}

unsigned int Serial::readBytesUntil(char terminator, char* buffer, unsigned int length){
	unsigned int i = 0;
	if(length == 0){
		length = IncomingBuffer.getbuffersize();
	}
	for(; i < length; i++){
		char temp = this->readByte();	
		buffer[i] = temp;
		if(temp == terminator){
			break;
		}
	}
	return i;
}

std::string Serial::readString(){
	std::string S;
	while(IncomingBuffer.getbuffersize()){
		S += this->readByte();
	}
	return S;
}

std::string Serial::readStringUntil(char terminator){
	std::string S;
	char temp;
	do{
		temp = this->readByte();
		S += temp;
	}
	while(temp != terminator && IncomingBuffer.getbuffersize());
	return S;
}

void Serial::writeByte(char val){
	try{
		OutGoingBuffer.push(val);
	}
	catch(Buffer::SerialBufferException const &e){
		for(int i = 0; (i <= 10) && !this->availableForWrite(); i++){
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
			if(i == 10){
				throw Buffer::WriteRetryTimeoutException(val);
			}
		}
	}
}

void Serial::writeStr(std::string str){
	for(char const &c : str)
	{
		writeByte(c);
	}
}

void Serial::writeBytes(char* buf, unsigned int len){
	for(int i = 0; i < len; i++)
	{
		writeByte(buf[i]);
	}
}

int Serial::ReadThreadFunc(Buffer::Buffer& IncomingBuffer, int& hSerial, std::mutex& SHMutex){
	while(!IncomingBuffer.StopThread){
		char szBuff[SERIALBYTESREADATONCE + 1] = {0};
		// Read bytes. The behaviour of read() (e.g. does it block?,
		// how long does it block for?) depends on the configuration
		// settings above, specifically VMIN and VTIME

		int n; // acts as a buffer to hold byte counts

		SHMutex.lock();
		ioctl(hSerial, TIOCMGET, &n);
		if(n){
			int n = read(hSerial, &szBuff, sizeof(szBuff));
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
			IncomingBuffer.Mutex.lock();
			for(int i = 0; i < SERIALBYTESREADATONCE; i++){
				IncomingBuffer.push(szBuff[i]);
			}
			IncomingBuffer.Mutex.unlock();
		}
	}

	return 0;
}

int Serial::SendThreadFunc(Buffer::Buffer& OutGoingBuffer, int& hSerial, std::mutex& SHMutex){
	while(!OutGoingBuffer.StopThread){
		char szBuff[SERIALBYTESWRITEATONCE + 1] = {0};

		if(OutGoingBuffer.count > 0){
			OutGoingBuffer.Mutex.lock();
			for(int i = 0; i < SERIALBYTESWRITEATONCE; i++){
				szBuff[i] = OutGoingBuffer.pop();
			}
			OutGoingBuffer.Mutex.unlock();
		}
		
		SHMutex.lock();
		int n = write(hSerial, szBuff, sizeof(szBuff));
		SHMutex.unlock();

		if(n < 0){
			// error occurred. Inform user
			std::cout << "Error writing: " << std::strerror(errno) << std::endl;
			//throw SerialException("Error reading: " << std::strerror(errno));
		}
	}

	return 0;
}