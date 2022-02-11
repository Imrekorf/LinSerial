#pragma once

#include <thread>
#include <mutex>
#include <string>
#include <atomic>

// Linux headers
//#include <fcntl.h> // Contains file controls like O_RDWR
//#include <errno.h> // Error integer and strerror() function
//#include <termios.h> // Contains POSIX terminal control definitions
//#include <unistd.h> // write(), read(), close()

#define SERIALBUFFERSIZE 1024
#define SERIALBYTESREADATONCE 1
#define SERIALBYTESWRITEATONCE 1

namespace LinSer {
	namespace Buffer {
		/**=========================== Buffer Exceptions =========================== **/
		class SerialBufferException: public std::exception {
		public:
			enum class EType{
				Overflow,
				Underflow,
				WriteRetryTimeout
			};
		private:
			std::string message_;
			EType error;
			unsigned char byte;
		public:
			
			explicit SerialBufferException(const std::string& message, const EType E, unsigned char byte) : 
				message_(message), error(E), byte(byte) {}
			/**
			 * @brief c-style string of the exception message
			 * 
			 * @return const char* 
			 */
			const char* what() const noexcept override {
				return message_.c_str();
			}
			/**
			 * @brief Get the Error Type of the exception
			 * 
			 * @return EType 
			 */
			EType GetType(){
				return error;
			}
			/**
			 * @brief The byte that caused the exception
			 * 
			 * @return unsigned char 
			 */
			unsigned char ThrownAtByte(){
				return byte;
			}
		};

		class SerialBufferOverflowException: public SerialBufferException {
		public:
			explicit SerialBufferOverflowException(unsigned char byte) : 
				SerialBufferException("Buffer overflow during push operation", EType::Overflow, byte) {}
		};
		class SerialBufferUnderflowException: public SerialBufferException {
		public:
			explicit SerialBufferUnderflowException(unsigned char byte) : 
				SerialBufferException("Buffer undeflow during pop operation", EType::Underflow, byte) {}
		};
		class WriteRetryTimeoutException: public SerialBufferException {
		public:
			explicit WriteRetryTimeoutException(unsigned char byte) : 
				SerialBufferException("Timeout after 50ms of trying to write character", EType::WriteRetryTimeout, byte) {}
		};




		/**=========================== Buffer logic =========================== **/
		typedef struct __buffer {
			unsigned int	count;
			char 		 	buff[SERIALBUFFERSIZE];
			unsigned int  	front	: 10;
			std::atomic<bool>	StopThread;
			std::mutex	  	Mutex;
			__buffer() : count(0), front(0), StopThread(false) {}
			/**
			 * @brief Pushes a byte to the buffer
			 * 
			 * @param C the byte to be pushed to the buffer
			 * @note can throw a SerialBufferOverflowException 
			 */
			void push(char C);
			/**
			 * @brief Pops a byte to the buffer
			 * 
			 * @returns char the byte popped from the buffer
			 * @note can throw a SerialBufferUnderflowException 
			 */
			char pop(void);
			/**
			 * @brief Flushes the buffer
			 * 
			 */
			void flushbuffer();
			/**
			 * @brief Returns the current size of the buffer
			 * 
			 * @return unsigned int 
			 */
			unsigned int getbuffersize();
		} Buffer;
	}; // end of namespace Buffer




	/**=========================== Serial Exceptions =========================== **/
	class SerialException: public std::exception {
	private:
		std::string message_;

	public:
		explicit SerialException(const std::string& message) : message_(message) {}
		const char* what() const noexcept override {
			return message_.c_str();
		}
	};




	/**=========================== Serial parameters  =========================== **/
	struct SerParam {
		enum class Baudrate {
			// values based on termios.h

			B0 		= 00,
			B50		= 01,
			B75		= 02,
			B110    = 03,
			B134	= 04,
			B150	= 05,
			B200	= 06,
			B300    = 07,
			B600    = 10,
			B1200   = 11,
			B1800	= 12,
			B2400   = 13,
			B4800   = 14,
			B9600   = 15,
			B19200  = 16,
			B38400  = 17,
			B57600  = 0010001,
			B115200 = 0010002,
			B230400	= 0010003,
			B460800 = 0010004,
		};

		enum class StopBits {
			ONE, // clear CSTOPB
			TWO  // set CSTOPB
		};

		// amount of bits per message
		enum class BitCount {
			// values based on termios.h

			CS5 = 00,
			CS6 = 20,
			CS7 = 40,
			CS8 = 60
		};

		enum class Parity {
			NONE,	// clear PARNEB
			EVEN,	// set PARENDB & clear PARODD
			ODD,	// set PARENB & set PARODD
		};

		Baudrate rate 			= Baudrate::B9600;
		Parity P 				= Parity::NONE;
		StopBits SB				= StopBits::ONE;
		BitCount bytesize 		= BitCount::CS8;
		SerParam(Baudrate rate = Baudrate::B9600, Parity P = Parity::NONE, StopBits SB = StopBits::ONE, BitCount bitcount = BitCount::CS8) :
			rate(rate), P(P), SB(SB), bytesize(bytesize) {}
	};

	struct SerTimeOut {
		/* calculated with:
		* ReadIntervalTimeout = max time between bytes
		* ReadTotalTimeoutMultiplier = (MaxTimeOut) / (MaxBytesRead)
		* ReadTotalTimeoutConstant = (MaxTimeOut) - ReadTotalTimeoutMultiplier
		*/

		unsigned char __VMIN = 0;
		unsigned char __VTIME = 1;

		SerTimeOut(unsigned char TimeOutms) {__VTIME = (unsigned char)((float)TimeOutms/100.0f); /* convert ms to deciseconds */}
		
		/**
		 * @brief Construct a new Ser Time Out object
		 * if MinCharCount > 0 && TimeOutms > 0, block until 1st character and then block until TimeOutms
		 * if MinCharCount > 0 && TimeOutms = 0, block until minimum amount of characters received
		 * if MinCharCount = 0 && TimeOutms > 0, block until timeoutms
		 * if MinCharCount = 0 && TimeOutms = 0, don't block, read what is avaible
		 * @param MinCharCount 
		 * @param TimeOutms 
		 */
		SerTimeOut(unsigned char MinCharCount, unsigned char TimeOutms = 0){__VMIN = MinCharCount; __VTIME = (unsigned char)((float)TimeOutms/100.0f);};
	};


	/**=========================== Serial Class  =========================== **/

	/**
	 * @brief Serial interface implementation using arduino style functions
	 * 
	 */
	class Serial {
	private:
		int hSerial = 0;

		Buffer::Buffer IncomingBuffer;
		Buffer::Buffer OutGoingBuffer;
		
		std::mutex	   SerialHandleMutex;	// gets locked whenever a function that uses hSerial is called
		
		static int ReadThreadFunc(Buffer::Buffer& IncomingBuffer, int& hSerial, std::mutex& SHMutex);
		static int SendThreadFunc(Buffer::Buffer& OutGoingBuffer, int& hSerial, std::mutex& SHMutex);

		std::thread ReadThread;
		std::thread SendThread;

	public:
		Serial(const char* Port, SerParam SP = SerParam(), SerTimeOut ST = SerTimeOut(0, 0));
		~Serial();
		
		/**
		 * @brief returns the number of bytes available to read
		 * 
		 * @return unsigned int the number of bytes available to read
		 */
		unsigned int available();

		/**
		 * @brief returns the number of bytes available to write without blocking the write operation
		 * 
		 * @return unsigned int the number of bytes available to write
		 */
		unsigned int availableForWrite();

		/**
		 * @brief waits for transmission of outgoing serial data to complete
		 * 
		 * @param refreshrate refreshrate in milliseconds for internal timer for how quickly to check if the outgoing buffer is empty
		 */
		void flush(unsigned int refreshrate = 5);

		/**
		 * @brief Clears currently stored buffer data, Incoming and Outgoing
		 */
		void clearBuffer();

		/**
		 * @brief Sets the timeout parameters of the serial port
		 * 
		 * @param ST the timeout parameters
		 */
		void setTimeout(SerTimeOut ST);

		/**
		 * @brief returns first byte of incoming serial data available.
		 * 
		 * @return char the read character from serial data.
		 * 
		 * @throws SerialBufferUnderflowException A serial buffer underflow exception when the buffer is empty but a read was attempted.
		 */
		char readByte();
		
		/**
		 * @brief reads incoming buffer into char array
		 * 
		 * @param buffer the buffer to read characters into
		 * @param length the amount of characters to be read, if 0 read all characters from the buffer
		 * 
		 * @return int the amount of read bytes
		 * 
		 * @note Terminates when buffer empty or if it reaches size of length
		 * @throws SerialBufferUnderflowException A serial buffer underflow exception when the buffer is empty but a read was attempted.
		 */
		unsigned int readBytes(char* buffer, unsigned int length = 0);

		/**
		 * @brief reads the Incomming buffer into a char array until the terminator character is read
		 * Terminator character is omitted from the end of the buffer and discarded from the buffer
		 * Terminates when buffer empty or if it reaches size of length
		 * 
		 * @param terminator the terminator character
		 * @param buffer the buffer to read characters into
		 * @param length the amount of characters to read, if 0 read all characters from the buffer
		 * 
		 * @return int the amount of read bytes
		 * 
		 * @throws SerialBufferUnderflowException A serial buffer underflow exception when the buffer is empty but a read was attempted.
		 */
		unsigned int readBytesUntil(char terminator, char* buffer, unsigned int length = 0);
		
		/**
		 * @brief reads the Incoming buffer into a string
		 * 
		 * @return std::string a string of the read bytes
		 * 
		* @throws SerialBufferUnderflowException A serial buffer underflow exception when the buffer is empty but a read was attempted.
		 */
		std::string readString();
		
		/**
		 * @brief reads the Incomming buffer into a char array until the terminator character is read
		 * Terminator character is omitted from the end of the buffer and discarded from the buffer
		 * Terminates when buffer empty or if it reaches terminator character
		 * 
		 * @param terminator the terminator character
		 * 
		 * @return std::string a string of read bytes
		 * 
		 * @throws SerialBufferUnderflowException A serial buffer underflow exception when the buffer is empty but a read was attempted.
		 */
		std::string readStringUntil(char terminator);

		/**
		 * @brief prints data to the serial port as human-readable ASCII text
		 * 
		 * @tparam type the data type of the desired value, should be convertable to string.
		 * @param val the value to be printed
		 */
		template<typename type>
		void print(type val){
			writeStr(std::to_string(val));
		}

		/**
		 * @brief prints data to the serial port as human-readable ASCII text followed by a new line and carriage return or '\n\r'
		 * 
		 * @tparam type type the data type of the desired value, should be convertable to string.
		 * @param val the value to be printed
		 */
		template<typename type>
		void println(type val){
			writeStr(std::to_string(val) + "\n\r");
		}

		/**
		 * @brief sends a single byte
		 * will return before any characters are transmitted over serial.
		 * If the transmit buffer is full then *write()* will block until there is enough space in the buffer.
		 * To avoid blocking calls check the amount of free space in the transmit buffer using *AvailableForWrite()*
		 * 
		 * @param val the value to be send over serial
		 * 
		 * @throws WriteRetryTimeoutException After 10 retries with a delay of 5ms the byte was unable to be send.
		 */
		void writeByte(char val);
		
		/**
		 * @brief sends a string as a series of bytes
		 * will return before any characters are transmitted over serial.
		 * If the transmit buffer is full then write() will block until there is enough space in the buffer.
		 * To avoid blocking calls check the amount of free space in the transmit buffer using *AvailableForWrite()*
		 * @param str the string to be send over serial
		 */
		void writeStr(std::string str);

		/**
		 * @brief writes an array as a series of bytes
		 * will return before any characters are transmitted over serial.
		 * If the transmit buffer is full then write() will block until there is enough space in the buffer.
		 * To avoid blocking calls check the amount of free space in the transmit buffer using *AvailableForWrite()*
		 * 
		 * @param buf buffer to be written over serial
		 * @param len the length of the buffer.
		 */
		void writeBytes(char* buf, unsigned int len);

	};
} // end of namespace WinSer