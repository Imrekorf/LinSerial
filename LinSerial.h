#pragma once

#include <thread>
#include <mutex>
#include <string>
#include <atomic>
#include <iostream>
#include <map>

#include <utility>
#include <type_traits>
#include <sstream>

/** @defgroup SerialBufferDefines */
/**@{*/

/**
 * @brief size of the FIFO buffer 
 */
#ifndef SERIALBUFFERSIZE
#define SERIALBUFFERSIZE 1024
#endif

/**
 * @brief The amount of time in milliseconds a Read thread sleeps for between trying to read the incoming serial data
 */
#ifndef SERIALREADSLEEPTIME
#define SERIALREADSLEEPTIME 10
#endif

/**
 * @brief Enables debug messages for certain functions
 * 0 = no debug
 * 1 = errors
 * 2 = Serial setup messages
 * 3 = debug
 * 4 = debug + mutex lock changes
 */
#ifndef LINSERDEBUG
#define LINSERDEBUG 2
#endif
/**@}*/

namespace LinSer {
	namespace Buffer {
		/**=========================== Buffer Exceptions =========================== **/
		
		/**
		 * @brief Base Serial buffer exception.
		 */
		class SerialBufferException: public std::exception {
		public:
			enum class BEType{
				Overflow,
				Underflow
			};
		private:
			std::string message_;
			BEType error;
			unsigned char byte;
		public:
			
			explicit SerialBufferException(const std::string& message, const BEType E, unsigned char byte) : 
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
			BEType GetType(){
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

		/**
		 * @brief Serial buffer overflow exception when trying to push too much data to a buffer
		 * Mas size of the Serial buffer is based on SERIALBUFFERSIZE
		 */
		class SerialBufferOverflowException: public SerialBufferException {
		public:
			explicit SerialBufferOverflowException(unsigned char byte) : 
				SerialBufferException("Buffer overflow during push operation", BEType::Overflow, byte) {}
		};

		/**
		 * @brief Serial buffer underflow exception when trying to pop too much data from a buffer
		 * 
		 */
		class SerialBufferUnderflowException: public SerialBufferException {
		public:
			explicit SerialBufferUnderflowException(unsigned char byte) : 
				SerialBufferException("Buffer undeflow during pop operation", BEType::Underflow, byte) {}
		};

		/**=========================== Buffer logic =========================== **/
		typedef struct __buffer {
			unsigned int	count;
			char 		 	buff[SERIALBUFFERSIZE];
			unsigned int  	front	: 10;
			bool			StopThread;
			std::mutex	  	Mutex;

			std::thread ThreadFunc;

			/**
			 * @brief Construct a new __buffer object
			 */
			__buffer() : count(0), front(0), StopThread(false) {}
			
			/**
			 * @brief Pushes a byte to the buffer
			 * 
			 * @param C the byte to be pushed to the buffer
			 * @throws a SerialBufferOverflowException when size of buffer = SERIALBUFFERSIZE
			 */
			void push(char C);

			/**
			 * @brief Pops a byte to the buffer
			 * 
			 * @returns char the byte popped from the buffer
			 * @throws a SerialBufferUnderflowException when size of buffer = 0
			 */
			char pop(void);

			/**
			 * @brief Flushes the buffer
			 */
			void flushbuffer();

			/**
			 * @brief Returns the current size of the buffer
			 * 
			 * @return unsigned int 
			 */
			inline unsigned int getBufferSize() const;

			/**
			 * @brief Locks this buffer's mutex
			 * 
			 * @param indicator If debugging is enabled by defining _LINSERDEBUGMUTEX then an extra message is printed as to which process locks the mutex
			 */
			void lock(const char* indicator);

			/**
			 * @brief Unlocks this buffer's mutex
			 * 
			 * @param indicator If debugging is enabled by defining _LINSERDEBUGMUTEX then an extra message is printed as to which process unlocks the mutex
			 */
			void unlock(const char* indicator);
		} Buffer;
	} // end of namespace Buffer




	/**=========================== Serial Exceptions =========================== **/
	
	/**
	 * @brief Serial exception thrown when an error with tcgetattr() or open() occurs
	 */
	class SerialException: public std::exception {
	public:
		enum SEType {
			openError,
			tcgetarrtError,
			tcsetattrError,
			NoEOLTimeout,
		};
	private:
		std::string message_;
		SEType error_;
	public:
		explicit SerialException(const std::string& message, SEType error) : message_(message), error_(error) {}
		const char* what() const noexcept override {
			return message_.c_str();
		}
	};

	/**
	 * @brief Throws this exception if after timeout time end of line still was not received
	 */
	class NoEOLTimeoutException: public SerialException {
	public:
		explicit NoEOLTimeoutException() : 
			SerialException("Timout after timeout ms, no end of line received", SEType::NoEOLTimeout) {}
	};




	/**=========================== Serial parameters  =========================== **/
	
	/**
	 * @brief Serial Parameter struct containing data about serialconnection. Values based on termios.h
	 * By default SerParam initializes with a baudrate of 9600, no parity checking, 1 stop bit and a byte size of 8.
	 */
	struct SerParam {
		/**
		 * @brief Baudrate enum for specificing the baudrate. 
		 */
		enum Baudrate {
			Baud0 		= 0000000,
			Baud50		= 0000001,
			Baud75		= 0000002,
			Baud110    	= 0000003,
			Baud134		= 0000004,
			Baud150		= 0000005,
			Baud200		= 0000006,
			Baud300     = 0000007,
			Baud600     = 0000010,
			Baud1200    = 0000011,
			Baud1800	= 0000012,
			Baud2400    = 0000013,
			Baud4800    = 0000014,
			Baud9600    = 0000015,
			Baud19200   = 0000016,
			Baud38400   = 0000017,
			Baud57600   = 0010001,
			Baud115200  = 0010002,
			Baud230400	= 0010003,
			Baud460800  = 0010004,
		};

		/**
		 * @brief Stop bit enum for specifing the amount of stop bits in communication
		 */
		enum StopBits {
			ONE_STOP, // clear CSTOPB
			TWO_STOP  // set CSTOPB
		};

		/**
		 * @brief Bitcount enum for specifing the amount of bits in a message. 
		 */
		enum BitCount {
			// values based on termios.h
			ByteSize5 = 00,
			ByteSize6 = 20,
			ByteSize7 = 40,
			ByteSize8 = 60
		};

		/**
		 * @brief Parity enum for specifing the parity control
		 */
		enum Parity {
			PAR_NONE,	// clear PARNEB
			PAR_EVEN,	// set PARENDB & clear PARODD
			PAR_ODD,	// set PARENB & set PARODD
		};

		Baudrate rate 			= Baudrate::Baud9600;
		Parity P 				= Parity::PAR_NONE;
		StopBits SB				= StopBits::ONE_STOP;
		BitCount bytesize 		= BitCount::ByteSize8;
		SerParam(Baudrate rate = Baudrate::Baud9600, Parity P = Parity::PAR_NONE, StopBits SB = StopBits::ONE_STOP, BitCount bitcount = BitCount::ByteSize8) :
			rate(rate), P(P), SB(SB), bytesize(bitcount) {}
	};

	/**
	 * @brief Serial Timeout struct containing data about serial timeout.
	 */
	struct SerTimeOut {
		/* calculated with:
		* ReadIntervalTimeout = max time between bytes
		* ReadTotalTimeoutMultiplier = (MaxTimeOut) / (MaxBytesRead)
		* ReadTotalTimeoutConstant = (MaxTimeOut) - ReadTotalTimeoutMultiplier
		*/

		unsigned char __VMIN = 0;
		unsigned char __VTIME = 1;
		SerTimeOut() {}

		SerTimeOut(unsigned char TimeOutms) {__VTIME = (unsigned char)((float)TimeOutms/100.0f); /* convert ms to deciseconds */}
		
		/**
		 * @brief Construct a new Ser Time Out object
		 * if MinCharCount > 0 && TimeOutms > 0, block until either MinCharCount has been received or Timeoutms after first character has elapsed.
		 * if MinCharCount > 0 && TimeOutms = 0, block until minimum amount of characters received
		 * if MinCharCount = 0 && TimeOutms > 0, block until timeoutms
		 * if MinCharCount = 0 && TimeOutms = 0, don't block, read what is avaible
		 * @param MinCharCount 
		 * @param TimeOutms 
		 */
		SerTimeOut(unsigned int MinCharCount, unsigned int TimeOutms = 0){__VMIN = MinCharCount; __VTIME = (unsigned char)((float)TimeOutms/100.0f);};
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
		
		std::mutex	   SerialHandleMutex;	// gets locked whenever a function that uses hSerial is called

		static int ReadThreadFunc(Buffer::Buffer& IncomingBuffer, int& hSerial, std::mutex& SHMutex);

	public:
		/**
		 * @brief Construct a new Serial object, starting two threads for reading and writing
		 * 
		 * @param Port a c style string name of the serial port to be read
		 * @param SP the serial parameter struct
		 * @param ST the serial timeout struct
		 * 
		 * @throws SerialException if open() or tcgetattr() fails. Use what() to get error message
		 */
		Serial(const char* Port, const SerParam& SP = SerParam(), const SerTimeOut& ST = SerTimeOut());
		
		/**
		 * @brief Destroy the Serial object, also closes the read and write thread
		 */
		~Serial();
		
		/**
		 * @brief returns the number of bytes available to read
		 * 
		 * @return unsigned int the number of bytes available to read
		 */
		unsigned int available();

		/**
		 * @brief Clears currently stored buffer data
		 */
		void clearBuffer();

		/**
		 * @brief Sets the timeout parameters of the serial port
		 * 
		 * @param ST the timeout parameters
		 * @throws SerialException when tcgetattr() or tcsetattr() fails.
		 */
		void setTimeout(const SerTimeOut& ST);

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
		 * Terminates when buffer empty or if it reaches size of length
		 * 
		 * @param[out] buffer the buffer to read characters into
		 * @param[in]  length the amount of characters to be read, if 0 read all characters from the buffer
		 * 
		 * @return unsigned int the amount of read bytes
		 * 
		 */
		unsigned int readBytes(char* buffer, unsigned int length = 0);

		/**
		 * @brief reads the Incomming buffer into a char array until the terminator character is read
		 * Terminates when buffer empty or if it reaches size of length
		 * 
		 * @param[out] buffer the buffer to read characters into
		 * @param[in]  terminator the terminator character
		 * @param[in]  length the amount of characters to read, if 0 read all characters from the buffer
		 * 
		 * @return int the amount of read bytes
		 */
		unsigned int readBytesUntil(char* buffer, const char terminator, unsigned int length = 0);
		
		/**
		 * @brief Reads a line until line_end from buffer. If line_end is not in buffer, hangs until it is received based on timeout.
		 * 
		 * @param[in] line_end The string that represents the end of the line
		 * @param[in] timeout_ms the maximum amount of time in milliseconds to wait if line_end character was not in present buffer
		 * @return std::string the read line
		 * 
		 * @throws NoEOLTimeoutException exception is line_end is not received after timeout_ms
		 */
		std::string readLine(const std::string& line_end = "\n", int64_t timeout_ms = 500);

		/**
		 * @brief reads the Incoming buffer into a string
		 * 
		 * @return std::string a string of the read bytes
		 */
		std::string readString();
		
		/**
		 * @brief reads the Incomming buffer into a string until the terminator character is read
		 * Terminator character is omitted from the end of the buffer and discarded from the buffer
		 * Terminates when buffer empty or if it reaches terminator character
		 * 
		 * @param[in] terminator the terminator character
		 * 
		 * @return std::string a string of read bytes
		 */
		std::string readStringUntil(const char terminator);

		/**
		 * @brief reads the Incomming buffer into a string until the substring is read
		 * Terminates when buffer empty or if a match with substr is made
		 * 
		 * @param[in] substr the substring to read until
		 * @return std::string the read string
		 */
		std::string readStringUntil(const std::string& substr);

		template<typename T>
		class is_streamable {
			template<typename TT>
			static auto supports_to_stream_test(int)
			-> decltype( std::declval<std::stringstream&>() << std::declval<TT>(), std::true_type() );

			template<typename, typename>
			static auto supports_to_stream_test(...) -> std::false_type;
		public:
			static const bool value = decltype(supports_to_stream_test<T>(0))::value;
		};

		/**
		 * @brief prints data to the serial port as human-readable ASCII text
		 * 
		 * @tparam type the data type of the desired value, should be convertable to string.
		 * @param[in] val the value to be printed
		 */
		template<typename type>
		typename std::enable_if<is_streamable<type>::value, void>::type 
		  print(type val){
			std::stringstream convert; 
			convert << val;
			writeStr(convert.str());
		}

		
		/**
		 * @brief prints data to the serial port as human-readable ASCII text followed by a new line and carriage return or '\n\r'
		 * 
		 * @tparam type type the data type of the desired value, should be convertable to string.
		 * @param[in] val the value to be printed
		 */
		template<typename type>
		typename std::enable_if<is_streamable<type>::value, void>::type 
		  println(type val, std::string ln = "\r\n"){
			print<type>(val);
			writeStr(ln);
		}

		/**
		 * @brief sends a single byte
		 *
		 * @param[in] val the value to be send over serial
		 * 
		 */
		void writeByte(const char val);
		
		/**
		 * @brief sends a string as a series of bytes
		 * @param[in] str the string to be send over serial
		 */
		void writeStr(const std::string str);

		/**
		 * @brief writes an array as a series of bytes
		 * 
		 * @param[in] buf buffer to be written to serial
		 * @param[in] len the length of the buffer.
		 */
		void writeBytes(const char* buf, const unsigned int len);

		template<typename T>
		Serial& operator<<(const T& val){
			print(val);
			return *this;
		}

	};
} // end of namespace WinSer
