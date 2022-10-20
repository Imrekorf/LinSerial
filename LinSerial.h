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
#ifndef SERIAL_BUFFER_SIZE
#define SERIAL_BUFFER_SIZE 1024
#endif

/**
 * @brief The amount of time in milliseconds a Read thread sleeps for between trying to read the incoming serial data.
 */
#ifndef SERIAL_READ_SLEEP_TIME
#define SERIAL_READ_SLEEP_TIME 10
#endif

#define LINSER_LOG_ERRORS 1
#define LINSER_LOG_INFO   2
#define LINSER_LOG_DEBUG  4
#define LINSER_LOG_MUTEX  128

/**
 * @brief Enables debug messages for certain functions depending on bits set.
 * All bits clear means no messages will be logged.
 * bit1 = errors.
 * bit2 = information messages.
 * bit3 = debug.
 * bit7 = mutex lock changes, this is used by developers.
 */
#ifndef LINSER_LOG_LEVEL
#define LINSER_LOG_LEVEL (LINSER_LOG_ERRORS | LINSER_LOG_INFO)
#endif
/**@}*/

/** @defgroup SerialBufferMacros */
#define LinSerLogPre(level) printf("[LinSer][%s] ", level)
#if (LINSER_LOG_LEVEL & LINSER_LOG_ERRORS)
	#define linSerLogError(...)   LinSerLogPre("Error"); printf(__VA_ARGS__)
#else
	#define linSerLogError (...)
#endif
#if (LINSER_LOG_LEVEL & LINSER_LOG_INFO) 
	#define linSerLogInfo(...)    LinSerLogPre("Info "); printf(__VA_ARGS__)
#else 
	#define linSerLogInfo(...)
#endif
#if (LINSER_LOG_LEVEL & LINSER_LOG_DEBUG)
	#define linSerLogDebug(...)   LinSerLogPre("Debug"); printf(__VA_ARGS__)
#else
	#define linSerLogDebug(...)
#endif

/**@}*/

namespace linSer {
	namespace buffer {
		/**=========================== Buffer exceptions =========================== **/
		
		/**
		 * @brief Base serial buffer exception.
		 */
		class serialBufferException: public std::exception {
		public:
			enum class buffExcType{
				overflow,
				underflow
			};
		private:
			std::string message;
			buffExcType error;
			unsigned char byte;
		public:
			
			explicit serialBufferException(const std::string& message, const buffExcType err, unsigned char byte) : 
				message(message), error(err), byte(byte) {}
			/**
			 * @brief C-style string of the exception message.
			 * 
			 * @return The C-style exception message.
			 */
			const char* what() const noexcept override {
				return message.c_str();
			}
			/**
			 * @brief Get the error type of the exception.
			 * 
			 * @return The error type 
			 */
			buffExcType getType(){
				return error;
			}
			/**
			 * @brief The byte that caused the exception.
			 * 
			 * @return The byte that caused the exception.
			 */
			unsigned char thrownAtByte(){
				return byte;
			}
		};

		/**
		 * @brief Serial buffer overflow exception when trying to push too much data to a buffer.
		 * Size of the serial buffer is based on SERIAL_BUFFER_SIZE.
		 */
		class serialBufferOverflowException: public serialBufferException {
		public:
			explicit serialBufferOverflowException(unsigned char byte) : 
				serialBufferException("Buffer overflow during push operation", buffExcType::overflow, byte) {}
		};

		/** Serial buffer underflow exception when trying to pop too much data from a buffer */
		class SerialBufferUnderflowException: public serialBufferException {
		public:
			explicit SerialBufferUnderflowException(unsigned char byte) : 
				serialBufferException("Buffer undeflow during pop operation", buffExcType::underflow, byte) {}
		};

		/**=========================== Buffer logic =========================== **/
		class buffer {
		private:
			unsigned int	count;
			char 		 	buff[SERIAL_BUFFER_SIZE];
			unsigned int  	front	: 10;
		public:
			/**
			 * @brief Construct a new __buffer object.
			 */
			buffer() : count(0), front(0) {}
			
			/**
			 * @brief Pushes a byte to the buffer.
			 * 
			 * @param C The byte to be pushed to the buffer.
			 * @throws serialBufferOverflowException when size of buffer = SERIAL_BUFFER_SIZE.
			 */
			void push(char C);

			/**
			 * @brief Pops a byte to the buffer.
			 * 
			 * @returns The byte popped from the buffer.
			 * @throws serialBufferUnderflowException when size of buffer = 0.
			 */
			char pop(void);

			/**
			 * @brief Flushes the buffer.
			 */
			void flushbuffer();

			/**
			 * @brief Returns the current size of the buffer.
			 * 
			 * @return The size of the buffer.
			 */
			inline unsigned int getBufferSize() const;
		};
	}; // end of namespace buffer.




	/**=========================== Serial exceptions =========================== **/
	
	/**
	 * @brief Serial exception thrown when an error with tcgetattr() or open() occurs.
	 */
	class serialException: public std::exception {
	public:
		enum serExcType {
			openError,
			tcgetarrtError,
			tcsetattrError,
			noEOLTimeout,
		};
	private:
		std::string message;
		serExcType error;
	public:
		explicit serialException(const std::string& message, serExcType error) : message(message), error(error) {}
		const char* what() const noexcept override {
			return message.c_str();
		}
	};

	/**
	 * @brief Serial exception thrown when no end of line is received after specified timeout passed.
	 */
	class noEOLTimeoutException: public serialException {
	public:
		explicit noEOLTimeoutException(const size_t timeout_ms) : 
			serialException("No end of line received after " + std::to_string(timeout_ms) + "ms", serExcType::noEOLTimeout) {}
	};


	/**=========================== Serial parameters  =========================== **/
	
	/**
	 * @brief Serial parameter struct containing data about the serial connection. Values based on termios.h.
	 * By default serParam initializes with a baudrate of 9600, no parity checking, 1 stop bit and a bit count of 8.
	 */
	struct serParam {
		/** @brief Baudrate enum for specificing the baudrate. */
		enum baudrate {
			b0 		= 0000000,
			b50		= 0000001,
			b75		= 0000002,
			b110    = 0000003,
			b134	= 0000004,
			b150	= 0000005,
			b200	= 0000006,
			b300    = 0000007,
			b600    = 0000010,
			b1200   = 0000011,
			b1800	= 0000012,
			b2400   = 0000013,
			b4800   = 0000014,
			b9600   = 0000015,
			b19200  = 0000016,
			b38400  = 0000017,
			b57600  = 0010001,
			b115200 = 0010002,
			b230400	= 0010003,
			b460800 = 0010004,
		};

		static std::map<baudrate, int> baudrateString;

		/** @brief Stop bit enum for specifing the amount of stop bits in communication. */
		enum stopBits {
			ONE_STOP = 0, // Clear CSTOPB.
			TWO_STOP = 1  // Set CSTOPB.
		};

		/**
		 * @brief Bit count enum for specifing the amount of bits in a byte. 
		 */
		enum bitCount {
			BIT_COUNT_5 = 00,
			BIT_COUNT_6 = 20,
			BIT_COUNT_7 = 40,
			BIT_COUNT_8 = 60
		};

		/**
		 * @brief Parity enum for specifing the parity control.
		 */
		enum parity {
			PAR_NONE = 0,	// Clear PARENB.
			PAR_EVEN = 1,	// Set PARENB & clear PARODD.
			PAR_ODD  = 2,	// Set PARENB & set PARODD.
		};

		baudrate rate 			= baudrate::b9600;
		parity 	 parity_ 		= parity::PAR_NONE;
		stopBits stopBits_		= stopBits::ONE_STOP;
		bitCount bitCount_ 		= bitCount::BIT_COUNT_8;
		serParam(baudrate rate = baudrate::b9600, parity par = parity::PAR_NONE, stopBits stop = stopBits::ONE_STOP, bitCount bits = bitCount::BIT_COUNT_8) :
			rate(rate), parity_(par), stopBits_(stop), bitCount_(bits) {}
	};

	/**
	 * @brief Serial timeout struct containing data about serial timeout.
	 */
	struct serTimeout {
		unsigned char __VMIN = 0;
		unsigned char __VTIME = 1;
		serTimeout() {}

		serTimeout(unsigned char timeout_ms) {__VTIME = (unsigned char)((float)timeout_ms*0.1f); /* convert ms to deciseconds. */}
		
		/**
		 * @brief Construct a new serTimeout object.
		 * If minCharCount > 0 && timeout_ms > 0, block until either minCharCount has been received or timeout_ms after first character has elapsed.
		 * If minCharCount > 0 && timeout_ms = 0, block until minimum amount of characters received.
		 * If minCharCount = 0 && timeout_ms > 0, block until timeout_ms.
		 * If minCharCount = 0 && timeout_ms = 0, don't block, read what is avaible.
		 * 
		 * @param minCharCount minimum amount of characters to read before unblocking.
		 * @param timeout_ms   amount of time to wait for an incoming message in ms.
		 */
		serTimeout(unsigned int minCharCount, unsigned int timeout_ms = 0){__VMIN = minCharCount; __VTIME = (unsigned char)((float)timeout_ms*0.1f);};
	};


	/**=========================== Serial Class  =========================== **/

	/**
	 * @brief Serial interface implementation using arduino style functions.
	 * 
	 */
	class serial {
	private:
		int hSerial = 0;

		bool		   stopThread = false;		// bool to stop receiving thread.
		buffer::buffer incomingBuffer;			// Buffer for storing incoming data.
		std::mutex	   incomingBufferMutex; 	// Mutex used to lock incoming buffer.
		std::thread    incomingThread;         // Thread for receiving incoming data.
		
		std::mutex	   serialHandleMutex;		// Gets locked whenever a function that uses hSerial is called.

		static int _readThreadFunc(serial& self);

	public:
		/**
		 * @brief Construct a new Serial object, starting one thread for reading.
		 * 
		 * @param port A C-style string name of the serial port to be read.
		 * @param serPar The serial parameter struct.
		 * @param SerTim The serial timeout struct.
		 * 
		 * @throws serialException if open() or tcgetattr() fails. Use what() to get error message.
		 */
		serial(const char* port, const serParam& serPar = serParam(), const serTimeout& SerTim = serTimeout());
		
		/**
		 * @brief Destroy the serial object, also closes the read thread.
		 */
		~serial();
		
		/**
		 * @brief Returns the number of bytes available to read.
		 * 
		 * @return The number of bytes available to read.
		 */
		unsigned int available();

		/**
		 * @brief Clears currently stored buffer data.
		 */
		void clearBuffer();

		/**
		 * @brief Sets the timeout parameters of the serial port.
		 * 
		 * @param serTim The timeout parameters.
		 * @throws serialException when tcgetattr() or tcsetattr() fails.
		 */
		void setTimeout(const serTimeout& serTim);

		/**
		 * @brief Returns first byte of incoming serial data available.
		 * 
		 * @return The read character from serial data.
		 * 
		 * @throws serialBufferUnderflowException when the buffer is empty but a read was attempted.
		 */
		char readByte();
		
		/**
		 * @brief Reads incoming buffer into char array.
		 * Terminates when buffer empty or if it reaches size of length.
		 * 
		 * @param[out] buffer The buffer to read characters into, should be allocated to atleast the size of length parameter.
		 * @param[in]  length The amount of characters to be read, if 0 read all characters from the buffer.
		 * 
		 * @return The amount of read bytes.
		 */
		unsigned int readBytes(char* buffer, unsigned int length = 0);

		/**
		 * @brief Reads the incoming buffer into a char array until the terminator character is read.
		 * Terminates when buffer empty or if it reaches size of length.
		 * 
		 * @param[out] buffer The buffer to read characters into.
		 * @param[in]  terminator The terminator character.
		 * @param[in]  length The amount of characters to read, if 0 read all characters from the buffer.
		 * 
		 * @return The amount of read bytes.
		 */
		unsigned int readBytesUntil(char* buffer, const char terminator, unsigned int length = 0);
		
		/**
		 * @brief Reads a line until lineEnd from buffer. If lineEnd is not in buffer, hangs until it is received or until timeout_ms has passed.
		 * 
		 * @param[in] lineEnd The string that represents the end of the line.
		 * @param[in] timeout_ms The maximum amount of time in milliseconds to wait if lineEnd character was not in present buffer.
		 * @return The read line.
		 * 
		 * @throws noEOLTimeoutException when lineEnd is not received after timeout_ms.
		 */
		std::string readLine(const std::string& lineEnd = "\n", size_t timeout_ms = 500);

		/**
		 * @brief Reads the incoming buffer into a string.
		 * 
		 * @return The read bytes as a string.
		 */
		std::string readString();
		
		/**
		 * @brief Reads the incoming buffer into a string until the terminator character is read.
		 * Terminator character is omitted from the end of the buffer and discarded from the buffer.
		 * Terminates when buffer empty or if it reaches terminator character.
		 * 
		 * @param[in] terminator the terminator character.
		 * 
		 * @return The read bytes as a string.
		 */
		std::string readStringUntil(const char terminator);

		/**
		 * @brief Reads the incoming buffer into a string until the substring is read.
		 * Terminates when buffer empty or if a match with substr is made.
		 * 
		 * @param[in] substr the substring to read until.
		 * @return The read bytes as a string.
		 */
		std::string readStringUntil(const std::string& substr);

		template<typename T>
		class isStreamable {
			template<typename TT>
			static auto supportsToStreamTest(int)
			-> decltype( std::declval<std::stringstream&>() << std::declval<TT>(), std::true_type() );

			template<typename, typename>
			static auto supportsToStreamTest(...) -> std::false_type;
		public:
			static const bool value = decltype(supportsToStreamTest<T>(0))::value;
		};

		/**
		 * @brief Prints data to the serial port as human-readable ASCII text.
		 * 
		 * @tparam type The data type of the desired value, should be convertable to string.
		 * @param[in] val The value to be printed.
		 */
		template<typename type>
		typename std::enable_if<isStreamable<type>::value, void>::type 
		  print(type val){
			std::stringstream convert; 
			convert << val;
			writeStr(convert.str());
		}

		
		/**
		 * @brief Prints data to the serial port as human-readable ASCII text followed by specified newline sequence.
		 * 
		 * @tparam type The data type of the desired value, should be convertable to string.
		 * @param[in] val The value to be printed.
		 */
		template<typename type>
		typename std::enable_if<isStreamable<type>::value, void>::type 
		  println(type val, std::string ln = "\r\n"){
			print<type>(val);
			writeStr(ln);
		}

		/**
		 * @brief Sends a single byte.
		 *
		 * @param[in] val The value to be send over serial.
		 * 
		 */
		void writeByte(const char val);
		
		/**
		 * @brief Sends a string as a series of bytes.
		 * @param[in] str The string to be send over serial.
		 */
		void writeStr(const std::string str);

		/**
		 * @brief Writes an array as a series of bytes.
		 * 
		 * @param[in] buf Buffer to be written to serial.
		 * @param[in] len The length of the buffer.
		 */
		void writeBytes(const char* buf, const unsigned int len);

		/**
		 * @brief Prints data to the serial port as human-readable ASCII text followed by specified newline sequence.
		 * 
		 * @tparam T The data type of the desired value, should be convertable to string.
		 * @param val The value to be printed.
		 * @return serial& reference to the serial object.
		 */
		template<typename T>
		serial& operator<<(const T& val){
			print(val);
			return *this;
		}

	};
} // end of namespace LinSer.
