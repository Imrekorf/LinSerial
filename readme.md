# Linux Serial C++ library

Arduino style Linux Serial C++ library

Create a serial object by passing the serial port name to the constructor
```C++
LinSer::Serial port("/dev/ttyACM0");
```
In the constructor you can also pass different parameters like the baudrate, parity, stopbits and bitcount by passing a SerParam struct. Timeout settings can also be added with the SerTimeOut struct. The timeout settings can be changed during runtime with the setTimeout function.
```C++
// default values for the SerParam and SerTimeOut struct expanded:
LinSer::Serial port("/dev/ttyACM0", LinSer::SerParam(
	LinSer::SerParam::Baudrate::Baud9600,
	LinSer::SerParam::Parity::PAR_NONE,
	LINSer::SerParam::Stopbits::ONE_STOP,
	LINSer::SerParam::Bitcount::ByteSize8
), LinSer::SerTimeOut(0, 100));
```

## define constants
* Serial reading is based on a reader thread polling the serial port for new data. The frequency of data polling is based on the *SERIALREADSLEEPTIME* define constant. To change this pass a ```-DSERIALREADSLEEPTIME=Value``` flag to the compiler. Frequency can be then determined by ![x=\frac{1}{Value * 1000}](https://latex.codecogs.com/svg.latex?x=\frac{1}{Value*1000})<br>
by default value is 10 thus having a frequency of 100Hz.<br>
* The reader thread will push any data received into a fifo buffer which size is based on the *SERIALBUFFERSIZE* define constant. To change this pass a ```-DSERIALBUFFERSIZE=Value``` flag to the compiler. Value is the size of the buffer in bytes, by default 1024 bytes.<br>
* Debug information printed can be changed by passing the ```-DLINSERDEBUG=Value``` flag to the compiler. Value can be one of the following:

| Value | information printed |
| ---- | --- |
| 0 | No debug information |
| 1 | Only error information is printed |
| 2 | Serial setup information is printed |
| 3 | Extra debug information is printed |
| 4 | Extra debug information is printed as well as internal mutex locks |

*Setting 4 can be used to debug possible race conditions.*

## Functions

Functions available:
| function | description | exception |
| -------- | ----------- | --------- | 
| available | Returns the bytes available in the incoming buffer | - |
| clearBuffer | Clears the incoming buffer | - |
| setTimeout | Sets the timeout settings | SerialException when tcgetattr() or tcsetattr() fails |
| readByte | returns the first byte of the incoming serial buffer | SerialBufferUnderflowException when no bytes are available |
| readBytes | Reads length amount of bytes from the incoming serial buffer, or the entire incoming serial buffer if length is not specified. Buffer to read the bytes into should be passed to the function. Returns the bytes read. | - |
| readBytesUntil | Functions the same as readBytes but stops reading if the terminator is read, or if no more bytes are available | - |
| readLine | Reads a line from the incoming buffer. Line end is specified by the line_end parameter. A timeout can be specified to wait if the line_end was not immediately available | NoEOLTimeoutException if line_end was not received after timeout_ms |
| readString | reads the currently available buffer into a string object. Can return an empty string if buffer is empty | - |
| readStringUntil | Functions the same as readString but stops reading if the terminator/substring is read, or if no more bytes are available |
| print | converts type T to string and writes as string to serial port | - |
| println | Functions the same as print but adds a line end. Line end is by default \r\n but can be overridden | - |
| writeByte | Writes a single byte to the serial port | - |
| writeStr | Writes a string to the serial port | - |
| writeBytes | writes a byte buffer to the serial port | - |

Serial port also supports constructs like:
```C++
port << "Write text as a stream\n\r";
```
note that std::endl is not supported.