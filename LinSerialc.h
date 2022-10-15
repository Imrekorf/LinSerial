#ifndef __LINSERIALC_H__
#define __LINSERIALC_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	SER_BAUD0 		= 0000000,
	SER_BAUD50		= 0000001,
	SER_BAUD75		= 0000002,
	SER_BAUD134		= 0000004,
	SER_BAUD150		= 0000005,
	SER_BAUD200		= 0000006,
	SER_BAUD300     = 0000007,
	SER_BAUD600     = 0000010,
	SER_BAUD1200    = 0000011,
	SER_BAUD1800	= 0000012,
	SER_BAUD2400    = 0000013,
	SER_BAUD4800    = 0000014,
	SER_BAUD9600    = 0000015,
	SER_BAUD19200   = 0000016,
	SER_BAUD38400   = 0000017,
	SER_BAUD57600   = 0010001,
	SER_BAUD115200  = 0010002,
	SER_BAUD230400	= 0010003,
	SER_BAUD460800  = 0010004,
} SerBaudrate;

typedef enum {
	ONE_STOP = 0, // clear CSTOPB
	TWO_STOP = 1 // set CSTOPB
} SerStopBits;

typedef enum  {
	// values based on termios.h
	SER_BYTE_SIZE5 = 00,
	SER_BYTE_SIZE6 = 20,
	SER_BYTE_SIZE7 = 40,
	SER_BYTE_SIZE8 = 60
} SerBitCount;

typedef enum {
	SER_PAR_NONE = 0,	// clear PARNEB
	SER_PAR_EVEN = 1,	// set PARENDB & clear PARODD
	SER_PAR_ODD  = 2,	// set PARENB & set PARODD
} SerParity;

typedef enum {
	SER_SUCCESS,
	SER_ERR_OPEN,
	SER_ERR_TC_GET_ATTR,
	SER_ERR_TC_SET_ATTR,
	SER_NO_EOL_TIMEOUT
} SerError;

typedef struct {
	SerBaudrate rate;
	SerParity	parity;
	SerStopBits stopBits;
	SerBitCount byteSize;
} SerParam;

typedef struct {
	/**
	 * if VMIN > 0 && VTIME > 0, block until either __VMIN has been received or VTIME * 10ms after first character has elapsed.
	 * if VMIN > 0 && VTIME = 0, block until minimum amount of characters received
	 * if VMIN = 0 && VTIME > 0, block until VTIME * 10ms
	 * if VMIN = 0 && VTIME = 0, don't block, read what is avaible
	 */
	unsigned char VMIN = 0;
	unsigned char VTIME = 1;	// timeout in deciseconds (10ms)
} SerTimeOut;

typedef struct {
	int hSerial;
	SerParam 	SParam;
	SerParity	SParity;
	SerTimeOut	STimeOut;
} SerLinuxData;

#ifdef __cplusplus
}
#endif

#endif