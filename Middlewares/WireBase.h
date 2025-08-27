#ifdef __cplusplus

#ifndef _WIREBASE_H_
#define _WIREBASE_H_
#include "hw_config.h"

#define WIRE_BUFSIZ 32

/* return codes from endTransmission() */
#define SUCCESS   0        /* transmission was successful */
#define EDATA     1        /* too much data */
#define ENACKADDR 2        /* received nack on transmit of address */
#define ENACKTRNS 3        /* received nack on transmit of data */
#define EOTHER    4        /* other error */

typedef struct i2c_msg {
	uint16_t addr;                /**< Address */
#define I2C_MSG_READ            0x1
#define I2C_MSG_10BIT_ADDR      0x2
	uint16_t flags;

	uint16_t length;              /**< Message length */
	uint16_t xferred;             /**< Messages transferred */
	uint8_t *data;                /**< Data */
} i2c_msg;


class WireBase { // Abstraction is awesome!
protected:
    i2c_msg itc_msg;
    uint8_t rx_buf[WIRE_BUFSIZ];      /* receive buffer */
    uint8_t rx_buf_idx;               /* first unread idx in rx_buf */
    uint8_t rx_buf_len;               /* number of bytes read */

    uint8_t tx_buf[WIRE_BUFSIZ];      /* transmit buffer */
    uint8_t tx_buf_idx;  // next idx available in tx_buf, -1 overflow
    bool tx_buf_overflow;

    // Force derived classes to define process function
    virtual uint8_t process() = 0;
public:
    WireBase() {}
    ~WireBase() {}

    /*
     * Initialises the class interface
     */
    // Allow derived classes to overwrite begin function
    virtual void begin(uint8_t = 0x00);

    /*
     * Sets up the transmission message to be processed
     */
    void beginTransmission(uint8_t);

    /*
     * Allow only 8 bit addresses to be used
     */
    void beginTransmission(int);

    /*
     * Call the process function to process the message if the TX
     * buffer has not overflowed.
     */
    uint8_t endTransmission(void);

    /*
     * Request bytes from a slave device and process the request,
     * storing into the receiving buffer.
     */
    uint8_t requestFrom(uint8_t, int);

    /*
     * Allow only 8 bit addresses to be used when requesting bytes
     */
    uint8_t requestFrom(int, int);

    /*
     * Stack up bytes to be sent when transmitting
     */
    void write(uint8_t);

    /*
     * Stack up bytes from the array to be sent when transmitting
     */
    void write(uint8_t*, int);

    /*
     * Ensure that a sending data will only be 8-bit bytes
     */
    void write(int);

    /*
     * Ensure that an array sending data will only be 8-bit bytes
     */
    void write(int*, int);

    /*
     * Stack up bytes from a string to be sent when transmitting
     */
    void write(char*);

    /*
     * Return the amount of bytes that is currently in the receiving buffer
     */
    uint8_t available();

    /*
     * Return the value of byte in the receiving buffer that is currently being
     * pointed to
     */
    uint8_t read();
};

#endif // _WIREBASE_H_
#endif
