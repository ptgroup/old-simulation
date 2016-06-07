#include "serial.h"

#include <stdlib.h>

#include "rs232.h"

void serial_start(int port) {
    // Start serial stuff
    puts("Closing port...");
    RS232_CloseComport(port);
    //delay(1);
    puts("Opening port...");
    if (RS232_OpenComport(port, 9600, "8N1")) {
        puts("Could not open port");
        puts("Press enter to exit...");
        getchar();
        exit(1);
    }
}

uint8_t serial_rx_byte(int port) {
    uint8_t ret;
    int got = RS232_PollComport(port, &ret, 1);

    return got ? ret : 0;
}

uint8_t serial_rx_byte_wait(int port) {
    uint8_t ret;
    int got;

    do {
        got = RS232_PollComport(port, &ret, 1);
    } while (!got);

    return ret;
}

void serial_tx_byte(int port, uint8_t value) {
    RS232_SendByte(port, value);
}

// Reads a floating point number (4-uint8_t IEEE 754) from the Propeller (MSB first)
float serial_rx_float(int port) {
    uint8_t b1 = serial_rx_byte_wait(port);
    uint8_t b2 = serial_rx_byte_wait(port);
    uint8_t b3 = serial_rx_byte_wait(port);
    uint8_t b4 = serial_rx_byte_wait(port);

    uint8_t uint8_ts[4] = {b4, b3, b2, b1};
    return *(float*)uint8_ts;
}

// Data comes MSB first
int32_t serial_rx_int32(int port) { 
    uint8_t b1 = serial_rx_byte_wait(port);
    uint8_t b2 = serial_rx_byte_wait(port);
    uint8_t b3 = serial_rx_byte_wait(port);
    uint8_t b4 = serial_rx_byte_wait(port);

    uint8_t uint8_ts[4] = {b4, b3, b2, b1};
    return *(int32_t*)uint8_ts;
}

// Writes MSB first
void serial_tx_float(int port, float value) {
    uint8_t *valueBytes = (uint8_t*)(&value);

    for (int i = 3; i >= 0; i--) {
        serial_tx_byte(port, valueBytes[i]);
    }
}

// MSB first
void serial_tx_int32(int port, int32_t value) {
    uint8_t *valueBytes = (uint8_t*)(&value);

    for (int i = 3; i >= 0; i--) {
        serial_tx_byte(port, valueBytes[i]);
    }
}
