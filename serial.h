// serial.h --- Provides basic serial communication for the simulation
#ifndef _SERIAL_H
#define _SERIAL_H

#include <stdint.h>

void serial_start(int port); // starts serial communication
uint8_t serial_rx_byte(int port); // gets the next byte without waiting (0x00 == "none")
uint8_t serial_rx_byte_wait(int port); // gets the next byte, waiting until it is received
void serial_tx_byte(int port, uint8_t value); // sends a byte
float serial_rx_float(int port); // reads a float into "value" from the Propeller
int32_t serial_rx_int32(int port); // reads a 32-bit integer from the Propeller
void serial_tx_float(int port, float value); // writes a float to the Propeller
void serial_tx_int32(int port, int32_t value); // writes a 32-bit integer to the Propeller

#endif
