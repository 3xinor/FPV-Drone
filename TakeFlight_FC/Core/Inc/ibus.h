/*
 * ibus.h
 *
 *  Created on: May 17, 2024
 *      Author: David Exinor
 */

#ifndef INC_IBUS_H_
#define INC_IBUS_H_

//#define CHECKSUM

#define IBUS_BUFFER_SIZE			32

#include <stdint.h>

typedef struct {
	uint8_t state; // state is either 1 for new values, 0, for same values or -1 for invalid values
	double ch1; // roll
	double ch2; // pitch
	double ch3; // throttle
	double ch4; // yaw
	double ch5; // SWA
	double ch6; // SWB
	double ch7; // SWC
	double ch8; // SWD
	double ch9; // VARA
	double ch10; //VARB
	float chsum;
	/* remaining 4 channels are present but not available for use on FS-IA6X 10AB */
} ibus_rx_t;

/*
 * Process iBus raw data which uses little endian 2 byte encodings for each of the 14 channels
 */
void process_iBus_data(uint8_t *data, ibus_rx_t* ibus_rx_struct_ptr);

#endif /* INC_IBUS_H_ */
