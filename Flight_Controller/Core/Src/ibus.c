/*
 * ibus.c
 *
 *  Created on: May 17, 2024
 *      Author: David Exinor
 */

#include "ibus.h"
#include "usbd_cdc_if.h"

/* Static Functions */

/*
 *  Scales value to specified range
 *  Note: Channel outputs on transmitter have been adjusted to reflect the passed values to the function from normalized_data
 */
static double normalize(double value, double old_min, double old_max, double new_min, double new_max) {
    return ((double)(value - old_min) * (new_max - new_min) / (double)(old_max - old_min)) + new_min;
}

static void normalize_data(uint16_t* channels, ibus_rx_t* ibus_rx_struct_ptr) {
	// state is only set to -1 in process_iBus_data for now, but might make an alg to verify valeus
	float sum = 0;
	/* normalize joystick values to be within -1.0 and 1.0 for pitch/roll/yaw and 0.0 to 1.0 for throttle */
	ibus_rx_struct_ptr->ch1 = normalize(channels[0], 4000, 56000, -100, 100), sum += ibus_rx_struct_ptr->ch1;
	ibus_rx_struct_ptr->ch2 = normalize(channels[1], 4000, 56000, -100, 100), sum += ibus_rx_struct_ptr->ch2;
	ibus_rx_struct_ptr->ch3 = normalize(channels[2], 4800, 56000, 0, 2000), sum += ibus_rx_struct_ptr->ch3;
	ibus_rx_struct_ptr->ch4 = normalize(channels[3], 4000, 56000, -100, 100), sum += ibus_rx_struct_ptr->ch4;

	/* normalize switch values to be between 0.0 and 1.0 */
	ibus_rx_struct_ptr->ch5 = normalize(channels[4], 55000, 58000, 0.0, 1.0), sum += ibus_rx_struct_ptr->ch5;
	ibus_rx_struct_ptr->ch6 = normalize(channels[5], 55000, 58000, 0.0, 1.0), sum += ibus_rx_struct_ptr->ch6;
	ibus_rx_struct_ptr->ch7 = normalize(channels[6], 55000, 58000, 0.0, 1.0), sum += ibus_rx_struct_ptr->ch7;
	ibus_rx_struct_ptr->ch8 = normalize(channels[7], 55000, 58000, 0.0, 1.0), sum += ibus_rx_struct_ptr->ch8;

	// Update Ibus Rx state
	(ibus_rx_struct_ptr->chsum != sum) ? (ibus_rx_struct_ptr->state = 1), (ibus_rx_struct_ptr->chsum = sum) : (ibus_rx_struct_ptr->state = 0);
}

#ifdef IBUS_DEBUG
static void debug_ibus_channels(ibus_rx_t* ibus_rx_struct_ptr) {
	// Print channel values and state for debugging
	char message_buffer[500];

	sprintf(message_buffer, "Ch1: %.02f\r\nCh2: %.02f\r\nCh3: %.02f\r\nCh4: %.02f\r\nCh5: %.02f\r\nState: %d\r\n",
			ibus_rx_struct_ptr->ch1, ibus_rx_struct_ptr->ch2, ibus_rx_struct_ptr->ch3, ibus_rx_struct_ptr->ch4,
			ibus_rx_struct_ptr->ch5,ibus_rx_struct_ptr->state);

	CDC_Transmit_FS((uint8_t*)message_buffer, strlen(message_buffer));
}

static void debug_ibus_packets(uint8_t* data) {
	int offset = 0;
	char entire_message[2000];
	// Clear the message buffer
	memset(entire_message, 0, sizeof(entire_message));

	char byte0[20];
	sprintf(byte0, "Byte 0: %d  ", data[0]);
	strcat(entire_message + offset, byte0);
	offset += strlen(byte0);

	char byte1[20];
	sprintf(byte1, "Byte 1: %d  \r\n", data[1]);
	strcat(entire_message + offset, byte1);
	offset += strlen(byte1);

	for (int i = 0 ; i < 15 ; i++)  {
		char temp[100];
		sprintf(temp, "Packet %d: %d  \r\n", i, (uint16_t)(((uint16_t)data[2 * i + 3] << 8) | data[2 * i + 2]));
		strcat(entire_message + offset, temp);
		offset += strlen(temp);
	}
	CDC_Transmit_FS((uint8_t*)entire_message, strlen(entire_message));
}
#endif /* IBUS_DBUG */

/* Functions */

void process_iBus_data(uint8_t *data, ibus_rx_t* ibus_rx_struct_ptr) {
    uint16_t channels[14];
    uint16_t checksum = 0;
    uint16_t received_checksum;

#ifdef CHECKSUM // checksum not working correctly
    // Calculate checksum
    for (int i = 0; i < 30; i++) {
    	checksum += data[i];
    }

    // Extract received checksum
    received_checksum = (uint16_t)(((uint16_t)data[30] << 8) | data[31]);

    if (checksum == received_checksum) {
#endif /* CHECKSUM */
    	// decode buffer packets
        for (int i = 0; i < 14; i++) {
        	channels[i] = (uint16_t)(((uint16_t)data[2 * i + 3] << 8) | data[2 * i + 2]);
        }
        // format and store channel data
        normalize_data(channels, ibus_rx_struct_ptr);
#ifdef IBUS_DEBUG
        debug_ibus_channels(ibus_rx_struct_ptr);
#endif /* IBUS_DEBUG */

#ifdef CHECKSUM
    } else {
        // Handle checksum error
    	debug_ibus_packets(data);
    }
#endif
}
