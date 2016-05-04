/**
 * @file rf_detector.cpp
 *
 * Definition the RF detector sensor class.
 * 
 * @author Adrien Perkins <adrienp@stanford.edu>
 */

#include <cstdlib>	/* standard library (std namespace) */
#include <unistd.h>	/* unix standard library (e.g. read call) */
#include <iostream>	/* output and printing library */

#include <cstdint>	/* for more datatypes */
#include <string.h>	/* use of strings and string related functions */
#include <math.h>	/* for pow and other math functions */

#include <iomanip>	/* std::setfill and setw functions */

#include <bitset>	/* std::bitset<#> for bit representing an int */

#include "rf_detector.h"


RFDetector::RFDetector(int fd) :
	_fd(fd),
	_decode_state(LTS_DECODE_SYNC_A),
	_payload_index(0),
	_chk_a(0),
	_chk_b(0)
{
	decode_init();
}

RFDetector::~RFDetector() { }


int RFDetector::read_measurement(strength_measurement_t *measurement) {

	// TODO: definitely want to add a timeout here, this is fairly dangerous...
	int msg_rcvd = 0;
	uint8_t c;
	while (msg_rcvd <= 0) {

		// read in a single char at a time (if one is present)
		if (read(_fd, &c, 1) > 0) {

			msg_rcvd = parse_char(c);

		} else {
			std::cout << "[RF DETECTOR] read error!\n";
			return -1;
		}
	}

	// copy the buffy over to the incoming strength message
	measurement->timestamp = _buf.signal_strength.timestamp;
	measurement->dir = _buf.signal_strength.dir;
	measurement->omni = _buf.signal_strength.omni;

	return 1;
}



// initialize all the globals for a successful message decoding
void RFDetector::decode_init() {
	// DEBUG
	//std::cout << "initializing decode\n";

	/* set state to wait for sync */
	_decode_state = LTS_DECODE_SYNC_A;

	/* reinit the payload starting at the header */
	_payload_index = 0;

	/* initialize checksum elements */
	_chk_a = 0;
	_chk_b = 0;
}


int RFDetector::parse_char(const uint8_t c) {
	// DEBUG - show incoming byte
	//std::cout << std::hex << std::setfill('0') << std::setw(2) << (int) c << " ";
	
	int ret = 0;  // default to still decoding

	switch(_decode_state) {
	
	/* Expecting Sync Message A */
	case LTS_DECODE_SYNC_A:

		if (c == LTS_SYNC_A) {
			_decode_state = LTS_DECODE_SYNC_B;
		}

		break;

	/* Expecting Sync Message B */
	case LTS_DECODE_SYNC_B:

		if (c == LTS_SYNC_B) {
			_decode_state = LTS_DECODE_PAYLOAD;
		} else {
			_decode_state = LTS_DECODE_SYNC_A;
			decode_init();
		}

		break;

	/* Expecting Payload */
	case LTS_DECODE_PAYLOAD:

		ret = payload_rx_add(c);
		add_byte_to_checksum(c);

		// check if completed the payload
		if (ret > 0) {
			_decode_state = LTS_DECODE_CHK_B;
		} 

		ret = 0;
		break;

	/* Expecting 1st Checksum Byte */
	case LTS_DECODE_CHK_B:

		if (_chk_b == c) {
			_decode_state = LTS_DECODE_CHK_A;
		} else {
			_decode_state = LTS_DECODE_SYNC_A;
			decode_init();
		}

		break;

	/* Expecting 2nd Checksum Byte */
	case LTS_DECODE_CHK_A:

		if (_chk_a == c) {
			payload_rx_done();
		}
		_decode_state = LTS_DECODE_SYNC_A;
		decode_init();

		break;
	}

	return ret;
}


int RFDetector::payload_rx_add(const uint8_t c) {
	int ret = 0;

	_buf.raw[_payload_index] = c;

	if (++_payload_index >= LTS_PAYLOAD_LEN) {
		ret = 1;  // paylaod completed
	}

	return ret;
}



int RFDetector::payload_rx_done(void) {
	int ret = 0;

	// for all messages, the data has already been peeled out...

	// TODO: just need to add timestamp and return 1

	// DEBUG - for now just a bunch of printing
	std::cout << "\nThe information is\n";
	std::cout << "\ttimestamp: " << std::dec << _buf.signal_strength.timestamp << "\n";
	std::cout << "\tdirectional: " << _buf.signal_strength.dir << "\n";
	std::cout << "\tomnidirectional: " << _buf.signal_strength.omni << "\n";

	ret = 1;
	return ret;
}


void RFDetector::add_byte_to_checksum(const uint8_t c) {
	_chk_a += c;
	_chk_b += _chk_a;
}