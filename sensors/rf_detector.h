/**
 * @file rf_detector.h
 *
 * This is a sensor class to handle reading from the RF detector.
 *
 * @author Adrien Perkins <adrienp@stanford.edu>
 */

#ifndef RF_DETECTOR_H_
#define RF_DETECTOR_H_


#define LTS_SYNC_A 0xA0
#define LTS_SYNC_B 0xB1

#define LTS_PAYLOAD_LEN 8	// since there is only 1 message type


/** the structures of the binary packets */
#pragma pack(push, 1)

typedef struct {
  uint32_t timestamp;
  int16_t dir;			/* directional antenna signal strength */
  int16_t omni;			/* omni directional atenna signal strength */
} strength_measurement_t;

/* messages and payload buffer */
typedef union {
	strength_measurement_t signal_strength;
	uint8_t raw[];
} msg_buf_t;


#pragma pack(pop)



/* decoding state */
typedef enum {
	LTS_DECODE_SYNC_A = 0,
	LTS_DECODE_SYNC_B,
	LTS_DECODE_PAYLOAD,
	LTS_DECODE_CHK_A,
	LTS_DECODE_CHK_B
} lts_decode_state_t;


class RFDetector {
public:
	
	/**
	 * constructor
	 */
	RFDetector(int fd);

	/**
	 * destructor
	 */
	~RFDetector();

	/**
	 * read the next measurement.
	 * @param  message the measurement struct to write the data to
	 * @return         -1 = error, 0 = no measurement, 1 = new measurement
	 */
	int read_measurement(strength_measurement_t *message);

	/**
	 * Parse the incoming bytes from the arduino.
	 * @param  c the incoming byte
	 * @return   0 = decoding, 1 = message completed
	 */
	int parse_char(const uint8_t c);

private:

	int _fd;
	
	/* states */
	lts_decode_state_t _decode_state;

	/* indices */
	int _payload_index;

	/* buffers */
	msg_buf_t _buf;

	/* checksum values */
	uint8_t _chk_a;
	uint8_t _chk_b;

	/**
	 * Reset the parse state machine for a fresh start.
	 */
	void decode_init();

	/**
	 * Add byte to the payload.
	 * @param  c the new byte
	 * @return   -1 = error, 0 = continue, 1 = completed
	 */
	int payload_rx_add(const uint8_t c);

	/**
	 * Finish payload rx.
	 * @return   0 = no message handled, 1 = completed
	 */
	int payload_rx_done(void);

	/**
	 * Calculate checksum.
	 * @param  c  the new byte
	 */
	void add_byte_to_checksum(const uint8_t c);


	

};


#endif /* RF_DETECTOR_H_ */