/**
 * udp.h
 *
 * A class to handle UDP broadcasts.
 *
 * @author Adrien Perkins <adrienp@stanford.edu>
 */
#ifndef UDP_H_
#define UDP_H_


#include <netdb.h>
#include <sys/socket.h>


#define SERVICE_PORT	21234	/* hard coded port to send data to */

// list of the possible bearing types
enum bearing_type_t {
	TYPE_BEARING_CC = 1,
	TYPE_BEARING_MLE
};

class UDP {


public:

	/* constructor */
	UDP();
	UDP(const char *server);

	/* destructor */
	~UDP();

	/* send a broadcast message */
	int send_broadcast(const char* buf, const size_t len);

	/* specific message sets to send */
	int send_bearing_message(const double &bearing_cc, const double &bearing_max, const double &bearing_max3,
								const int32_t &lat, const int32_t &lon, const float &alt);
	int send_rssi_message(const int &in_rot, const int &dir_rssi, const int &omni_rssi, const int16_t &heading,
								const int32_t &lat, const int32_t &lon, const float &alt);

	int send_mean_message(const double &mu_x, const double &mu_y);
	int send_cov_message(const double &a, const double &b, const double &c, const double &d);

private:

	/* the address of the server (?) */
	const char* _server;

	/* the socket file descriptor */
	int _socket_fd;

	/* addresses */
	struct sockaddr_in _myaddr;		// my address (local host (?))
	struct sockaddr_in _remaddr;	// remote address (broadcast remote)

	int setup_server();


};

#endif /* UDP_H_ */
