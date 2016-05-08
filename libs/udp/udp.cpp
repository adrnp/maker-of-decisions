/**
 * udp.cpp
 *
 * implementation code of the UDP handling class.
 *
 * @author Adrien Perkins <adrienp@stanford.edu>
 */
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <netdb.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "udp.h"


UDP::UDP() :
_socket_fd(-1)
{
	// default server address
	_server = (char *) "127.0.0.1";

	// initialize the structs
	memset((char *) &_myaddr, 0, sizeof(_myaddr));
	memset((char *) &_remaddr, 0, sizeof(_remaddr));

	// setup the socket and such
	if (setup_server() < 0) {
		throw EXIT_FAILURE;
	}
}


UDP::~UDP() {
	// close the socket
	if (_socket_fd > 0) {
		close(_socket_fd);
	}
}


/* return values: -1 error */
int UDP::send_broadcast(const char* buf, const size_t len) {

	/* simply send the message passed along */
	return sendto(_socket_fd, buf, len, 0, (struct sockaddr *) &_remaddr, sizeof(_remaddr));
}


int UDP::send_bearing_message(const double &bearing_cc, const double &bearing_max, const double &bearing_max3,
								const int32_t &lat, const int32_t &lon, const float &alt) {

	// build the message
	// TODO: add structure (limit number of characters displayed for each of the values(?))!
	char buf[1024];
	sprintf(buf, "BEAR:%f,%f,%f,%d,%d,%f", bearing_cc, bearing_max, bearing_max3, lat, lon, alt);

	// actually send the message
	return send_broadcast(buf, strlen(buf));
}


int UDP::send_rssi_message(const int &in_rot, const int &dir_rssi, const int &omni_rssi, const int16_t &heading,
							const int32_t &lat, const int32_t &lon, const float &alt) {

	// build the message
	// TODO: add structure (limit number of characters displayed for each of the values(?))!
	char buf[1024];
	sprintf(buf, "RSSI:%d,%d,%d,%d,%d,%d,%f\n", in_rot, dir_rssi, omni_rssi, heading, lat, lon, alt);

	// actually send the message
	return send_broadcast(buf, strlen(buf));
}


/* return values: -1 error, 0 success */
int UDP::setup_server() {

	/* create a socket */
	if ((_socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
		printf("socket failed to create\n");
		return -1;
		// this is probable a failure condition...
	}

	/* bind it to all local addresses and pick any port number */
	_myaddr.sin_family = AF_INET;
	_myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	_myaddr.sin_port = htons(0);

	if (bind(_socket_fd, (struct sockaddr *) &_myaddr, sizeof(_myaddr)) < 0) {
		perror("bind failed\n");
		return -1;
	}       

	/* now define remaddr, the address to whom we want to send messages */
	/* For convenience, the host address is expressed as a numeric IP address */
	/* that we will convert to a binary format via inet_aton */
	_remaddr.sin_family = AF_INET;
	_remaddr.sin_port = htons(SERVICE_PORT);
	if (inet_aton(_server, &_remaddr.sin_addr) == 0) {
		fprintf(stderr, "inet_aton() failed\n");
		return -1;
	}

	// if got to this point, all is good
	return 0;
}