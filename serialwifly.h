
#ifndef SERIALWIFLY_H_
#define SERIALWIFLY_H_

/* start the connection */
int start_connection(char *port);

/* Ensure that you are in command mode */
int commandmode(int);

/* Parses output from scan  */
int getrssi(char *, char *);

/* Scan the channels for a specific SSID */
int scanrssi(int, char *);
int scanrssi_f(int fd, char *ssid, FILE *f, int numtimes);
int scanrssi_2(int fd1, int fd2, char *ssid, FILE *f1, FILE *f2, int numtimes);




#endif /* SERIALWIFLY_H_ */