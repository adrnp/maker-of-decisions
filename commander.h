/* header file for commander.cpp.
 * commander contains all the functions required for 
 * generating and sending commands as needed.
 */


#ifndef COMMANDER_H_
#define COMMANDER_H_


/* send the next command for testing movement */
void sendNextCommand();

/* send a command to rotate the vehicle */
void sendRotateCommand();

/* send the bearing message */
// TODO: instead of commander, need a writer thread....
void send_bearing_message(double &bearing, int32_t &lat, int32_t &lon, float &alt);

int write_to_serial(mavlink_message_t &message);





#endif /* COMMANDER_H_ */