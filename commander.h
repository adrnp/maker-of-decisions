/* header file for commander.cpp.
 * commander contains all the functions required for 
 * generating and sending commands as needed.
 */


#ifndef COMMANDER_H_
#define COMMANDER_H_


/* send the next command for testing movement */
void sendNextCommand();

/* send a command to rotate the vehicle
 * direction: -1 is CCW and 1 is CW
 */
void sendRotateCommand(float direction);

/* send a command to go into off mode */
void send_finish_command();

/* send the bearing message */
// TODO: instead of commander, need a writer thread....
void send_bearing_message(double &bearing, int32_t &lat, int32_t &lon, float &alt);

/* send the rssi message */
void send_rssi_message(int &rssi, int16_t &heading, int32_t &lat, int32_t &lon, float &alt);

int write_to_serial(mavlink_message_t &message);





#endif /* COMMANDER_H_ */