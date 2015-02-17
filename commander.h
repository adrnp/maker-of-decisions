/* header file for commander.cpp.
 * commander contains all the functions required for 
 * generating and sending commands as needed.
 */


#ifndef COMMANDER_H_
#define COMMANDER_H_



void sendNextCommand();

int write_to_serial(mavlink_message_t &message);





#endif /* COMMANDER_H_ */