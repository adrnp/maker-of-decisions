/* header file for commander.cpp.
 * commander contains all the functions required for 
 * generating and sending commands as needed.
 */


#ifndef COMMANDER_H_
#define COMMANDER_H_

#include <vector>


/* load in a set of move commands from a file */
bool load_move_commands();

/* handles the overall decision of which command to actually send */
void send_next_command(uint8_t &prev_state, uint8_t &new_state, double &bearing, int &rssi);


/* send the next command for testing movement */
void send_move_command();

/* send df mode command to the df arduino */
void send_df_mode(int mode);





#endif /* COMMANDER_H_ */
