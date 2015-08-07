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

/* send the next tracking command */
void sendTrackingCommand(float &north, float &east);

/* send the next command for testing movement */
void sendMoveCommand();

/* send a command to rotate the vehicle
 * direction: -1 is CCW and 1 is CW
 */
void sendRotateCommand(float direction);

/* send a command to go into off mode */
void send_finish_command();

/* send the bearing message */
// TODO: instead of commander, need a writer thread....
void send_bearing_cc_message(double &bearing, int32_t &lat, int32_t &lon, float &alt);

/* send the mle bearing message */
void send_bearing_mle_message(double &bearing, int32_t &lat, int32_t &lon, float &alt);

/* send the rssi message */
void send_rssi_message(int &rssi, int &rssi2, int16_t &heading, int32_t &lat, int32_t &lon, float &alt);

/* calculate the next tracking command */
std::vector<float> calc_next_command(double &bearing, int &rssi);

/* send df mode command to the df arduino */
void send_df_mode(int mode);





#endif /* COMMANDER_H_ */
