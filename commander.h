/**
 * @file commander.h
 *
 * Set of functions to handle generating and sending commands as needed.
 * 
 * @Author Adrien Perkins <adrienp@stanford.edu>
 */

#ifndef COMMANDER_H_
#define COMMANDER_H_


/**
 * handles the overall decision of which command to actually send
 * @param prev_state  the previous hunt state
 * @param new_state   the current (and new) hunt state
 */
void send_next_command(uint8_t &prev_state, uint8_t &new_state);

/**
 * send df mode command to the df arduino
 * @param mode  the mode of operation for the DF antenna (sum or difference)
 */
void send_df_mode(int mode);



#endif /* COMMANDER_H_ */
