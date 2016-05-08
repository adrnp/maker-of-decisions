/** this file contains the important system ids used in Mavlink to identify owner and intended receiver of messages */

#ifndef SYSTEM_IDS_H_
#define SYSTEM_IDS_H_

// UNIQUE SYSTEM IDS, NEED TO BE DIFFERENT THAN ALL OTHER IDS TALKING TO THE PIXHAWK
int sysid = 16;             ///< The unique system id of this MAV, 0-127. Has to be consistent across the system
int compid = 101;			// the autopilot's component ID
int serial_compid = 109;	// the computer component ID that indentifies this application

#endif /* SYSTEM_IDS_H_ */
