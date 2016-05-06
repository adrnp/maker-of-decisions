/**
 * @file mod.cpp
 *
 * Contains the core running functions.
 * Reads a config file to populate all the necessary parameters and settings.
 * Based on configuration, it starts the necessary thread.
 *
 * @author Adrien Perkins <adrienp@stanford.edu>
 */


// system inclusions
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <map>
#include <signal.h>
#include <time.h>	// to determine the time of day and day of week
#include <sys/stat.h>	// to create the logfile directory as needed

#include "common.h"

#include "read_thread.h"
#include "wifly_thread.h"
#include "rfhunter_thread.h"
#include "dirk_thread.h"

#include "libs/planners/fixed_planner.h"
#include "libs/planners/circle_planner.h"
#include "libs/planners/naive_planner.h"

#include "mod.h"

using std::string;
using namespace std;


// common variable declarations
namespace common {
	bool verbose = false;	// default verbose to false
	bool debug = false;		// default debug to false

	MAVInfo uav;			// object to hold all the state information on the UAV
	int RUNNING_FLAG = 1;	// default the read and write threads to be running

	MavlinkSerial* pixhawk = nullptr;	// serial connection to the pixhawk
	SerialPort* df_arduino = nullptr;	// serial connection to the df arduino

	const char* sensor_port = (char*)"/dev/ttyUSB0";
	const char* omni_wifly_port = (char*)"/dev/ttyUSB2";

	bool dual_wifly = false;	// default to only have one wifly active

	float flight_alt = 380;			// default flight is AMSL
	int tracker_type = TRACK_NAIVE;	// the type of tracker to use

	bool emily = false;			// default to not running the emily antenna configuration
	
	string logfile_dir;		// the logfile directory that will be used

	Planner *planner = nullptr;
	FILE *output_logfile = nullptr;
}


// "local" globals
bool phased_array = false;	// default to not using a phased array antenna
char* pa_port;

bool nowifly = false;	// default to wanting wifly

int mission_type = 0;
int sensor_type = 0;

const char *pixhawk_port = (char*)"/dev/ttyUSB1";
int baudrate = 115200;


const char *command_file = (char*) "commands/commands.csv";
string planner_config_file;


int get_configuration(int argc, char **argv) {

	char *config_filename = (char*)"config.cfg";

	// string to be displayed on incorrect inputs to show correct function usage
	const char *commandline_usage = "\tusage: -c/--config <config_filename>\n\n"
									"\tthe config filename is optional and will default to config.cfg\n"
									"\t(see config.cfg to see what is required in the config file)\n\n"
									"\texample: ./mod -d config.cfg\n";

	// loop through all the program arguments
	for (int i = 1; i < argc; i++) { /* argv[0] is "mavlink" */

		// help text requested
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			cout << commandline_usage;
			return -1;
		}

		/* UART device ID */
		if (strcmp(argv[i], "-c") == 0 || strcmp(argv[i], "--config") == 0) {
			if (argc > i + 1) {
				config_filename = argv[i + 1];
			}
		}
	}

	// the map containing all the params
	map<string, string> config_map;

	// the logfile that will get all the config params
	// doing this so we know what configs were used for each flight
	string config_logfile_path = common::logfile_dir + "config.cfg";
	ofstream config_logfile(config_logfile_path);
	bool log_open = config_logfile.is_open();

	/* parse through the config file */
	ifstream infile(config_filename);
	
	string line, key, value;  // some initialization
	while (getline(infile, line)) { // loop while we can get a new line from the file

		// skip comment lines
		size_t found = line.find_first_not_of(" \t");
		if (found != string::npos) {
			if (line[found] == '#') continue;
		}

		// get an input string stream of the current line
		istringstream curr_line(line);
		if (getline(curr_line, key, '=')) {  // read up to the = sign and set that to key
			if (getline(curr_line, value)) {  // read rest set that to value
				config_map[key] = value;
				cout << "key: " << key << "\tvalue: " << value << "\n";
				
				// log the configuration
				if (log_open) {
					config_logfile << key << "=" << value << "\n";
				}
			}
		}
	}

	// close the configuration log
	if (log_open) {
		config_logfile.close();
	}

	/* general */
	if (config_map.find("verbose") != config_map.end()) {
		common::verbose = (stoi(config_map["verbose"]) == 1);
	}

	if (config_map.find("planner_type") != config_map.end()) {
		common::tracker_type = stoi(config_map["planner_type"]);
	}

	if (config_map.find("sensor_type") != config_map.end()) {
		sensor_type = stoi(config_map["sensor_type"]);
	}

	if (config_map.find("flight_alt") != config_map.end()) {
		common::flight_alt = stof(config_map["flight_alt"]);
	}

	/* command file */
	if (config_map.find("command_file") != config_map.end()) {
		command_file = config_map["command_file"].c_str();
	}

	/* planner config gile file */
	if (config_map.find("planner_config_file") != config_map.end()) {
		planner_config_file = config_map["planner_config_file"];
	}

	/* pixhawk */
	if (config_map.find("pixhawk_port") != config_map.end()) {
		pixhawk_port = config_map["pixhawk_port"].c_str();
	} else {
		cout << "Pixhawk port is a required config.\nPlease check your config file (" << config_filename << ")\n";
		return -1;
	}

	if (config_map.find("pixhawk_baudrate") != config_map.end()) {
		baudrate = stoi(config_map["pixhawk_baudrate"]);
	} else {
		cout << "Pixhawk baudrate is a required config.\nPlease check your config file (" << config_filename << ")\n";
		return -1;
	}

	/* wifly */
	if (config_map.find("sensor_port") != config_map.end()) {
		common::sensor_port = config_map["sensor_port"].c_str();
	} else {
		cout << "Sensor port is a required config.\nPlease check your config file (" << config_filename << ")\n";
		return -1;
	}

	if (config_map.find("dual_wifly") != config_map.end()) {
		common::dual_wifly = (stoi(config_map["dual_wifly"]) == 1);
	}

	if (common::dual_wifly) {
		if (config_map.find("omni_wifly_port") != config_map.end()) {
			common::omni_wifly_port = config_map["omni_wifly_port"].c_str();
		} else {
			cout << "Running 2 wiflys but failed to provide port for second wifly.\nPlease check your config file (" << config_filename << ")\n";
			return -1;
		}
		
	}

	/* emily */
	if (config_map.find("emily_antenna") != config_map.end()) {
		common::emily = (stoi(config_map["emily_antenna"]) == 1);
	}

	return 1;
}



void quit_handler(int sig) {

	// notify user of termination
	printf("Terminating script\n");

	// set the running flag to 0 to kill all loops
	common::RUNNING_FLAG = 0;

	common::pixhawk->end_serial();

}

// 0 = success, -1 = error
int create_directory(const char *path, mode_t mode) {
	struct stat st;
	int status = 0;
	
	if (stat(path, &st) != 0) {
		if (mkdir(path, mode) != 0) {
			status = -1;
		}
	} else {
		cout << "directory already exists\n";
	}

	// return the status
	return status;
}

string to_zeropad_string(const int &val, const int &len) {
	string input = to_string(val);
	return string(len - input.length(), '0') + input;
}


int setup_logfiles() {

	// the return variable
	int status = 0;
	
	// get the current day of the week for the logging file
	//const string day[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
	
	time_t rawtime;
	struct tm *timeinfo;
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	//int wday = timeinfo->tm_wday;
	
	// get the current time while we are at it for actually naming the files....
	int hour = timeinfo->tm_hour;
	int min = timeinfo->tm_min;

	// check to see if the logs directory exists, and if it doesn't create it
	string log_path = "logs/";
	status = create_directory(log_path.c_str(), 0777);
	if (status < 0) {
		return -1;
	}
	
	// now check to make sure if this specific day of the week directory has been created
	//string day_path = log_path + day[wday] + "/";
	string day_path = log_path + to_zeropad_string(timeinfo->tm_mon + 1, 2) + "_" + to_zeropad_string(timeinfo->tm_mday, 2) + "/";
	status = create_directory(day_path.c_str(), 0777);
	if (status < 0) {
		return -1;
	}	

	// finally create yet another directory that is timestamped that will contain all the log files
	string time_path = day_path + to_zeropad_string(hour, 2) + "_" + to_zeropad_string(min, 2) + "/";
	status = create_directory(time_path.c_str(), 0777);
	if (status < 0) {
		return -1;
	}

	// at this point means that all the directories are now present so the logfile dir can be populared
	common::logfile_dir = time_path;

	return 0;
}


int main(int argc, char **argv) {

	cout << "[MOD] starting...\n";

	// setup termination using CRT-C
	signal(SIGINT, quit_handler);

	// ---------------------------------- //
	// get and set all the configurations
	// ---------------------------------- //
	
	// setup all the log file stuff for this run
	setup_logfiles();
	
	// open the main output logfile
	string output_logfile_name = common::logfile_dir + "log.out";
	common::output_logfile = fopen(output_logfile_name.c_str(), "a");
	if (common::output_logfile == NULL) {
		cout << "[MOD] error opening main logfile\n";
		return 0;
	}

	//cout << "[MOD] reading arguments\n";
	LOG_STATUS("[MOD] reading arguments");
	
	
	// get all the confguration parameters
	if (get_configuration(argc, argv) < 0) {
		fclose(common::output_logfile);
		return 0;
	}

	// ---------------------------------- //
	// initialize things
	// ---------------------------------- //

	// initialize the planner
	common::planner = new FixedPlanner(command_file);	// default to running command files, this ensures this isn't null
	switch (common::tracker_type) {
		/* the naive tracker */
		case TRACK_NAIVE:
			LOG_STATUS("[MOD] running naive planner");
			common::planner = new NaivePlanner();
			break;

		/* variable step naive tracker */
		case TRACK_VARIABLE:
			LOG_STATUS("[MOD] running naive planner variable");
			common::planner = new NaivePlanner();
			break;

		/* the pomdp tracker */
		case TRACK_POMDP:
			LOG_STATUS("[MOD] running pomdp planner");
			break;

		/* the circle planner */
		case TRACK_CIRCLE:
			LOG_STATUS("[MOD] running circle planner");
			common::planner = new CirclePlanner(planner_config_file);
	}
	LOG_STATUS("[MOD] initializing planner...");
	common::planner->initialize();

	// connect to the pixhawk
	LOG_STATUS("[MOD] connecting to pixhawk...");
	common::pixhawk = new MavlinkSerial(common::logfile_dir, common::verbose, pixhawk_port, baudrate);
	LOG_STATUS("[MOD] pixhawk fd is %d", common::pixhawk->get_fd());
	
	// ---------------------------------- //
	// create threads
	// ---------------------------------- //
	
	// ids of the threads
	pthread_t readId;
	pthread_t huntingId;

	// read thread
	LOG_STATUS("[MOD] handling threads");
	pthread_create(&readId, NULL, read_thread, (void *)&common::uav);


	// create a thread for the wifly stuff (only if want wifly running)
	bool hunt_running = false;
	switch (sensor_type) {
	case 1:
		LOG_STATUS("[MOD] starting wifly thread...");
		pthread_create(&huntingId, NULL, wifly_thread, (void *)&common::uav);
		hunt_running = true;
		break;
	case 2:
		LOG_STATUS("[MOD] starting rfdetector thread...");
		pthread_create(&huntingId, NULL, rfhunter_thread, (void *)&common::uav);
		hunt_running = true;
		break;
	case 3:
		LOG_STATUS("[MOD] starting dirk antenna thread...");
		pthread_create(&huntingId, NULL, dirk_thread, (void *)&common::uav);
		hunt_running = true;
		break;
	}

	// ---------------------------------- //
	// wrap things up
	// ---------------------------------- //

	pthread_join(readId, NULL);
	
	if (hunt_running == true) {
		pthread_join(huntingId, NULL);
	}

	// close the pixhawk connection
	common::pixhawk->end_serial();


	/*
	// also create connection to df arduino if needed here
	if (common::emily) {
		df_arduino = new SerialPort(verbose, df_uart_name, baudrate);
	}
	*/

	// close the logfile
	fclose(common::output_logfile);
	return 0;
}

