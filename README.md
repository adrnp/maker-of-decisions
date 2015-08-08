# Maker of Decisions (MOD)
The decision making heart of the jammer tracking system.

## Getting Started

To download and prepare the code:

        git clone https://github.com/adrnp/maker-of-decisions.git
        cd maker-of-decisions
        git submodule init
        git submodule update

Then you should be ready to make, which can be done simply with:

        make

## Branches ##
There are currently 3 active branches for MOD.

 - **master**
	 - Most up to date version of all the code that compiles successfully
	 - IS NOT GUARANTEED TO ACTUALLY WORK, HAS NOT BEEN FLIGHT TESTED
 - **stable**
	 - This branch contains the most recent updated version of the code that has been successfully flight tested and performs as desired
	 - DO NOT EDIT THIS BRANCH OR MERGE CHANGES INTO IT UNTIL THOSE CHANGES HAVE BEEN FLIGHT TESTED
 - **pomdp_planning**
	 - This branch contains the skeletal framework for doing pomdp actions

## Structure ##

The code is still a bit of a mess, but a semblance of structure is being created.  The general idea is that helping code is being placed in different sub directories with each directory being a coherent thought.  Then there are a bunch of files that are not in any sub directory which are mostly the main bits of code required for MOD.

### Directories ###
Here is a list of the current sub directories that can be found and what they contain.  For specific details on each of the files inside these directories, please read through the files themselves for now.

 - **bearing_lib**
	 - directory containing all of the files required for bearing calculation.  Any additional bearing calculation methods should be added to this directory
	 - To add bearing methods, create a .cpp file with all the functions you need.  Make sure to include `bearing.h` to your file and add the outwards facing bearing calculation function to the list of functions in `bearing.h`.  Your outwards facing function should be of the format `get_bearing_NAME(vector<double> &angles, vector<double> &gains, ...)`
	 - To use the bearing functions in a file, make sure to include `bearing/bearing.h` in your file
	 - [More details blow](#bearing)
 - **serial_lib**
	 - directory containing the definition of all the classes required for communication with the various serial devices.  Classes include the following:
		 - `SerialPort` - a generic serial port configuration (perfect for arduinos)
		 - `MavlinkSerial` - a mavlink specific serial port configuration (perfect for pixhawk)
		 - `WiflySerial` - a wifly specific serial port configuration (perfect for wifly)
 - **commands**
	 - directory containing the commands for different flight paths that can be commanded by the odroid (to be selected when starting up the script)
	 - to add a new command file, simply create a `.csv` file where each line is a command in the format `d_north,d_east,alt`
		 - d_north and d_east are distances in meters from the current position
		 - altitude is meters AMSL
 - **mavlink**
	 - all the stuff required for mavlink, DO NOT EDIT DIRECTLY (it's a submodule)
 - **planner**
 - **policyExecution**

### Files ###
Here is a brief overview of some of the more important standalone files in the project.

 - mod.cpp
	 - This is the file that contains the `void main()` function that is actually executed when the program run
	 - Reading in of all the starting parameters also takes placed here
	 - NOTE: the main function basically just handles the starting and stopping of the different threads that are running, with each thread having its own file
 - read_thread.cpp
	 - This file contains all the code that is running on a loop in the read threa started by MOD
	 - reads in all the data from the pixhawk and updates state information that is accessible in other threads
	 - basically updates MOD's knowledge of the pixhawk's state, etc
 - wifly_thread.cpp
	 - This file contains the code that is running on a loop in the wifly thread started by MOD
	 - makes all the measurements from the wifly
	 - understands which mode the vehicle is currently in and sends commands as needed to the pixhawk
 - dirk_thread.cpp
	 - This file contains all the code that is running on a loop in the dirk thread that can be started by MOD
	 - This would be run instead of the wifly thread in the case that the phased array antenna is being used
	 - Does the exact same thing as the wifly thread, except instead of communicating with a wifly, it is doing communication with an arduino
 - commander.cpp
	 - Contains all the helper functions to send messages and commands to the pixhawk
 - common.h
	 - Contains a declaration of all variables that are to be shared and accessible to any thread (or file) that includes it
	 - This is a bit of a brute force way of sharing information and makes it very important to be careful of what each of the files are doing to these variables
	 - To ensure thread safety of these variables, mod.cpp and read_thread.cpp are the only 2 files that should be writing to any of these variables (and they write to them at different times), every other file should only be reading from these variables!!!


## Running the Code
On the Odroid the code currently resides in `~/GitDocs/maker-of-decisions`.

#### Input Arguments ####
There are a variety of different input arguments that need to be and can be passed to the script on startup to change the behavior of the script.  Here is a list of all the available flags and arguments:

    -d / --device	# port the pixhawk is connected to
	-b / --baud		# baudrate of the pixhawk connection
	
	-w1	/ --wifly1	# port the directional wifly antenna is connected to
	-w2 / --wifly2	# port the omnidirectional wifly antenna is connected to (if present)

	-pa / --phased	# flag to mark using the phased array configuration
	-e / --emily	# flag to mark using the direction finding configuration

	-c / --commands	# flag to use commands odroid commands, can specify specific file
	-t / --track	# flag to use tracking logic (defaults to naive)

	-v / --verbose	# flag to have verbose output (for debugging purposes)
	-nw / --nowifly	# flag to mark no wifly is connected (for debugging purposes)

To run the code, the `-d`, `-b`, and `-w1` parameters are the minimum required set to be able to successfully start the script.  Anything beyond that is optional is further configures the running of MOD.

Some of the flags are mutually exclusive, for example you cannot run with both a `-nw` and `-w1` or `-w2` flag, as that would be a contradictory condition.  You also cannot run both a tracking (`-t`) and command (`-c`) simultaneously.  If both tracking and command flags are set, a tracking mission will be executed.

#### Examples ####

Here is a basic example of connecting to a pixhawk located at `/dev/ttyUSB1` with a baudrate of 115200 and a directional antenna connected to `/dev/ttyUSB0`:

		./mod -d /dev/ttyUSB1 -b 115200 -w1 /dev/ttyUSB0

For a more complex example, let's say in addition to the previous configuration you have a command file `box.csv` that you want the odroid to run through, you would start the script with the following command:

		./mod -d /dev/ttyUSB1 -b 115200 -w1 /dev/ttyUSB0 -c box

For an even more complex example, let's say you have a pixhawk connected to `/dev/ttyUSB1`, a directional antenna connected to `/dev/ttyUSB0` and an omnidirectional antenna connected to `/dev/ttyUSB2` and you want to execute a naive tracking mission with a verbose output for debugging, you would run the following command:

		./mod -d /dev/ttyUSB1 -b 115200 -w1 /dev/ttyUSB0 -w2 /dev/ttyUSB2 -t -v



## Testing the Pixhawk Connection (if things don't seem to be working)
If for some reason you need to test whether or not the connection to the pixhawk is working, you can run the ground station software on the Odroid to check the connection.  The groundstation software can be found in `~/GitDocs/jager_gs/`.

The the groundstation can be started with the following command:

        ./jager_gs -b 115200 -d /dev/ttyUSB1

Once connected you should see a message appear with the heartbeat information (almost instantly, at most a couple seconds after connected).  If you do not see this message and are hardwired into the pixhawk, then you have the wrong device and try another ttyUSB#.

## Bearing
The bearing files currently consist of:

 - bearing.h
 - bearing\_helper.cpp
 - bearing\_mle.cpp
 - bearing\_cc.cpp
 - bearing\_max.cpp
 - obs\_model.csv

The function prototypes are stored in the header file bearing.h.
The file bearing\_helper.cpp contains functions that I imagine many bearing methods will require.
The files bearing\_mle.cpp and bearing\_cc.cpp contain two methods for getting the bearing.
The model required to make some of these methods work is stored in obs\_model.csv.

The get\_bearing functions currently implemented are:

 - `get_bearing_mle(vector<double> angles, vector<int> norm_gains)`
 - `get_bearing_cc(vector<double>angles, vector<double> dir_gains)`
 - `get_bearing_max(vector<double>angles, vector<double> dir_gains)`

The MLE method can be called before a full rotation is made.
The cross-correlation (cc) method requires a full rotation before it will work.

To get the normalized gains, you can use the function `gains2normgain` located in bearing\_helper.cpp. 
It takes in the directional antenna gain and the omnidirectional antenna gain as arguments (in that order).
The argument gains can be either ints or doubles, but they must both be of the same type.
The output normalized gain is an integer.


