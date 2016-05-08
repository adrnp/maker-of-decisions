# Maker of Decisions (MOD)
The decision making heart of the jammer tracking system.

Table of Contents
 - [Getting Started](#getting-started)
 - [Running the Code](#running-the-code)
 - [Branches](#branches)
 - [Structure](#structure)
 - [Bearing](#bearing)


## Getting Started ##

To download and prepare the code:

```
git clone https://github.com/adrnp/maker-of-decisions.git
cd maker-of-decisions
git submodule init
git submodule update
```

Once downloaded, you will need to run `cmake` before being able to make the project.  To do so, run the following command:

```
(cd build && cmake ..)
```

Then you should be ready to make, which can be done simply with:

```
(cd build && make)
```

Note that the reason for the `cd build` portion is that now with cmake, all the build files now exist in the build folder.  However, the executable in the end will be in the main directory, so to avoid having to `cd build` and then `cd ..` a bunch, the commands can simply be run at once with the above syntax.


## Running the Code ##
On the Odroid the code currently resides in `~/GitDocs/maker-of-decisions`.

#### Input Arguments ####
This script only takes at most a single input argument, `-c <config file>`, the configuration file to use.  Here is an example of starting the script.

```
./mod -c local_config.cfg
```

You can also start the script with no input parameters, e.g.

```
./mod
```

which will use the default `config.cfg` file.  Note that this probably won't work unless you plugged in the sensor(s) and the pixhawk in the same order as the config files assumes.

To get an idea of the different configurations available and how to set them, read through the default `config.cfg`, it's all very well commented.


## Branches ##

I try and keep a minimum of active branches at one time (limited to the current ongoing work), so below is a list of both the active branches and the old closed branches.

### Active ###
There are currently 3 active branches for MOD.

 - **master**
	 - Most up to date version of all the code that compiles successfully
	 - IS NOT GUARANTEED TO ACTUALLY WORK, HAS NOT BEEN FLIGHT TESTED
 - **stable**
	 - This branch contains the most recent updated version of the code that has been successfully flight tested and performs as desired
	 - DO NOT EDIT THIS BRANCH OR MERGE CHANGES INTO IT UNTIL THOSE CHANGES HAVE BEEN FLIGHT TESTED


### Closed ###
These are the branches that have been completed and therefore closed.

 - **pomdp_planning**
	 - This branch contained the skeletal framework for doing pomdp actions
	 - Merged into master post JIFX 16-1 successful flights
 - **restructuring**
 	 - Restructured a bunch of the different files (e.g. using classes and namespaces to contain variables and common threads of work)
 	 - Merged into master pre JIFX 16-3 in preperation for the event

## Structure ##

The code is still a bit of a mess, but a semblance of structure is being created.  The general idea is that helping code is being placed in different sub directories with each directory being a coherent thought.  Then there are a bunch of files that are not in any sub directory which are mostly the main bits of code required for MOD.

### Libraries ###
All of the "libraries" are contained in the **libs** directory.  This is a collection of standalone set of classes that are used throughout the main code, but for attempt at making the code easier to look at, have been moved into separate libraries.

 - **bearing**
	 - contains all of the files required for bearing calculation.  Any additional bearing calculation methods should be added to this directory
	 - To add bearing methods, create a .cpp file with all the functions you need.  Make sure to include `bearing.h` to your file and add the outwards facing bearing calculation function to the list of functions in `bearing.h`.  Your outwards facing function should be of the format `get_bearing_NAME(vector<double> &angles, vector<double> &gains, ...)`
	 - To use the bearing functions in a file, make sure to include `bearing/bearing.h` in your file
	 - [More details blow](#bearing)
 - **planners**
	 - Modular Planning Suite (MPS) see (insert link here)  submodule
	 - contains all the different planners that can be used for the decision making (a planner determines the next action given a current set of observations)
 - **serial**
	 - directory containing the definition of all the classes required for communication with the various serial devices.  Classes include the following:
		 - `SerialPort` - a generic serial port configuration (perfect for arduinos)
		 - `MavlinkSerial` - a mavlink specific serial port configuration (perfect for pixhawk)
		 - `WiflySerial` - a wifly specific serial port configuration (perfect for wifly)
 - **udp**
	 - contains a class to broadcast UDP messages that can then be parsed by the simple mod-udp-gs "ground station" (insert link here)

### Directories ###
Here is a list of additional sub directories that can be found and what they contain.  For specific details on each of the files inside these directories, please read through the files themselves for now.

 - **commands**
	 - directory containing the commands for different flight paths that can be commanded by the odroid (to be selected when starting up the script)
	 - to add a new command file, simply create a `.csv` file where each line is a command in the format `d_north,d_east,d_yaw,alt`
		 - d_north and d_east are distances in meters from the current position
		 - d_yaw is the angle through which to rotate before getting to next point if < 360, and is an absolute starting point if > 360
		 	- e.g. to start a rotatino at 40 degrees, send a command of 400
		 - altitude is meters AMSL
 - **mavlink**
	 - all the stuff required for mavlink, DO NOT EDIT DIRECTLY (it's a submodule)


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
 - rfhunter_thread.cpp
	 - This file contains the code that is running on a loop in the rf hunter thread started by MOD
	 - makes all the measurements from the rf detector
	 - understands which mode the vehicle is currently in and sends commands as needed to the pixhawk
 - dirk_thread.cpp
	 - This file contains all the code that is running on a loop in the dirk thread that can be started by MOD
	 - This would be run instead of the wifly thread in the case that the phased array antenna is being used
	 - Does the exact same thing as the wifly thread, except instead of communicating with a wifly, it is doing communication with an arduino
 - commander.cpp
	 - Contains all the helper functions to send messages and commands to the pixhawk
	 - Basically acts as the intermediary between the threads (reading the measurements), the planners, and the pixhawk
 - common.h
	 - Contains a declaration of all variables that are to be shared and accessible to any thread (or file) that includes it
	 - This is a bit of a brute force way of sharing information and makes it very important to be careful of what each of the files are doing to these variables
	 - To ensure thread safety of these variables, mod.cpp and read_thread.cpp are the only 2 files that should be writing to any of these variables (and they write to them at different times), every other file should only be reading from these variables!!!


## Bearing ##
The bearing files currently consist of:

 - bearing.h
 - bearing\_helper.cpp
 - bearing\_mle.cpp
 - bearing\_cc.cpp
 - bearing\_max.cpp
 - beaing\_max3.cpp
 - obs\_model.csv

The function prototypes are stored in the header file `bearing.h`.
The file `bearing\_helper.cpp` contains functions that I imagine many bearing methods will require.
The files `bearing\_mle.cpp` and `bearing\_cc.cpp` contain two methods for getting the bearing.
The model required to make some of these methods work is stored in `obs\_model.csv`.

The get\_bearing functions currently implemented are:

```c++
get_bearing_mle(vector<double> angles, vector<int> norm_gains);
get_bearing_cc(vector<double>angles, vector<double> dir_gains);
get_bearing_max(vector<double>angles, vector<double> dir_gains);
get_bearing_max3(vector<double>angles, vector<double> dir_gains);
```

The MLE method can be called before a full rotation is made.
The cross-correlation (cc) method requires a full rotation before it will work.

To get the normalized gains, you can use the function *gains2normgain* located in `bearing\_helper.cpp`. 
It takes in the directional antenna gain and the omnidirectional antenna gain as arguments (in that order).
The argument gains can be either ints or doubles, but they must both be of the same type.
The output normalized gain is an integer.


## Odroid Specific Comments ##
Here are some specific instructions for the odroid we have configured for research and does not necessarily apply directly to the maker of decision script.

### Testing the Pixhawk Connection (if things don't seem to be working) ###
If for some reason you need to test whether or not the connection to the pixhawk is working, you can run the ground station software on the Odroid to check the connection.  The groundstation software can be found in `~/GitDocs/jager_gs/`.

The the groundstation can be started with the following command:

```
./jager_gs -b 115200 -d /dev/ttyUSB1
```

Once connected you should see a message appear with the heartbeat information (almost instantly, at most a couple seconds after connected).  If you do not see this message and are hardwired into the pixhawk, then you have the wrong device and try another ttyUSB#.

