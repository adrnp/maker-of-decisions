# Maker of Decisions (MOD)
The decision making hard of the jammer tracking system.

### Getting Started

To download and prepare the code:

        git clone https://github.com/adrnp/maker-of-decisions.git
        cd maker-of-decisions
        git submodule init
        git submodule update

Then you should be ready to make, which can be done simply with:

        make

### Running the Code
On the Odroid the code currently resides in `~/GitDocs/maker-of-decisions` or something like.

To run the code, you will need access to the command line.  If running it from the odroid (with a hardwire connection to the pixhawk) run the following command:

        ./mod -b 115200 -d /dev/ttyUSB1

If running the code with a wireless connection to the pixhawk:

        ./mod -b 57600 -d /dev/ttyUSB#

where # is the number of the port on which the 3DR antenna is connected.

**Note:** the wifly antenna must be connected first as it is hardcoded as */dev/ttyUSB0*

Some explanation:
 - `-b 115200` sets the baud rate to be 115200
 - `-d /dev/ttyUSB1` sets the connection to the pixhawk, not to the wifly!! (wifly is hardcoded as */dev/ttyUSB0*)

### Testing the Pixhawk Connection (if things don't seem to be working)
If for some reason you need to test whether or not the connection to the pixhawk is working, you can run the ground station software on the Odroid to check the connection.  The groundstation software can be found in `~/GitDocs/jager_gs/` (not sure if the folder is actually called GitDocs).

The the groundstation can be started with the following command:

        ./jager_gs -b 115200 -d /dev/ttyUSB1

Once connected you should see a message appear with the heartbeat information (almost instantly, at most a couple seconds after connected).  If you do not see this message and are hardwired into the pixhawk, then you have the wrong device and try another ttyUSB#.

### Bearing
The bearing files currently consist of:
* bearing.h
* bearing\_helper.cpp
* bearing\_mle.cpp
* bearing\_cc.cpp
* obs\_model.csv

The function prototypes are stored in the header file bearing.h.
The file bearing\_helper.cpp contains functions that I imagine many bearing methods will require.
The files bearing\_mle.cpp and bearing\_cc.cpp contain two methods for getting the bearing.
The model required to make some of these methods work is stored in obs\_model.csv.

The get\_bearing functions currently implemented are:
* get\_bearing\_mle(vector\<double\> angles, vector<int> norm\_gains)
* get\_bearing\_cc(vector<double>angles, vector<double> dir\_gains)
