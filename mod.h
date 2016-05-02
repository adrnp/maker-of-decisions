/**
 * @file mod.h
 *
 * The main running function for maker of decision.
 * 
 * @Author Adrien Perkins <adrienp@stanford.edu>
 */


#ifndef MOD_H_
#define MOD_H_


/**
 * stops all running threads with Ctrl-C
 * @param sig  the interupt signal
 */
void quit_handler(int sig);

/**
 * read in the config file
 * @param  argc  command line arguments
 * @param  argv  command line values
 * @return       -1 for error
 */
int get_configuration(int argc, char **argv);

/**
 * check to see if a directory exists and create it if it doesn't
 * @param  path  the desired directory path
 * @param  mode  the permissions for the directory
 * @return       -1 for error
 */
int create_directory(const char *path, mode_t mode);

/**
 * convert an integer to a string with that integer zeropaded to a given length
 * @param  val  integer value to convert
 * @param  len  the full zeropadded length
 * @return      the zeropadded string
 */
std::string to_zeropad_string(const int &val, const int &len);

/**
 * create all the folders needed for the logging and populate the logfile dir variable
 * @return  -1 for error
 */
int setup_logfiles();

/**
 * main function
 * @param  argc  command line arguments
 * @param  argv  command line values
 * @return       standard return
 */
int main(int argc, char **argv);


#endif /* MOD_H_ */
