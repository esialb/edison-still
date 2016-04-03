/*
 * still [options] [[--] command [args...]]
 *
 * Poll LSM9DS0 accelerometer and (optionally) run a command when it detects movement.
 * If command is not specified, exit instead.
 *
 * Specifying a command:
 * --: separate options to still from the command to run
 * command: the optional command to run
 * args...: additional arguments for the command
 *
 * Configuring the trigger:
 * --buffer n: set the size of the accelerometer sample buffer to n
 * --discard ms: discard all sensor readings for the first ms milliseconds
 * --threshold t: trigger threshold for deviation from calibrated mean,
 * 		as a fraction of the calibrated mean magnitude
 *
 * Using the watchdog timer
 * --watchdog: open /dev/watchdog and write to it for every sample
 * --timeout t: set the trigger timeout for /dev/watchdog, implies --watchdog
 */

#include <iostream>
#include <unistd.h>
#include <stdint.h>
#include <time.h>
#include <math.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <linux/watchdog.h>
#include <boost/program_options.hpp>
#include <boost/format.hpp>

#include "SFE_LSM9DS0.h"

namespace po = boost::program_options;

using namespace std; // typing std:: all the time is annoying

/*
 * A simple (x,y,z) coordinate
 */
struct xyz {
	float x;
	float y;
	float z;
};

/*
 * The LSM9DS0 interface as implemented by SparkFun
 */
static LSM9DS0 *imu;

/*
 * The command to run when triggered.  NULL indicates that triggers should call exit(0)
 * instead of execvp'ing a command.
 */
static char **trigger_command;

/*
 * The number of xyz samples to buffer for noise smoothing
 */
static int xyz_buf_size = 8;
/*
 * Duration after startup when all samples should be discarded
 */
static int discard_time = 1000;
/*
 * Trigger threshold, specified as a fraction of the magnitude of the calibrated
 * mean (x,y,z) coordinate.  If the distance from the calibrated mean to the current mean
 * is greater than the calibrated magnitude times this threshold, the command
 * is triggered.
 */
static float threshold = 0.01;

/*
 * Array of accelerometer samples
 */
static struct xyz *xyz_buf;
/*
 * Position of the next sample in xyz_buf
 */
static int xyz_buf_pos = 0;

/*
 * Path to the watchdog timer device
 */
#define WATCHDOG_DEV "/dev/watchdog"
/*
 * Is writing to the watchdog timer enabled?
 */
static bool watchdog = false;
/*
 * Watchdog timer timeout (seconds)
 */
static int watchdog_timeout = 2;
/*
 * File descriptor of the open watchdog timer device
 */
static int watchdog_fd;

/*
 * System clock (ms) when timestamp_ms() was first called
 */
static int64_t timestamp_clockstart;
/*
 * Has timestamp_ms() been called yet?
 */
static bool timestamp_initialized = false;
/*
 * Return a timestamp in milliseconds.  Returns zero from
 * the first invocation, and the time since zero for all
 * subsequent invocations.
 */
static int64_t timestamp_ms();

/*
 * Read the accelerometer and write to a coordinate
 */
static bool xyz_read_accel(struct xyz *p);
/*
 * Add the coordinate values *q to those in *p, returning p
 */
static struct xyz *xyz_add(struct xyz *p, struct xyz *q);
/*
 * Subtract the coordinate values *q from this in *p, returning p
 */
static struct xyz *xyz_subtract(struct xyz *p, struct xyz *q);
/*
 * Write the mean coordinate values of the n-length array starting with *q into *p, returning p
 */
static struct xyz *xyz_mean(struct xyz *p, struct xyz *q, int n);
/*
 * Return the magnitude of *p
 */
static float xyz_magnitude(struct xyz *p);

/*
 * Main program entry
 */
int main(int argc, char **argv);

/*
 * Parse command-line arguments and set options
 */
static void parse_args(int argc, char **argv);
/*
 * Initializes the watchdog timer and begins ticking
 */
static void init_watchdog();
/*
 * Trigger the command
 */
static void trigger();

int main(int argc, char** argv) {
	// set options based on args
	parse_args(argc, argv);

	// coordinate buffer and means
	xyz_buf = (struct xyz *) malloc(xyz_buf_size * sizeof(struct xyz));
	struct xyz calibrated_mean;
	struct xyz current_mean;
	float calibrated_magnitude = 0;

	// access the IMU
	imu = new LSM9DS0(0x6B, 0x1D);
	imu->begin();

	// set IMU to 2G scale at 50Hz (IMU overflow will trigger the command)
	imu->setAccelScale(LSM9DS0::A_SCALE_2G);
	imu->setAccelODR(LSM9DS0::A_ODR_50);
	imu->setAccelABW(LSM9DS0::A_ABW_50);

	if(watchdog) // maybe initialize watchdog timer device
		init_watchdog();


	// how many samples (out of xyz_buf_size required) have been collected for calibration?
	int calibration_samples = 0;
	// has calibration finished?
	bool calibrated = false;

	for(;;) {
		struct xyz *p = xyz_buf + xyz_buf_pos;
		if((xyz_read_accel(p))) {
			if(watchdog) // tick the watchdog if enabled
				ioctl(watchdog_fd, WDIOC_KEEPALIVE, 0);

			if(timestamp_ms() < discard_time) // discard early points for excessive noise
				continue;

			xyz_buf_pos = (xyz_buf_pos + 1) % xyz_buf_size; // advance next buffer slot

			if(!calibrated) { // if still collecting calibration points
				if(++calibration_samples == xyz_buf_size) { // if we got enough points
					xyz_mean(&calibrated_mean, xyz_buf, xyz_buf_size); // calibrated mean
					calibrated_magnitude = xyz_magnitude(&calibrated_mean); // calibrated magnitude
					// renormalize the point buffer from the calibrated mean
					for(int i = 0; i < xyz_buf_size; i++)
						xyz_subtract(xyz_buf + i, &calibrated_mean);
					calibrated = true; // done calibrating
				}
				continue;
			}

			xyz_subtract(p, &calibrated_mean); // renormalize the point from the calibrated mean

			xyz_mean(&current_mean, xyz_buf, xyz_buf_size); // update current mean
			float current_magnitude = xyz_magnitude(&current_mean); // current mean distance from calibrated mean

			// trigger if accelerometer coordinates changed enough, or if there was an overflow
			if(current_magnitude > threshold * calibrated_magnitude || imu->xDataOverflow())
				trigger();
		} else if(calibrated) // if already calibrated
			usleep(10000); // sleep 10ms
	}

	return 0;
}

static void parse_args(int argc, char **argv) { // parse args
	vector<string> command;

	po::options_description visible;
	po::options_description hidden;
	po::options_description all;

	string buffer_help = (boost::format("sample buffer size (%1%)") % xyz_buf_size).str();
	string calibration_help = (boost::format("sample buffer initial discard ms (%1%)") % discard_time).str();
	string threshold_help = (boost::format("sample buffer deviation threshold (%1%)") % threshold).str();
	string watchdog_help = string("enable watchdog timer");
	string watchdog_timeout_help = (boost::format("specify watchdog timer timeout (%1%)") % watchdog_timeout).str();

	visible.add_options()
			("help", "show this help")
			("buffer", po::value<int>(), buffer_help.c_str())
			("discard", po::value<int>(), calibration_help.c_str())
			("threshold", po::value<float>(), threshold_help.c_str())
			("watchdog", watchdog_help.c_str())
			("timeout", po::value<int>(), watchdog_timeout_help.c_str())
			;
	hidden.add_options()
			("command", po::value(&command))
			;
	all.add(visible).add(hidden);


	po::positional_options_description pdesc;
	pdesc.add("command", -1);

	po::variables_map vm;
	po::basic_parsed_options<char> parsed = po::command_line_parser(argc, argv).
			options(all).
			positional(pdesc).
			allow_unregistered().
			run();

	po::store(parsed, vm);
	po::notify(vm);

	bool unrecognized = po::collect_unrecognized(parsed.options, po::exclude_positional).size() > 0;

	if(vm.count("help") || unrecognized) {
		cout << "usage: " << *argv << " [options] [[--] command [args...]]\n";
		cout << "waits until the LSM9DS0 accelerometer detects movement, then optionally executes a command\n\n";
		cout << "options:\n";
		cout << visible;
		exit(unrecognized ? -1 : 0);
	}
	if(vm.count("buffer"))
		xyz_buf_size = vm["buffer"].as<int>();
	if(vm.count("discard"))
		discard_time = vm["discard"].as<int>();
	if(vm.count("threshold"))
		threshold = vm["threshold"].as<float>();
	if(vm.count("watchdog"))
		watchdog = true;
	if(vm.count("timeout")) {
		watchdog = true;
		watchdog_timeout_help = vm["timeout"].as<int>();
	}

	if(command.size() == 0)
		trigger_command = NULL;
	else {
		char **cc = trigger_command = (char**) malloc((command.size() + 1) * sizeof(char*));
		for(size_t i = 0; i < command.size(); i++) {
			string s = command[i];
			*cc = (char*) malloc((s.size() + 1) * sizeof(char));
			strncpy(*cc, s.c_str(), s.size() + 1);
			cc++;
		}
		*cc = NULL;
	}
}

static void init_watchdog() { // set up watchdog timer device
	watchdog_fd = open(WATCHDOG_DEV, O_WRONLY);
	if(watchdog_fd >= 0) {
		int options = WDIOS_ENABLECARD;
		ioctl(watchdog_fd, WDIOC_SETOPTIONS, &options);

		int timeout = watchdog_timeout;
		ioctl(watchdog_fd, WDIOC_SETTIMEOUT, &timeout);
		if(timeout != watchdog_timeout)
			cerr << "tried to set watchdog timeout to " << watchdog_timeout << " but actually set to " << timeout << "\n";
		watchdog_timeout = timeout;

		ioctl(watchdog_fd, WDIOC_KEEPALIVE, 0);
	} else {
		cerr << "unable to open " WATCHDOG_DEV ", disabling watchdog support\n";
		watchdog = false;
	}
}

static void trigger() { // trigger the command
	if(watchdog) // close the watchdog timer device so execvp'd command can't write to it
		close(watchdog_fd);
	if(!trigger_command)
		exit(0);
	execvp(*trigger_command, trigger_command);
}

static struct xyz *xyz_add(struct xyz *p, struct xyz *q) { // add a coordinates
	p->x += q->x;
	p->y += q->y;
	p->z += q->z;
	return p;
}

static struct xyz *xyz_subtract(struct xyz *p, struct xyz *q) { // subtract a coordinate
	p->x -= q->x;
	p->y -= q->y;
	p->z -= q->z;
	return p;
}

static float xyz_magnitude(struct xyz *p) { // coordinate magnitude
	return sqrt(p->x*p->x + p->y*p->y + p->z*p->z);
}

static struct xyz *xyz_mean(struct xyz *p, struct xyz *q, int n) { // mean coordinate
	p->x = p->y = p->z = 0;
	for(int i = 0; i < n; i++)
		xyz_add(p, q++);
	p->x /= n;
	p->y /= n;
	p->z /= n;
	return p;
}

static bool xyz_read_accel(struct xyz *p) { // read coordinate from IMU
	if(imu->newXData()) {
		while(imu->newXData()) {
			imu->readAccel();
			p->x = imu->calcAccel(imu->ax);
			p->y = imu->calcAccel(imu->ay);
			p->z = imu->calcAccel(imu->az);
		}
		return true;
	} else
		return false;
}

static int64_t timestamp_ms() { // ms since first invocation of timestamp_ms()
	struct timespec clk;
	clock_gettime(CLOCK_MONOTONIC, &clk);
	int64_t ms = clk.tv_sec * 1000;
	ms += clk.tv_nsec / 1000000;
	if(timestamp_initialized) {
		return ms - timestamp_clockstart;
	} else {
		timestamp_clockstart = ms;
		timestamp_initialized = true;
		return 0;
	}
}
