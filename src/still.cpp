//============================================================================
// Name        : eit-logger.cpp
// Author      : Robin Kirkman
// Version     :
// Copyright   : BSD
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <unistd.h>

#include "SFE_LSM9DS0.h"

#include <stdint.h>
#include <time.h>

#include <math.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <linux/watchdog.h>

#include <boost/program_options.hpp>
#include <boost/format.hpp>

namespace po = boost::program_options;

using namespace std;

static bool initialized = false;

static int64_t clockstart_ns;

static LSM9DS0 *imu;

struct xyz {
	float x;
	float y;
	float z;
};

static int xyz_buf_size = 8;
static int discard_time = 1000;
static float threshold = 0.01;

static struct xyz *xyz_buf;
static int xyz_buf_pos = 0;

static char **trigger_command;

#define WATCHDOG_DEV "/dev/watchdog"
#define WATCHDOG_DISABLE "/sys/devices/virtual/misc/watchdog/disable"
static bool watchdog = false;
static int watchdog_timeout = 2;
static int watchdog_fd;

static int64_t timestamp_ms();

static bool xyz_read_accel(struct xyz *p);
static struct xyz *xyz_add(struct xyz *p, struct xyz *q);
static struct xyz *xyz_subtract(struct xyz *p, struct xyz *q);
static struct xyz *xyz_mean(struct xyz *p, struct xyz *q, int n);
static float xyz_magnitude(struct xyz *p);

int main(int argc, char **argv);
static void parse_args(int argc, char **argv);
static void init_watchdog();
static void trigger();

int main(int argc, char** argv) {
	parse_args(argc, argv);

	xyz_buf = (struct xyz *) malloc(xyz_buf_size * sizeof(struct xyz));

	struct xyz calibrated_mean;
	struct xyz current_mean;
	float calibrated_magnitude = 0;

	imu = new LSM9DS0(0x6B, 0x1D);
	imu->begin();

	imu->setAccelScale(LSM9DS0::A_SCALE_2G);
	imu->setAccelODR(LSM9DS0::A_ODR_50);
	imu->setAccelABW(LSM9DS0::A_ABW_50);

	if(watchdog)
		init_watchdog();

	bool calibrated = false;
	int calibration_samples = 0;
	for(;;) {
		struct xyz *p = xyz_buf + xyz_buf_pos;
		if((xyz_read_accel(p))) {
			if(watchdog)
				ioctl(watchdog_fd, WDIOC_KEEPALIVE, 0);

			if(timestamp_ms() < discard_time)
				continue;

			xyz_buf_pos = (xyz_buf_pos + 1) % xyz_buf_size;

			if(!calibrated) {
				if(++calibration_samples == xyz_buf_size) {
					calibrated_magnitude = xyz_magnitude(xyz_mean(&calibrated_mean, xyz_buf, xyz_buf_size));
					for(int i = 0; i < xyz_buf_size; i++)
						xyz_subtract(xyz_buf + i, &calibrated_mean);
					calibrated = true;
				}
				continue;
			}

			xyz_subtract(p, &calibrated_mean);

			float current_magnitude = xyz_magnitude(xyz_mean(&current_mean, xyz_buf, xyz_buf_size));

			if(current_magnitude > threshold * calibrated_magnitude || imu->xDataOverflow())
				trigger();
		} else if(calibrated)
			usleep(10000);
	}

	return 0;
}

static void parse_args(int argc, char **argv) {
	vector<string> command;

	po::options_description visible;
	po::options_description hidden;
	po::options_description all;

	string buffer_help = (boost::format("sample buffer size (%1%)") % xyz_buf_size).str();
	string calibration_help = (boost::format("sample buffer initial discard ms (%1%)") % discard_time).str();
	string threshold_help = (boost::format("sample buffer deviation threshold (%1%)") % threshold).str();
	string watchdog_help = (boost::format("enable watchdog timer (%1%)") % (watchdog ? "true" : "false")).str();
	string watchdog_timeout_help = (boost::format("specify watchdog timer timeout (%1%)") % watchdog_timeout).str();

	visible.add_options()
			("help", "show this help")
			("buffer", po::value<int>(), buffer_help.c_str())
			("discard", po::value<int>(), calibration_help.c_str())
			("threshold", po::value<float>(), threshold_help.c_str())
			("watchdog", po::value<bool>(), watchdog_help.c_str())
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

static void init_watchdog() {
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

static void trigger() {
	if(watchdog)
		close(watchdog_fd);
	if(!trigger_command)
		exit(0);
	execvp(*trigger_command, trigger_command);
}

static struct xyz *xyz_add(struct xyz *p, struct xyz *q) {
	p->x += q->x;
	p->y += q->y;
	p->z += q->z;
	return p;
}

static struct xyz *xyz_subtract(struct xyz *p, struct xyz *q) {
	p->x -= q->x;
	p->y -= q->y;
	p->z -= q->z;
	return p;
}

static float xyz_magnitude(struct xyz *p) {
	return sqrt(p->x*p->x + p->y*p->y + p->z*p->z);
}

static struct xyz *xyz_mean(struct xyz *p, struct xyz *q, int n) {
	p->x = p->y = p->z = 0;
	for(int i = 0; i < n; i++)
		xyz_add(p, q++);
	p->x /= n;
	p->y /= n;
	p->z /= n;
	return p;
}

static bool xyz_read_accel(struct xyz *p) {
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

static int64_t timestamp_ms() {
	struct timespec clk;
	clock_gettime(CLOCK_MONOTONIC, &clk);
	int64_t ns = clk.tv_sec;
	ns *= 1000;
	ns += clk.tv_nsec / 1000000;
	if(initialized) {
		return ns - clockstart_ns;
	} else {
		clockstart_ns = ns;
		initialized = true;
		return 0;
	}
}
