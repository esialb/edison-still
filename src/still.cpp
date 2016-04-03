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

static bool read(struct xyz *p);

static int xyz_buf_size = 8;
static struct xyz *xyz_buf;
static int xyz_buf_pos = 0;

static int discard_time = 1000;
static float threshold = 0.1;

static int64_t ms();

static void xyz_subtract(struct xyz *p, struct xyz *q);
static float xyz_dist(struct xyz *p);
static float xpos_av(struct xyz *p, struct xyz *q, int n);

static char** parse_args(int argc, char **argv);

int main(int argc, char** argv) {
	char **command = parse_args(argc, argv);

	xyz_buf = (struct xyz *) malloc(xyz_buf_size * sizeof(struct xyz));

	struct xyz now;
	struct xyz av;
	float av_dist = 0;

	imu = new LSM9DS0(0x6B, 0x1D);
	imu->begin();

	bool calibrated = false;
	int calibration_samples = 0;
	for(;;) {
		struct xyz *p = xyz_buf + xyz_buf_pos;
		if((read(p))) {
			if(ms() < discard_time)
				continue;

			xyz_buf_pos = (xyz_buf_pos + 1) % xyz_buf_size;

			if(!calibrated) {
				if(++calibration_samples == xyz_buf_size) {
					av_dist = xpos_av(&av, xyz_buf, xyz_buf_size);
					for(int i = 0; i < xyz_buf_size; i++)
						xyz_subtract(xyz_buf + i, &av);
					calibrated = true;
				}
				continue;
			}

			xyz_subtract(p, &av);

			float now_dist = xpos_av(&now, xyz_buf, xyz_buf_size);

			if(calibrated && now_dist > threshold * av_dist) {
				if(command == NULL)
					return 0;
				execvp(*command, command);
			}
		} else if(calibrated)
			usleep(10000);
	}

	return 0;
}

static char** parse_args(int argc, char **argv) {
	vector<string> command;

	po::options_description visible;
	po::options_description hidden;
	po::options_description all;

	string buffer_help = (boost::format("sample buffer size (%1%)") % xyz_buf_size).str();
	string calibration_help = (boost::format("sample buffer initial discard ms (%1%)") % discard_time).str();
	string threshold_help = (boost::format("sample buffer deviation threshold (%1%)") % threshold).str();

	visible.add_options()
			("help", "show help")
			("buffer", po::value<int>(), buffer_help.c_str())
			("discard", po::value<int>(), calibration_help.c_str())
			("threshold", po::value<float>(), threshold_help.c_str())
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

	if(command.size() == 0)
		return NULL;

	char **cmd = (char**) malloc((command.size() + 1) * sizeof(char*));
	char **cc = cmd;
	for(size_t i = 0; i < command.size(); i++) {
		string s = command[i];
		*cc = (char*) malloc((s.size() + 1) * sizeof(char));
		strncpy(*cc, s.c_str(), s.size() + 1);
		cc++;
	}
	*cc = NULL;
	return cmd;
}

static void xyz_subtract(struct xyz *p, struct xyz *q) {
	p->x -= q->x;
	p->y -= q->y;
	p->z -= q->z;
}

static float xyz_dist(struct xyz *p) {
	return sqrt(p->x*p->x + p->y*p->y + p->z*p->z);
}

static float xpos_av(struct xyz *p, struct xyz *q, int n) {
	p->x = 0;
	p->y = 0;
	p->z = 0;
	for(int i = 0; i < n; i++) {
		p->x += q->x;
		p->y += q->y;
		p->z += q->z;
		q++;
	}
	p->x /= n;
	p->y /= n;
	p->z /= n;
	return xyz_dist(p);
}

static bool read(struct xyz *p) {
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

static int64_t ms() {
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
