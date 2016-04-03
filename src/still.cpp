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

static bool initialized = false;

static int64_t clockstart_ns;

static LSM9DS0 *imu;

struct xpos {
	float x;
	float y;
	float z;
	float dist;
};

static bool read(struct xpos *p);

#define XPOSBUFSIZE 8
static struct xpos *xposbuf;
static int xposbuf_pos = 0;

#define DISCARD_TIME 1000

static int64_t ms();

static void xpos_subtract(struct xpos *p, struct xpos *q);
static float xpos_dist(struct xpos *p);
static void xpos_av(struct xpos *p);


int main(int argc, char** argv) {
	xposbuf = (struct xpos *) malloc(XPOSBUFSIZE * sizeof(struct xpos));

	struct xpos av;

	imu = new LSM9DS0(0x6B, 0x1D);
	imu->begin();

	bool calibrated = false;
	int calibration_samples = 0;
	for(;;) {
		struct xpos *p = xposbuf + xposbuf_pos;
		if((read(p))) {
			if(ms() < DISCARD_TIME)
				continue;

			xposbuf_pos = (xposbuf_pos + 1) % XPOSBUFSIZE;

			if(!calibrated) {
				if(++calibration_samples == XPOSBUFSIZE) {
					xpos_av(&av);
					for(int i = 0; i < XPOSBUFSIZE; i++)
						xpos_subtract(xposbuf + i, &av);
					calibrated = true;
				}
			} else
				xpos_subtract(p, &av);

			struct xpos now;
			xpos_av(&now);

			if(calibrated && now.dist > 0.1 * av.dist) {
				if(argc == 1)
					return 0;
				char **args = (char **) malloc((argc) * sizeof(char*));
				memcpy(args, argv+1, (argc - 1) * sizeof(char *));
				args[argc - 1] = NULL;
				execvp(args[0], args);
			}
		} else if(calibrated)
			usleep(10000);
	}

	return 0;
}

static void xpos_subtract(struct xpos *p, struct xpos *q) {
	p->x -= q->x;
	p->y -= q->y;
	p->z -= q->z;
	p->dist = xpos_dist(p);
}

static float xpos_dist(struct xpos *p) {
	return sqrt(p->x*p->x + p->y*p->y + p->z*p->z);
}

static void xpos_av(struct xpos *p) {
	p->x = 0;
	p->y = 0;
	p->z = 0;
	struct xpos *q;
	for(int i = 0; i < XPOSBUFSIZE; i++) {
		q = xposbuf + i;
		p->x += q->x;
		p->y += q->y;
		p->z += q->z;
	}
	p->x /= XPOSBUFSIZE;
	p->y /= XPOSBUFSIZE;
	p->z /= XPOSBUFSIZE;
	p->dist = xpos_dist(p);
}

static bool read(struct xpos *p) {
	if(imu->newXData()) {
		while(imu->newXData()) {
			imu->readAccel();
			p->x = imu->calcAccel(imu->ax);
			p->y = imu->calcAccel(imu->ay);
			p->z = imu->calcAccel(imu->az);
			p->dist = xpos_dist(p);
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
