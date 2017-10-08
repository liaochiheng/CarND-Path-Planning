#ifndef UTIL_HPP
#define UTIL_HPP
#include <math.h>
#include <vector>
#include <string>

using namespace std;

namespace util
{

	const double LANE_WIDTH = 4.0;

	const double SAFETY_DIST = 30.0; // safety distance to front vehicle(m)

	const double MIN_DIST = 10.0; // min distance to front vehicle(m)

	const double LC_DIST = 30.0; // lane change distance in Frenet-s

	const double MAX_VEL = 49.5 / 2.24;

	const double MAX_ACC = 10.0;

	const double MAX_JERK = 40.0;

	extern double pi();

	extern double deg2rad(double x);

	extern double rad2deg(double x);

	extern double distance(double x1, double y1, double x2, double y2);
}

#endif