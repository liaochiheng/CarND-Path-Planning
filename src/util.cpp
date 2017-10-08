#include <math.h>
#include <vector>
#include <string>
#include "util.hpp"

using namespace std;

namespace util
{
	extern double pi() { return M_PI; }

	extern double deg2rad(double x) { return x * pi() / 180; }

	extern double rad2deg(double x) { return x * 180 / pi(); }

	extern double distance(double x1, double y1, double x2, double y2) {
		return sqrt( (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) );
	}
}
