#include <math.h>
#include "vehicle.h"
#include "util.hpp"

using namespace std;
using namespace util;

Vehicle::Vehicle() {
	this->id = -1;
	this->state = "KL"; // Keep Lane
}

Vehicle::Vehicle(double id) {
	this->id = id;
	this->state = "KL"; // Keep Lane
}

Vehicle::~Vehicle() {}

void Vehicle::update(double x, double y, double vx, double vy, double s, double d) {
	this->x = x;
	this->y = y;
	this->vx = vx;
	this->vy = vy;
	this->s = s;
	this->d = d;
}

double Vehicle::speed() {
	return sqrt( vx * vx + vy * vy );
}

Vehicle Vehicle::pos_at( double t ) {
	Vehicle veh( this->id );
	veh.update( x + vx * t, y + vy * t, vx, vy,
		s + sqrt( vx * vx + vy * vy ) * t, d );
	return veh;
}