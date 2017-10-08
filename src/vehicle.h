#ifndef VEHICLE_H
#define VEHICLE_H
#include <vector>
#include <string>
#include "util.hpp"

using namespace std;
using namespace util;

class Vehicle {

public:

	double id;

	double x;
	double y;
	double vx;
	double vy;
	double s;
	double d;

	string state;

	/**
	* Default Constructor
	*/
	Vehicle();

		/**
	* Constructor
	*/
	Vehicle(double id);

	/**
	* Destructor
	*/
	virtual ~Vehicle();

	// update all the state variables
	void update(double x, double y, double vx, double vy, double s, double d);

	double speed();

	Vehicle pos_at( double t );

};

#endif