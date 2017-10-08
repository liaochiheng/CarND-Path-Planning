#ifndef ROAD_H
#define ROAD_H
#include <vector>
#include <string>
#include "vehicle.h"

using namespace std;

class Road {

public:

	vector<double> wps_x;
	vector<double> wps_y;
	vector<double> wps_s;
	vector<double> wps_dx;
	vector<double> wps_dy;

	vector<Vehicle> vehicles;

	/**
	* Constructor
	*/
	Road(vector<double> wps_x, vector<double> wps_y, vector<double> wps_s,
		vector<double> wps_dx, vector<double> wps_dy);

	/**
	* Destructor
	*/
	virtual ~Road();

	// double pi();
	// double deg2rad(double x);
	// double rad2deg(double x);
	// double distance(double x1, double y1, double x2, double y2);
	
	int _closestWaypoint(double x, double y);
	int _nextWaypoint(double x, double y, double theta);

	vector<double> getFrenet(double x, double y, double theta);
	vector<double> getXY(double s, double d);

	void addVehicle(double id, double x, double y, double vx, double vy, double s, double d);

	Vehicle nearestVehicleFront( double s, double d, double t = 0.0 );
	Vehicle nearestVehicleRear( double s, double d, double t = 0.0 );

};

#endif