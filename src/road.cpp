#include <iostream>
#include <math.h>
#include <vector>
#include "road.h"
#include "vehicle.h"
#include "util.hpp"

using namespace std;
using namespace util;

Road::Road(vector<double> wps_x, vector<double> wps_y, vector<double> wps_s,
		vector<double> wps_dx, vector<double> wps_dy) {
	this->wps_x = wps_x;
	this->wps_y = wps_y;
	this->wps_s = wps_s;
	this->wps_dx = wps_dx;
	this->wps_dy = wps_dy;
}

Road::~Road() {}

int Road::_closestWaypoint(double x, double y) {

	double closestLen = 100000; //large number
	int wp = 0;

	for(int i = 0; i < wps_x.size(); i++) {
		double map_x = wps_x[i];
		double map_y = wps_y[i];
		double dist = distance(x, y, map_x, map_y);
		if(dist < closestLen) {
			closestLen = dist;
			wp = i;
		}
	}

	return wp;
}

int Road::_nextWaypoint(double x, double y, double theta) {

	int wp = _closestWaypoint(x, y);

	double map_x = wps_x[wp];
	double map_y = wps_y[wp];

	double heading = atan2(map_y - y, map_x - x);

	double angle = fabs(theta - heading);

	if(angle > pi() / 4)
		wp ++;

	return wp;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> Road::getFrenet(double x, double y, double theta) {
	int next_wp = _nextWaypoint(x,y, theta);

	int prev_wp;
	prev_wp = next_wp - 1;

	if(next_wp == 0)
		prev_wp  = wps_x.size() - 1;

	double n_x = wps_x[next_wp] - wps_x[prev_wp];
	double n_y = wps_y[next_wp] - wps_y[prev_wp];
	double x_x = x - wps_x[prev_wp];
	double x_y = y - wps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
	double proj_x = proj_norm * n_x;
	double proj_y = proj_norm * n_y;

	double frenet_d = distance(x_x, x_y, proj_x, proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-wps_x[prev_wp];
	double center_y = 2000-wps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
		frenet_d *= -1;

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
		frenet_s += distance( wps_x[i], wps_y[i], wps_x[i + 1], wps_y[i + 1] );

	frenet_s += distance(0, 0, proj_x, proj_y);

	return {frenet_s, frenet_d};

}

vector<double> Road::getXY(double s, double d) {
	int prev_wp = -1;

	while( s > wps_s[prev_wp + 1] && prev_wp < (int)( wps_s.size() - 1 ) )
		prev_wp++;

	int wp2 = (prev_wp + 1) % wps_x.size();

	double heading = atan2( wps_y[wp2] - wps_y[prev_wp],
							wps_x[wp2] - wps_x[prev_wp] );

	// the x,y,s along the segment
	double seg_s = s - wps_s[prev_wp];

	double seg_x = wps_x[prev_wp] + seg_s * cos(heading);
	double seg_y = wps_y[prev_wp] + seg_s * sin(heading);

	double perp_heading = heading - pi() / 2;

	double x = seg_x + d * cos(perp_heading);
	double y = seg_y + d * sin(perp_heading);

	return {x, y};
}

void Road::addVehicle(double id, double x, double y, double vx, double vy, double s, double d) {
	bool find = false;
	for ( int i = 0; i < vehicles.size(); i ++ ) {
		if ( vehicles[i].id == id ) {
			vehicles[i].update(x, y, vx, vy, s, d);
			find = true;
			break;
		}
	}
	if ( !find ) {
		Vehicle v( id );
		v.update(x, y, vx, vy, s, d);
		vehicles.push_back( v );
	}
}

Vehicle Road::nearestVehicleFront( double s, double d, double t ) {
	int lane = floor( d / LANE_WIDTH );
	Vehicle target( -1 );

	if ( lane < 0 || lane > 2 )
		return target;
	
	double dist = 10000.0;

	// cout << "===== Searching nearestVehicleFront..." << vehicles.size() << endl;
	for ( int i = 0; i < vehicles.size(); i ++ ) {
		Vehicle veh = vehicles[i].pos_at( t );

		// cout << "(" << veh.id << ": " << veh.s << ", " << veh.d << "), ";
		if ( veh.d >= lane * LANE_WIDTH && veh.d < ( lane + 1 ) * LANE_WIDTH ) {
			if ( veh.s > s && veh.s - s < dist ) {
				dist = veh.s - s;
				target = veh;
			}
		}
	}
	// cout << endl << "===== Searching nearestVehicleFront Done!" << endl;

	return target;
}

Vehicle Road::nearestVehicleRear( double s, double d, double t ) {
	int lane = floor( d / LANE_WIDTH );
	Vehicle target( -1 );

	if ( lane < 0 || lane > 2 )
		return target;
	
	double dist = 10000.0;

	// cout << "===== Searching nearestVehicleRear..." << vehicles.size() << endl;
	for ( int i = 0; i < vehicles.size(); i ++ ) {
		Vehicle veh = vehicles[i].pos_at( t );
		// cout << "(" << veh.id << ": " << veh.s << ", " << veh.d << "), ";
		if ( veh.d >= lane * LANE_WIDTH && veh.d < ( lane + 1 ) * LANE_WIDTH ) {
			if ( veh.s < s && s - veh.s < dist ) {
				dist = s - veh.s;
				target = veh;
			}
		}
	}
	// cout << endl << "===== Searching nearestVehicleRear Done!" << endl;

	return target;
}