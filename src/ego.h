#ifndef EGO_H
#define EGO_H
#include <vector>
#include <string>
#include "util.hpp"
#include "road.h"
#include "spline.h"

using namespace std;
using namespace util;

class Ego {

public:

	double x;
	double y;
	double s;
	double d;
	double yaw;
	double v;

	int lane;

	struct State
	{
		string state = "NONE";
		double target_s;
		// double target_d;
		int target_lane;

		double dist_front;
	} state;

	vector<double> prev_path_x;
	vector<double> prev_path_y;
	double prev_end_s;
	double prev_end_d;

	double prev_vel;
	double prev_acc;
	// double prev_jerk = MAX_JERK;

	/**
	* Constructor
	*/
	Ego();

	/**
	* Destructor
	*/
	virtual ~Ego();

	void update( double x, double y, double s, double d, double yaw, double v );

	void update_prev( vector<double> prev_path_x, vector<double> prev_path_y, 
		double prev_end_s, double prev_end_d );

	vector< vector<double> > generate_trajectory( Road& road );

	tk::spline _spline( Road& road, double& ref_x, double& ref_y, double& ref_yaw, 
		double target_d, double target_s );

	void __traj_acc_max();
	void __traj_deacc_max( double target_v );
	vector< vector<double> > __traj( Road& road, int lane, double delta_s );

	void __traj_control( Road& road );

	vector< vector<double> > _traj_KL( Road& road );
	vector< vector<double> > _traj_LC( Road& road );

	Ego::State _state_LC( Road& road, int delta_lane );

};

#endif