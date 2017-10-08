#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include "ego.h"
#include "spline.h"
#include "util.hpp"

using namespace std;
using namespace util;

Ego::Ego() {
	this->state.state = "KL"; // Keep Lane
  this->state.target_lane = 1;
}

Ego::~Ego() {}

void Ego::update(double x, double y, double s, double d, double yaw, double v) {
	this->x = x;
	this->y = y;
	this->s = s;
	this->d = d;
	this->yaw = yaw;
	this->v = v;

  this->lane = floor( d / LANE_WIDTH );
}

void Ego::update_prev( vector<double> prev_path_x, vector<double> prev_path_y,
    double prev_end_s, double prev_end_d ) {
	this->prev_path_x = prev_path_x;
	this->prev_path_y = prev_path_y;
  this->prev_end_s = prev_end_s;
  this->prev_end_d = prev_end_d;
}

/**
 * Called by main function, generate trajectory of current state.
 * 1. If state = LCL or LCR, just excute the lane change trajectory. The only thing
 *    need to worry about is the time to switch back to KL.
 * 2. If state = KL, will check the front vehicle, whether it is close enough.
 *    #1). If so, will try to find a safety LC state.
 *    #2). If the LC state is good enough, switch to it.
 *    #3). If the LC state is not good enough, stay KL.
 *    #4). Strategy of "good enough": Front vehicle of target lane is two times far
 *          away than current lane. It's very simple, since many checks have been done
 *          in function _state_LC.
 */
vector< vector<double> > Ego::generate_trajectory( Road& road ) {

  vector< vector<double> > traj;
  
  string st = this->state.state;

  cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
  cout << "[ generate_trajectory ] ========= lane = " << this->lane << ", state = " 
        << st << ", ego.state.target_s = " << this->state.target_s << endl;

  cout << "[ generate_trajectory ] >>>>>>>>>>>>>>> Ego Position:" << endl;
  cout << "[ generate_trajectory ] >>> ego.s = " << this->s << ", prev_end_s = " << this->prev_end_s << endl;
  cout << "[ generate_trajectory ] >>> ego.d = " << this->d << ", ego.v = " << this->v << endl;
  cout << "[ generate_trajectory ] >>> prev_size = " << this->prev_path_x.size() << endl;
  cout << "[ generate_trajectory ] >>> prev_acc = " << prev_acc << ", prev_vel = " << prev_vel << endl;
  cout << "[ generate_trajectory ] <<<<<<<<<<<<<<< Ego position end." << endl;

  if ( st != "KL" ) { // LCL or LCR

    // Execute lane change.
    traj = _traj_LC( road );

    // End of lane change. set state back to KL.
    if ( this->s >= this->state.target_s ) {
      this->state.state = "KL";
      this->state.target_lane = this->lane;
    }

  } else { // KL
      
    Vehicle veh = road.nearestVehicleFront( this->s, this->d );
    double dist = veh.id == -1 ? 10000.0 : veh.s - this->s;
    
    // Front vehicle is far away, just keep lane.
    if ( dist > SAFETY_DIST * 2 ) {

      traj = _traj_KL( road );

    } else { // Front vehicle is close, time to consider a lane change.
      
      double d_l = 2.0 + ( this->lane - 1 ) * LANE_WIDTH;
      double d_r = 2.0 + ( this->lane + 1 ) * LANE_WIDTH;

      Vehicle veh_l = road.nearestVehicleFront( this->s, d_l );
      Vehicle veh_r = road.nearestVehicleFront( this->s, d_r );

      double dist_l = veh_l.id == -1 ? 10000.0 : veh_l.s - this->s;
      double dist_r = veh_r.id == -1 ? 10000.0 : veh_r.s - this->s;

      Ego::State state;
      double dist_to;

      if ( this->lane == 0 ) {
        state = _state_LC( road, 1 );
        dist_to = dist_l;
      } else if ( this->lane == 2 ) {
        state = _state_LC( road, -1 );
        dist_to = dist_r;
      } else {
        state = dist_r > dist_l ? _state_LC( road, 1 ) : _state_LC( road, -1 );
        dist_to = max( dist_l, dist_r );
      }

      if ( state.state == "NONE" || dist_to < dist * 2 ) {
        // Not save to do lane change OR the other lane have a closer front vehicle.
        traj = _traj_KL( road );
      } else {
        this->state = state;
        traj = _traj_LC( road );
      }
      
    }
    
  }

  // Add prev path at the beginning.
  traj[0].insert( traj[0].begin(), prev_path_x.begin(), prev_path_x.end() );
  traj[1].insert( traj[1].begin(), prev_path_y.begin(), prev_path_y.end() );

  cout << "[ generate_trajectory ] =========================== " << endl;

  return traj;
}

/**
 * Return a spline curve to target_s and target_d.
 */
tk::spline Ego::_spline( Road& road, double& ref_x, double& ref_y, double& ref_yaw, 
      double target_d, double target_s ) {
  vector<double> ptsx;
  vector<double> ptsy;

  ref_x = x;
  ref_y = y;
  ref_yaw = yaw;

  int prev_size = prev_path_x.size();

  if (prev_size < 2) {
    double prev_car_x = x - cos(ref_yaw);
    double prev_car_y = y - sin(ref_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(y);
  } else {
    ref_x = prev_path_x[prev_size - 1];
    ref_y = prev_path_y[prev_size - 1];

    double ref_x_prev = prev_path_x[prev_size - 2];
    double ref_y_prev = prev_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  vector<double> next_wp0 = road.getXY( target_s, target_d );
  vector<double> next_wp1 = road.getXY( target_s + 30, target_d );
  vector<double> next_wp2 = road.getXY( target_s + 60, target_d );

  ptsx.push_back( next_wp0[0] );
  ptsx.push_back( next_wp1[0] );
  ptsx.push_back( next_wp2[0] );

  ptsy.push_back( next_wp0[1] );
  ptsy.push_back( next_wp1[1] );
  ptsy.push_back( next_wp2[1] );

  for (int i = 0; i < ptsx.size(); ++i) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
    ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
  }

  tk::spline spline;
  spline.set_points(ptsx, ptsy);

  return spline;
}

/**
 * Accelerate with max jerk until reaching max velocity.
 */
void Ego::__traj_acc_max() {
  if ( prev_vel == MAX_VEL ) {
    prev_acc = 0.0;
  } else {
    prev_acc = min( MAX_ACC, prev_acc + MAX_JERK * 0.02 );
    prev_vel = min( MAX_VEL, prev_vel + prev_acc * 0.02 );
  }
}

/**
 * Break down with max jerk until reaching target velocity.
 */
void Ego::__traj_deacc_max( double target_v ) {
  if ( prev_vel == 1.0 || prev_vel <= target_v ) {
    prev_acc = 0.0;
  } else {
    prev_acc = max( -MAX_ACC, prev_acc - MAX_JERK * 0.02 );
    prev_vel = max( 1.0, prev_vel + prev_acc * 0.02 );
  }
}

/**
 * Generate trajectory to target lane & target s, with prev path not included.
 */
vector< vector<double> > Ego::__traj( Road& road, int target_lane, double target_s ) {

  cout << "--[__traj] target_lane = " << target_lane << ", target_s = " << target_s << endl;

  vector<double> traj_x;
  vector<double> traj_y;

  double ref_x = x;
  double ref_y = y;
  double ref_yaw = yaw;
  
  double target_d = 2.0 + target_lane * LANE_WIDTH;
  if ( this->state.state == "LCL" )
    target_d += 1.0;
  if ( this->state.state == "LCR" )
    target_d -= 1.0;
  tk::spline spline = _spline( road, ref_x, ref_y, ref_yaw, 
                            target_d, target_s );

  double target_x = 30.0;
  double target_y = spline(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);

  int prev_size = this->prev_path_x.size();
  double N = target_dist / (prev_vel * 0.02);

  for (int i = 1; i <= 50 - prev_size; i ++) {
    double x_point = i * target_x / N;
    double y_point = spline(x_point);

    double x_ref = x_point;
    double y_ref = y_point;

    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    traj_x.push_back(x_point);
    traj_y.push_back(y_point);
  }

  return { traj_x, traj_y };
}

/**
 * Generate trajectory for KL state.
 */
vector< vector<double> > Ego::_traj_KL( Road& road ) {

  // Adjust "prev_acc" and "prev_vel".
  __traj_control( road );

  int prev_size = prev_path_x.size();
  double target_s = ( prev_size > 2 ? this->prev_end_s : this->s ) + 30.0;

  return __traj( road, this->lane, target_s );
}

/**
 * Trying to generate a safety lane change state, LCL or LCR.
 * 1. Find the nearest Front and Rear vehicle in the target lane at a future time, which
 *    is the end of the prev_path. 
 * 2. Calculate the s-distance between Front (or Rear) vehicle and ego at a further future
 *    time, which is the prev_end time + estimate of lane change time(which is also the time
 *    ego get "target_s").
 * 3. The safety LC state should satisfy:
 *    #1). dist_front is more than SAFETY_DIST * 2.
 *    #2). dist_front is more than SAFETY_DIST.
 * 4. Return a "NONE" state if failed to find a safety LC state.
 */
Ego::State Ego::_state_LC( Road& road, int delta_lane ) {

  cout << "[ _state_LC ] ************* Lane Change trying: delta_lane = " << delta_lane << endl;

  Ego::State state;

  double target_lane = this->lane + delta_lane;

  if ( target_lane < 0 || target_lane > 2 ) {
    cout << "[ _state_LC ] ************* Invalid lane: " << target_lane << endl;
    return state;
  }

  double target_d = 2.0 + target_lane * LANE_WIDTH;
  double target_s = this->prev_end_s + LC_DIST;

  int prev_size = this->prev_path_x.size();

  Vehicle veh_front = road.nearestVehicleFront( this->prev_end_s, target_d, prev_size * 0.02 );
  Vehicle veh_rear = road.nearestVehicleRear( this->prev_end_s, target_d, prev_size * 0.02 );

  double dd = this->d - target_d;
  double t = sqrt( LC_DIST * LC_DIST + dd * dd ) / prev_vel;

  double dist_front = veh_front.id != -1 ? 
                veh_front.s + veh_front.speed() * t - target_s : 10000.0;
  double dist_rear = veh_rear.id != -1 ? 
                target_s - ( veh_rear.s + veh_rear.speed() * t ) : 10000.0;

  if ( dist_front <= SAFETY_DIST * 2 || dist_rear <= SAFETY_DIST ) {
    cout << "[ _state_LC ] ************* Not Safe! dist_front = " << dist_front 
          << ", dist_rear = " << dist_rear << endl;
    return state;
  }

  state.state = delta_lane == -1 ? "LCL" : "LCR";
  state.target_lane = target_lane;
  state.target_s = target_s;
  state.dist_front = dist_front;

  cout << "[ _state_LC ] LC available! state = " << state.state << endl;

  cout << "[ _state_LC ] target_lane = " << target_lane
        << ", target_s = " << target_s << endl;

  cout << "[ _state_LC ] dist_front = " << dist_front
        << ", dist_rear = " << dist_rear << endl;

  cout << "[ _state_LC ] ************* Lane Change end <<<<<<" << endl;

  return state;

}

/**
 * Generate trajectory for LCL or LCR state.
 * 1. "target_s" is the s in the target lane, it should remain the same value throughout
 *    the entire lane change movement.
 * 2. When "prev_end_s" is close enough to "target_s", which means we already have the 
 *    whole trajectory of the lane change, we shoule set "target_s" to a bigger value to
 *    avoid exceptions in "spline"(when the points are not in ascending order).
 */
vector< vector<double> > Ego::_traj_LC( Road& road ) {

  // Adjust "prev_acc" and "prev_vel".
  __traj_control( road );

  cout << "[ _traj_LC ] prev_acc = " << prev_acc << ", prev_vel = " << prev_vel << endl;

  double target_s = this->state.target_s;
  double end_s = this->prev_end_s;

  // The road is cyclic, when prev_end_s is small, probably it's on next cycle.
  if ( ( target_s > 6000 && end_s < 100 ) || end_s >= target_s - 1.0 )
    target_s = end_s + 30.0;

  return __traj( road, this->state.target_lane, target_s );
}

/**
 * @param road: Road
 * Adjust "prev_acc" and "prev_vel" based on the front vehicle.
 * Then function "__traj" will use these two parameters to move ego.
 */
void Ego::__traj_control( Road& road ) {

  Vehicle veh = road.nearestVehicleFront( this->s, this->d );

  if ( veh.id == -1 )
    cout << "[ _traj_control ] Front vehicle: No Vehicle!"<< endl;
  else
    cout << "[ _traj_control ] Front vehicle: id = " << veh.id << ", dist = " << veh.s - this->s 
          << ", speed = " << veh.speed() << endl;

  int prev_size = prev_path_x.size();

  if ( veh.id == -1 ) { // No vehicles at front or far away

    __traj_acc_max();

  } else { // There is a vehicle at front

    double speed = veh.speed();
    double delta_s_end = veh.s + prev_size * speed * 0.02 - this->prev_end_s;
    double delta_s = delta_s_end > SAFETY_DIST ? delta_s_end - SAFETY_DIST : delta_s_end - MIN_DIST;

    cout << "[ _traj_control ] ###############" << endl;
    cout << "[ _traj_control ] ego.s = " << this->s << ", prev_end_s = " << this->prev_end_s << endl;
    cout << "[ _traj_control ] prev_size = " << prev_size << endl;
    cout << "[ _traj_control ] veh.s = " << veh.s << ", veh.speed = " << speed << endl;
    cout << "[ _traj_control ] delta_s_end = " << delta_s_end << ", delta_s = " << delta_s << endl;
    cout << "[ _traj_control ] ###############" << endl;


    if ( veh.s - this->s <= MIN_DIST || delta_s_end <=  MIN_DIST ) {
      // Very Dangerous !!
      cout << "[ _traj_control ] Dangerous!! dist = " << veh.s - this->s
            << ", delta_s_end = " << delta_s_end << endl;
      prev_acc = - MAX_ACC;
      prev_vel = max( 1.0, prev_vel + prev_acc * 0.02 );
      if( prev_vel == 1.0 )
        prev_acc = 0.0;
    } else if ( prev_vel > speed ) { // approaching front vehicle

      double delta_acc = 0.0;
      double delta_v = prev_vel - speed;
      double delta_t = 2 * delta_s / delta_v;

      if ( delta_s != 0 )
        delta_acc = -0.5 * delta_v * delta_v / delta_s;

      cout << "[ _traj_control ] ############# approaching..." << endl;
      cout << "[ _traj_control ] delta_v = " << delta_v << endl;
      cout << "[ _traj_control ] delta_t = " << delta_t << endl;
      cout << "[ _traj_control ] delta_s = " << delta_s << endl;

      cout << "[ _traj_control ] delta_acc = " << delta_acc << endl;
      cout << "[ _traj_control ] ############# approaching end" << endl;

      if ( delta_s_end < SAFETY_DIST ) { // need break down
        __traj_deacc_max( speed > 2.0 ? speed - 2.0 : speed / 2.0 );
        cout << "[ _traj_control ] <<<<< Break down: prev_acc = " << prev_acc 
              << ", prev_vel = " << prev_vel << endl;
      } else { // far away, keep acceleration or max speed
        __traj_acc_max();
      }

    } else if ( delta_s_end < SAFETY_DIST ) {
      __traj_deacc_max( speed > 2.0 ? speed - 2.0 : speed / 2.0 );
    } else {
      __traj_acc_max();
    }

    cout << "[ _traj_control ] With Front vehicle: prev_acc = " << prev_acc 
          << ", prev_vel = " << prev_vel << endl;
  }
}