# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Goals
1. Safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. 
2. The car should go as close as 50 MPH.
3. Max acceleration: 10 m/s^2.
4. Max jerk: 50 m/s^3.
5. Make smooth lane changes if possible.
6. Avoid collisions.
7. Make at least one loop.

### Source files Overview
1. `main.cpp`: main function.
2. `ego.h/cpp`: The ego car class, including the most primary navigation strategy code.
3. `road.h/cpp`: The highway class, maintaining other vehicles and some function like `getXY'`, `getFrenet`, `nearestVehicleFront`, `nearestVehicleRear`.
4. `spline.h`: The spline funtion for generating interpolation.
5. `util.hpp/cpp`: Utility variables and functions, like `LANE_WIDTH`, `SAFETY_DIST`, `pi()`, `deg2rad`.
6. `vehicle.h/cpp`: The vehicles class in the highway, simple.

### Stategy for generating paths

#### Outline
1. Ego have 3 states: KL (Keep Lane), LCL (Lane Change Left), LCR (Lane Change Right).
2. Default state: KL, as fast as possible, keep a `SAFETY_DIST`([util.hpp](src/util.hpp)) to front vehicle.
3. Ego will keep KL, until distance to front vehicle is less than SAFETY_DIST * 2.
4. Then ego will try to find out whether a good LC state exists.
5. If find one LCL or LCR, switch to LC state, and keep this state until finishing LC.
6. If not, keep KL.
7. NOTES: 
   * When the ego found a good LCL/LCR, I make sure that is safety([ego.cpp line 297](src/ego.cpp)).
   * So ego will execute lance change until it's done, no interruption.
   * But still, the ego will adjust its velocity based on front vehicle of current lane every single time.

#### Code Details
Almost all the code about strategy are included inside [ego.cpp](src/ego.cpp). I will walk through the whole code flow here:
1. The main function will call `Ego.generate_trajectory`[ego.cpp line 50](src/ego.cpp) every interval.
2. In `Ego.generate_trajectory`, it will check the ego state[ego.cpp line 67](src/ego.cpp).
   ```
   if ( st != "KL" ) { // LCL or LCR

    // Execute lane change.
    traj = _traj_LC( road );

    // End of lane change. set state back to KL.
    if ( this->s >= this->state.target_s ) {
      this->state.state = "KL";
      this->state.target_lane = this->lane;
    }

    } else { // KL
    ...
    }
   ```
3. If state is LCL or LCR, simply execute lane change until reach the end, and then switch back to KL.
4. If state is KL, there are a lot of work to do.
   * Find the front vehicle, check the distance to ego. [ego.cpp line 80](src/ego.cpp)
    ```
    Vehicle veh = road.nearestVehicleFront( this->s, this->d );
    double dist = veh.id == -1 ? 10000.0 : veh.s - this->s;
    ```
   * If distance further than `2 * SAFETY_DIST`([util.hpp](src/util.hpp), then stay KL.
   * If less, need to consider lane change. [ego.cpp line 88](src/ego.cpp)
      * Find nearest front vehicle in left and right lane, and calculate distance individually.[ego.cpp line 90 ~ 97](src/ego.cpp)
      * Try to get a safety LC state by calling `_state_LC` for left and right lane(if any). Then choose LCL or LCR which is better, and store distance into `dist_to`.
        ```
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
        ```
      * Check the candidate LC state to see whether it is better than KL or not. If distance of front vehicle of target lane is larger than 2 times of current lane, then switch.
        ```
        if ( state.state == "NONE" || dist_to < dist * 2 ) {
          // Not save to do lane change OR the other lane have a closer front vehicle.
          traj = _traj_KL( road );
        } else {
          this->state = state;
          traj = _traj_LC( road );
        }
        ```
      * Then, `_traj_KL` or `_traj_LC` will generate trajectory for that state.
5. `_state_LC` [ego.cpp line 297](src/ego.cpp): Trying to generate a safety lane change state, LCL or LCR.
   1. Find the nearest Front and Rear vehicle in the target lane at a future time, which is the end of the prev_path. 
   2. Calculate the s-distance between Front (or Rear) vehicle and ego at a further future time, which is the prev_end time + estimate of lane change time(which is also the time ego get "target_s").
   3. The safety LC state should satisfy:
    * #1). dist_front is more than SAFETY_DIST * 2.
    * #2). dist_front is more than SAFETY_DIST.
   4. Return a "NONE" state if failed to find a safety LC state.
6. `_traj_KL` and `_traj_LC` are doing the similar work:
   * Adjust `prev_acc ` and `prev_vel` by calling `__traj_control`.
   * Set target s for trajectory.
   * Return trajector by calling `__traj`.
7. `__traj_control` [ego.cpp line 381](src/ego.cpp): Adjust "prev_acc" and "prev_vel" based on the front vehicle.
   * `prev_acc ` and `prev_vel` are the acc and velocity of ego of next few points in trajectory.
   * Find the front vehicle in current lane.(Note: Current lane will change in lane change.)
   * If no vehicle at front, that's simple, just speed up.
   * If the vehicle exist:
      * If the distance is too close, that's very dangerous, should break down with max deacc.
        ```
        if ( veh.s - this->s <= MIN_DIST || delta_s_end <=  MIN_DIST ) {
          // Very Dangerous !!
          cout << "[ _traj_control ] Dangerous!! dist = " << veh.s - this->s
                << ", delta_s_end = " << delta_s_end << endl;
          prev_acc = - MAX_ACC;
          prev_vel = max( 1.0, prev_vel + prev_acc * 0.02 );
          if( prev_vel == 1.0 )
            prev_acc = 0.0;
        }
        ```
      * Then if ego's speed is larger than the front vehicle, which means we are approaching. Ego need to check the distance at prev path end time, if less than SAFETY_DIST, break down; otherwise, means far away, speed up.
        ```
        if ( delta_s_end < SAFETY_DIST ) { // need break down
          __traj_deacc_max( speed > 2.0 ? speed - 2.0 : speed / 2.0 );
          cout << "[ _traj_control ] <<<<< Break down: prev_acc = " << prev_acc 
                << ", prev_vel = " << prev_vel << endl;
        } else { // far away, keep acceleration or max speed
          __traj_acc_max();
        }
        ```
      * Then if ego's speed is less than the front vehicle, which means we are leaving. Ego still need to check this distance  at prev path end time, if less than SAFETY_DIST, break down to front vehicle's speed - 2.0; otherwise, means far away, speed up.
        ```
        else if ( delta_s_end < SAFETY_DIST ) {
          __traj_deacc_max( speed > 2.0 ? speed - 2.0 : speed / 2.0 );
        } else {
          __traj_acc_max();
        }
        ```
8. `__traj` [ego.cpp line 225](src/ego.cpp): Generate trajectory to target lane & target s, with prev path not included, simply using spline to generate.


### For improvement
I have test a lot of loops with this strategy, and that's safe in almost all the situations. But still have some disadvantages need to be improved:

1. During lane change, there maybe a collision if another car switch to the same lane close to ego. That's because ego have not check maybe some vehicle from the other lane switch to the target lane. However this situation is not so much for now...
2. There maybe a max jerk, while a vehicle suddenly switch to ego's lane and so close to ego. That makes ego have to make a great deacc. But safe is first, right?




