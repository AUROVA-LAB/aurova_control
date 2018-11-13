/**
 * \file state_space_control.h
 *
 *  Created on: 29 Oct. 2018
 *      Author: m.a.munoz
 */

#ifndef _state_space_control_h_
#define _state_space_control_h_

#include <math.h>

#define OK 0
#define CROSSED_GOAL 1
#define BAD_ORIENTATION 2
#define PI 3.141592

struct PoseCtrl
{
  float x;
  float y;
  float z;
  double yaw;
  double pitch;
  double roll;
};

/**
 * \brief Class for control using state space model
 *
 * This class contains the necessary methods to generate a steering and speed for control
 * an ackermann vehicle
 *
 */
class StateSpaceControl;
typedef StateSpaceControl* StateSpaceControlPtr;

class StateSpaceControl
{
private:

  /**
   * \brief generic transform for a point
   *
   * this function rotate a point around his 0, and translate to a new point.
   *
   * @param transform (input) is the rotation and orientation for a transform.
   * @param original_point (input) is the original point.
   * @param transformed_point (output) is the transformed point.
   */
  int transformPoint(struct PoseCtrl transform, struct PoseCtrl original_point, struct PoseCtrl &transformed_point);

  /**
   * \brief restriction rules
   *
   * in this method we specify the differens rules for get restrictions in the control algorithm.
   * This function returns the status encoded in an integer. The tags to interpret this number are:
   * OK = 0, CROSSED_GOAL = 1, and BAD_ORIENTATION = 2.
   *
   * @param pose_in_goal_frame (imput) is the vehicle pose in goal frame.
   */
  int restrictionRules(struct PoseCtrl pose_in_goal_frame);

public:

  float ku_d_;
  float ku_a_;
  float kv_d_;
  float kv_a_;
  float v_base_;
  float v_max_;
  float time_out_wait_goal_;
  float error_d_sat_;
  float max_steering_;
  struct PoseCtrl st_goal_;
  struct PoseCtrl st_pose_;
  struct PoseCtrl st_pose_in_goal_frame_;

  /**
   * \brief constructor
   *
   * initialization of necessary parameters.
   */
  StateSpaceControl(void);

  /**
   * \brief Destructor
   *
   * This destructor is called when the object is about to be destroyed.
   */
  ~StateSpaceControl(void);

  /**
   * \brief check restrictions in state space control
   *
   * This method checks if the goal has been passed, or if it is oriented in the opposite direction.
   * This function returns the status encoded in an integer. The tags to interpret this number are:
   * OK = 0, CROSSED_GOAL = 1, and BAD_ORIENTATION = 2.
   *
   * @param this->pose_ (imput) the current pose of the vehicle.
   * @param this->goal_ (imput) the desired pose of the vehicle.
   */
  int checkRestrictions(void);

  /*
   * \brief calculation of error signals for steering and speed calculation
   *
   * The error signals are: 1) Error in distance, that are the perpendicular distance between the
   * vehicle and the line oriented with the goal. 2) Error in angle between vehicle and goal.
   *
   * @param this->st_pose_in_goal_frame_ (imput) the current pose of the vehicle in the goal frame.
   * @param error_d (output)
   * @param error_a (output)
   */
  int calculationErrorSignals(float& error_d, float& error_a);

  /*
   * \brief calculation of control signals; steering and speed.
   *
   * application of equations: u = k1*ed + k2*ea, v_pre = k3*ed + k4*ea, and
   * v = v_max - v_pre;
   *
   * @param steering (output)
   * @param speed (output)
   * @param error_d (imput)
   * @param error_a (imput)
   */
  int calculationControlSignals(float& steering, float& speed, float error_d, float error_a);
};

#endif
