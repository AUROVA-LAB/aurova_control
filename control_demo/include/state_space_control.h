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
  int transformPoint (struct PoseCtrl transform, struct PoseCtrl original_point, struct PoseCtrl &transformed_point);

  /**
   * \brief restriction rules
   *
   * in this method we specify the differens rules for get restrictions in the control algorithm.
   *
   * @param pose_in_goal_frame (imput) is the vehicle pose in goal frame.
   */
  int restrictionRules (struct PoseCtrl pose_in_goal_frame);

public:

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
   *
   * @param pose_ (imput) the current pose of the vehicle.
   * @param goal_ (imput) the desired pose of the vehicle.
   */
  int checkRestrictions(struct PoseCtrl st_pose, struct PoseCtrl st_goal);
};

#endif
