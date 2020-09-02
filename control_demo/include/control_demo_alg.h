// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author 
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// 
// IMPORTANT NOTE: This code has been generated through a script from the 
// iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. Please
// refer to the IRI wiki page for more information:
// http://wikiri.upc.es/index.php/Robotics_Lab
/**
 * \file ackermann_to_odom_alg_node.h
 *
 *  Created on: 29 Oct 2018
 *      Author: m.a.munoz
 */

#ifndef _control_demo_alg_h_
#define _control_demo_alg_h_

#include <control_demo/ControlDemoConfig.h>
#include "state_space_control.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include <tf/tf.h>

//include control_demo_alg main library

/**
 * \brief IRI ROS Specific Driver Class
 *
 *
 */
class ControlDemoAlgorithm
{
protected:
  /**
   * \brief define config type
   *
   * Define a Config type with the ControlDemoConfig. All driver implementations
   * will then use the same variable type Config.
   */
  pthread_mutex_t access_;

  // private attributes and methods

public:

  bool flag_pose_active_;
  bool flag_goal_active_;
  StateSpaceControlPtr control_;

  /**
   * \brief define config type
   *
   * Define a Config type with the ControlDemoConfig. All driver implementations
   * will then use the same variable type Config.
   */
  typedef control_demo::ControlDemoConfig Config;

  /**
   * \brief config variable
   *
   * This variable has all the driver parameters defined in the cfg config file.
   * Is updated everytime function config_update() is called.
   */
  Config config_;

  /**
   * \brief constructor
   *
   * In this constructor parameters related to the specific driver can be
   * initalized. Those parameters can be also set in the openDriver() function.
   * Attributes from the main node driver class IriBaseDriver such as loop_rate,
   * may be also overload here.
   */
  ControlDemoAlgorithm(void);

  /**
   * \brief Lock Algorithm
   *
   * Locks access to the Algorithm class
   */
  void lock(void)
  {
    pthread_mutex_lock(&this->access_);
  }
  ;

  /**
   * \brief Unlock Algorithm
   *
   * Unlocks access to the Algorithm class
   */
  void unlock(void)
  {
    pthread_mutex_unlock(&this->access_);
  }
  ;

  /**
   * \brief Tries Access to Algorithm
   *
   * Tries access to Algorithm
   *
   * \return true if the lock was adquired, false otherwise
   */
  bool try_enter(void)
  {
    if (pthread_mutex_trylock(&this->access_) == 0)
      return true;
    else
      return false;
  }
  ;

  /**
   * \brief config update
   *
   * In this function the driver parameters must be updated with the input
   * config variable. Then the new configuration state will be stored in the
   * Config attribute.
   *
   * \param new_cfg the new driver configuration state
   *
   * \param level level in which the update is taken place
   */
  void config_update(Config& config, uint32_t level = 0);

  // here define all control_demo_alg interface methods to retrieve and set
  // the driver parameters

  /**
   * \brief Destructor
   *
   * This destructor is called when the object is about to be destroyed.
   */
  ~ControlDemoAlgorithm(void);

  /**
   * \brief Control Loop
   *
   * This method is the one that contains the procedure to generate
   * control actions given pose and goal. This function returns the status
   * encoded in an integer. The tags to interpret this number are in the class
   * StateSpaceControl and are: OK = 0, CROSSED_GOAL = 1, and BAD_ORIENTATION = 2.
   *
   * @param last_pose (imput) is the current pose of vehicle.
   * @param last_goal (imput) is the desired pose of vehicle.
   * @param d_vehicle (imput) is the length of the vehicle.
   * @param ackermann_state (output) is the control signal.
   * @param twist_state (output) is the control signal expressed in twist format.
   */
  int controlLoop(geometry_msgs::PoseWithCovarianceStamped last_pose, geometry_msgs::PoseStamped last_goal, float d_vehicle,
                  ackermann_msgs::AckermannDriveStamped& ackermann_state, geometry_msgs::Twist& twist_state);
};

#endif
