/*
* Software License Agreement (BSD License)
* Copyright (c) 2013, Georgia Institute of Technology
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**********************************************
 * @file run_control_loop.cuh
 * @author Grady Williams <gradyrw@gmail.com>
 * @date May 24, 2017
 * @copyright 2017 Georgia Institute of Technology
 * @brief Runs the control loop for a given feedback control law
 * parameter server.
 ***********************************************/

#ifndef RUN_CONTROLLER_CUH_
#define RUN_CONTROLLER_CUH_

#include "autorally_plant.h"
#include "param_getter.h"
#include <chrono>
#include <ros/ros.h>

namespace autorally_control {

template <class CONTROLLER_T> 
void runControlLoop(CONTROLLER_T controller, SystemParams params, ros::NodeHandle mppi_node)
{
  //Initial condition of the robot
  Eigen::MatrixXf state(7,1);
  state << params.x_pos, params.y_pos, params.heading, 0, 0, 0, 0;
  //Initial control value
  Eigen::MatrixXf u(2,1);
  u << 0, 0;
  //Robot is initially not active
  int robot_status = 1;
  int last_status = 1;

  AutorallyPlant robot(mppi_node, params.debug_mode, params.hz);
  AutorallyPlant::FullState fs;

  //std::vector<pcl::PointXY> points;
  sensor_msgs::PointCloud2Ptr points;

  autorally_msgs::pathIntegralStatus crash_msg;

  //Set the loop rate
  ros::Rate loop_rate(params.hz);

  //Counter and timing variables.
  int num_iter = 0;
  ros::Time last_pose_update = robot.getLastPoseTime();
  ros::Time last_pc_update = robot.getLastPointCloudTime();
  ros::Time last_track_pc_update = robot.getLastTrackPointCloudTime();
  ros::Time last_obs_update = robot.getLastObstacleResetTime();

  ros::Publisher path_pub; ///< Publisher of nav_mags::Path on topic nominalPath.
  ros::Publisher ips_pub; ///< Publisher of nav_mags::Path on topic importance sampler.
  ros::Publisher crash_pub; ///< Publisher of autorally_msgs:pathIntegralStats on topic crashStatus.

  path_pub = mppi_node.advertise<nav_msgs::Path>("nominal_path_debug", 1);
  ips_pub = mppi_node.advertise<nav_msgs::Path>("importance_sampler", 1);
  crash_pub = mppi_node.advertise<autorally_msgs::pathIntegralStatus>("crashStatus", 1);

  int crash = 0;

  //ros::AsyncSpinner spinner(8); ///< TODO: Make thread count a parameter
  //spinner.start();

  //Start the control loop.
  while (ros::ok()) {
    std::chrono::time_point<std::chrono::high_resolution_clock> t1l = std::chrono::high_resolution_clock::now();

    if (params.debug_mode){ //Display the debug window.
     controller.costs_->debugDisplay(state(0), state(1));
    }
    
    if (last_pose_update != robot.getLastPoseTime()){ //If we've received a new state estimate
      last_pose_update = robot.getLastPoseTime();
      fs = robot.getState(); //Get the new state.
      state << fs.x_pos, fs.y_pos, fs.yaw, fs.roll, fs.u_x, fs.u_y, fs.yaw_mder;
    }

    if (last_pc_update != robot.getLastPointCloudTime()){
      last_pc_update = robot.getLastPointCloudTime();
      points = robot.getPointCloud();
      controller.costs_->updateObstacleMap(points);
      //ROS_INFO("CREATE OBSTACLE MAP: %ld",duration);
    }

    if (last_track_pc_update != robot.getLastTrackPointCloudTime()){
      last_pc_update = robot.getLastTrackPointCloudTime();
      points = robot.getTrackPointCloud();
      controller.costs_->updateTrackMap(points);
      //ROS_INFO("CREATE OBSTACLE MAP: %ld",duration);
    }

    if (robot.getResetObstacles() && last_obs_update != robot.getLastObstacleResetTime())
    {
      controller.costs_->resetObstacleMap();
      last_obs_update = robot.getLastObstacleResetTime();
    }

    u = controller.computeControl(state, crash); //Compute the control
    controller.model_->enforceConstraints(state, u);
    controller.model_->updateState(state, u); //Update the state using motion model.
    
    robot.pubControl(u(0), u(1)); //Publish steering u(0) and throttle u(1)
    robot.pubPath(controller.nominal_traj_, path_pub, controller.num_timesteps_, params.hz); //Publish the planned path.
    robot.pubPath(controller.importance_sampler_, ips_pub, controller.num_timesteps_, params.hz); //Publish the planned path.

    crash_msg.status = crash;
    crash_msg.header.stamp = ros::Time::now();
    crash_pub.publish(crash_msg);

    //Check system status: 0 -> good, 1-> not active, 2-> bad
    if (!params.debug_mode){ //In simulation/debug mode everything is always ok.
      last_status = robot_status;
      robot_status = robot.checkStatus();
      if (robot_status == 2){
        controller.cutThrottle(); //Set desired speed and max throttle to zero.
      }
      if (last_status == 2 && robot_status != 2){
        controller.model_->control_rngs_[1].y = params.max_throttle; //Reset max throttle
        controller.model_->paramsToDevice();
        //Desired speed stays at zero, and needs to be reset manually from dynamic reconfigure.
      }
    }

    //Publish the controller status.
    robot.pubStatus();

    num_iter += 1;
    //Save debug info, do not change to ROS_INFO.
    ROS_DEBUG("Current State: (%f, %f, %f, %f, %f, %f, %f, %f, %f), Slip Angle: (%f), Avg. Iter Time: %f \n", 
              state(0), state(1), state(2), state(3), state(4), state(5), state(6), u(0), u(1),
              state(4) > 0.01 ? -atan(state(5)/fabs(state(4))) : 0,  controller.total_iter_time_/num_iter);

    std::chrono::time_point<std::chrono::high_resolution_clock> t2l = std::chrono::high_resolution_clock::now();
    auto duration_l = std::chrono::duration_cast<std::chrono::milliseconds>( t2l - t1l ).count();
    //ROS_INFO("TOTAL LOOP: %ld",duration_l);

    loop_rate.sleep(); //Sleep to get loop rate to match hz.
    t2l = std::chrono::high_resolution_clock::now();
    duration_l = std::chrono::duration_cast<std::chrono::milliseconds>( t2l - t1l ).count();
    //ROS_INFO("AFTER SLEEP: %ld",duration_l);

    ros::spinOnce(); //Process subscriber callbacks.
    t2l = std::chrono::high_resolution_clock::now();
    duration_l = std::chrono::duration_cast<std::chrono::milliseconds>( t2l - t1l ).count();
    //ROS_INFO("AFTER SPINNING: %ld",duration_l);
  }
}

}

#endif /*RUN_CONTROLLER_CUH_*/