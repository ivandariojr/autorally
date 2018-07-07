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
* @file obs_costs.cuh
* @author Jake Sacks <jsacks6@gatech.edu>
* @date July 6, 2018
* @copyright 2018 Georgia Institute of Technology
* @brief MPPIObsCosts class definition
***********************************************/
#ifndef MPPI_OBS_COSTS_CUH_
#define MPPI_OBS_COSTS_CUH_

#include "obs_costs.cuh"

namespace autorally_control {

/**
* @class MPPIObsCosts obs_costs.cuh
* @brief Maintains cost parameters for the model predictive path integral
* control algorithm which considers obstacles.
*
*/
class MPPIObsCosts: public MPPICosts
{
public:

  // Struct for holding obstacle-specific cost parameters.
  typedef struct
  {
    float static_obs_coeff;
  } ObsCostParams;

  ObsCostParams obs_params_; ///< Struct for obstacle cost parameters.

  /**
  * @brief Simple constructor for MPPIObsCost.
  * @param width The width (# elements across one row) of the costmap.
  * @param height The height (# elements across one column) of the costmap.
  */
  MPPIObsCosts(int width, int height);

  /**
  * @brief Constructor for when loading cost grid and transform from a file.
  * @param mppi_node Node handle to the controller ROS node.
  */
  MPPIObsCosts(ros::NodeHandle mppi_node);

  /**
  * @brief Free cuda memory.
  */
  ~MPPIObsCosts();

protected:
  //Primary variables
  ObsCostParams* obs_params_d;
};

}

#include "obs_costs.cu"

#endif /*MPPI_OBS_COST_CUH*/