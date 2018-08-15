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
class MPPIObstacleCosts: public MPPICosts
{
public:

  // Struct for holding obstacle-specific cost parameters.
  typedef struct
  {
    float static_obs_coeff;
    float3 r_c1;
    float3 r_c2;
    float3 trs;
  } ObstacleParams;

  ObstacleParams obs_params_; ///< Struct for obstacle cost parameters.

  /**
  * @brief Simple constructor for MPPIObsCost.
  * @param width The width (# elements across one row) of the costmap.
  * @param height The height (# elements across one column) of the costmap.
  */
  MPPIObstacleCosts(int width, int height);

  /**
  * @brief Constructor for when loading cost grid and transform from a file.
  * @param mppi_node Node handle to the controller ROS node.
  */
  MPPIObstacleCosts(ros::NodeHandle mppi_node);

  /**
  * @brief Free cuda memory.
  */
  ~MPPIObstacleCosts();

  /**
  * @breif Allocate memory to CUDA array which is bound to a texture.
  */
  void allocateObstacleTexMem();

  /**
  * @brief Take a pointer to CPU memory for the obstacle map and binds it to a CUDA texture.
  * @param obs_costs Pointer to an array of floats containing the obstacle ocsts of size width*height.
  */
  void obsToTexture(float* obs_costs);

  /**
  * @brief Updates the current obstacle coordinate transform.
  * @param h Matrix representing a transform from world to (offset) obstacle map coordinates.
  * @param trs Array representing the offset.
  */
  void updateObstacleTransform(Eigen::MatrixXf h, Eigen::ArrayXf trs);

  /**
  * @brief Copy the obs_params_ struct to the GPU.
  */
  void obstacleParamsToDevice();

  /**
  * @brief Updates the obstacle parameters by reading them from the rosparam server.
  * @param mppi_node Node handle to the controller ROS node.
  */
  void updateObstacleParams(ros::NodeHandle mppi_node);

  /**
  * @brief Callback for the dynamic reconfigure of parameters which includes obstacles.
  * @param config struct which contains parameters received from the rosparam server
  * @param lvl level value of the parameters
  */
  void updateParams_dcfg(autorally_control::PathIntegralParamsConfig &config, int lvl);

  /**
  * @brief Loads obstacle data from a file
  * @param obstacle_path C-string representing the path to the obstacle map data file
  * @param R Matrix representing a transform from world to (offset) obstacle map coordinates
  * @param trs Array representing the offset
  */
  std::vector<float> loadObstacleData(const char* obstacle_path, Eigen::Matrix3f &R, Eigen::Array3f &trs);

  /**
  * @brief Free CUDA variables and memory.
  */
  void freeCudaMem();

  /**
  * @brief Compute a coordinate transform going from world to obstacle coordinates.
  */
  __host__ __device__ void obsCoorTransform(float x, float y, float* u, float* v, float* w);

  /*
  * @brief Compute the current obstacle cost based on obstacle map.
  */
  float getObstacleCost(float* s);

  /*
  * @brief Compute all the individual cost terms and add them together.
  */
  __device__ float computeCost(float* s, float* u, float* du, float* vars, int* crash, int timestep);

protected:
  //Primary variables
  int obs_width_, obs_height_; /// Width and height of obstacle map
  ObstacleParams* obs_params_d_; /// Device-side copy of obs_params_
  cudaArray *obs_costs_d_; /// CUDA array for texture binding
  cudaChannelFormatDesc obs_channel_desc_; /// CUDA texture channel description
  cudaTextureObject_t obs_costs_tex_; /// CUDA texture object
};

}

#include "obs_costs.cu"

#endif /*MPPI_OBS_COST_CUH*/