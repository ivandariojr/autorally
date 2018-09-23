/**********************************************
 * @file obstacles_factory.cpp
 * @author Ivan Dario Jimenez <ijimenez3@gatech.edu>
 * @date Sept 12, 2018
 * @copyright 2018 Georgia Institute of Technology
 * @brief Randomly place obstacles on AutoRally Gazebo track.
 *
 ***********************************************/

#include <ignition/math/Pose3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <thread>
#include <memory>
#include <sstream>
#include <ros/callback_queue.h>
#include <std_msgs/Float64.h>
#include <autorally_msgs/spawnObstacles.h>
#include <autorally_msgs/obstacleStatus.h>

namespace gazebo {
class ObstacleFactory : public WorldPlugin {

  const std::string obstacle_prefix = "obstacle_";
  const std::string default_obstacle_type = "construction_cone";
  const std::vector<ignition::math::Pose3d> default_poses{
    ignition::math::Pose3d(1.8, -6.3, 0, 0, 0, 0),
    ignition::math::Pose3d(5.09, -4.72, 0, 0, 0, 0),
    ignition::math::Pose3d(3.33, 10.79, 0, 0, 0, 0),
    ignition::math::Pose3d(-4.52, -5.32, 0, 0, 0, 0),
    ignition::math::Pose3d(-6.41, 1.05, 0, 0, 0, 0),
    ignition::math::Pose3d(-10.29, -2.26, 0, 0, 0, 0)
  };

  ros::Subscriber rosSub;
  ros::Publisher obstacle_name_pub_;

  ros::CallbackQueue rosQueue;
  std::thread rosQueueThread;
  std::unique_ptr<ros::NodeHandle> rosNode;
  physics::WorldPtr _parent;
  std::vector<std::string> added_models;
  std::mutex remove_mutex;
  bool appended = false;
  long int prev_num_obstacles = -1;

public:
  static const std::string make_obstacle_sdf(const ignition::math::Pose3d &pose, const std::string &name,
                                             const std::string &type) {
    std::ostringstream stream;
    stream << "<sdf version='1.6'>"
           << "<model name=\"" << name << "\" >"
           << "<pose>" << pose << "</pose>"
           << "<static>true</static>"
           << "<include>"
           << "<name>" << name << "</name>"
           << "<uri>model://urdf/models/" << type << "</uri>"
           << "</include>"
           << "</model>"
           << "</sdf>";
    return stream.str();
  }

  void Load(physics::WorldPtr _parent, sdf::ElementPtr /* _sdf*/) override {
    this->_parent = _parent;
    bool random = false;
    int num_spawn = 500; //large to verify the distribution is correct.

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                       << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    gzdbg << "[PLUGIN] spawn_obstacles loaded correctly" << std::endl;
    rosNode = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle("obstacle_factory"));
    gzdbg << "[PLUGIN] rosnode created" << std::endl;
    ros::SubscribeOptions sub_options = ros::SubscribeOptions::create<autorally_msgs::spawnObstacles>(
    "/spawn_obstacles",
    1,
    boost::bind(&ObstacleFactory::spawn_obstacles, this, _1), ros::VoidPtr(), &this->rosQueue);
    gzdbg << "[PLUGIN] subs opts created" << std::endl;
    rosSub = rosNode->subscribe(sub_options);
    gzdbg << "[PLUGIN] subscribed" << std::endl;

    obstacle_name_pub_ = rosNode->advertise<autorally_msgs::obstacleStatus>("/obstacles_status", 1);

    rosQueueThread = std::thread{std::bind(&ObstacleFactory::queue_thread, this)};
  }

  bool move_if_same_number(long int num_spawn, bool default_locations) {
    if (prev_num_obstacles != num_spawn) {
      prev_num_obstacles = num_spawn;
      return false;
    }
    ignition::math::Quaterniond q_rot;
    q_rot.Axis(ignition::math::Vector3d(0, 0, 1), -M_PI_4);
    for (const auto &m_name: added_models) {
#ifdef GAZEBO_9
      _parent->ModelByName(m_name)->SetWorldPose(sample_track().RotatePositionAboutOrigin(q_rot));
#else
      _parent->GetModel(m_name)->SetWorldPose(sample_track().RotatePositionAboutOrigin(q_rot));
#endif
    }
    return true;
  }

  void spawn_obstacles(const autorally_msgs::spawnObstaclesConstPtr &_msg) {
    remove_mutex.lock();
    //Change Seed
    if (ignition::math::Rand::Seed() != _msg->seed || _msg->reseed) {
      gzdbg << "[PLUGIN] Changed Seed value from: " << ignition::math::Rand::Seed() << "to " << _msg->seed << std::endl;
      ignition::math::Rand::Seed(_msg->seed);
    }

    long int num_spawn = _msg->num_obstacles;
    bool default_locations = _msg->default_locations;
    std::vector<std::string> obstacle_types = _msg->obstacle_types;
    // Move Existing Objects if the number spawned is the same from last time
    /*
    if (move_if_same_number(num_spawn, default_locations)) {
      gzdbg << "[PLUGIN] Only Moving Obstacle" << std::endl;
      remove_mutex.unlock();
      return;
    } */
    // To Avoid removing and adding the same object prepend 0 to the number
    // if we have added obstacles before.
    std::string appended_name;
    if (!added_models.empty() and not appended) {
      appended_name = std::string("0");
      appended = true;
    } else {
      appended = false;
    }
    //Send remove commands and wait until actual number of models matches expected
    auto expected = WorldModelCount() - added_models.size();
    while (!added_models.empty()) {
      gzdbg << "[PLUGIN] Removing " << added_models.back() << std::endl;
      _parent->RemoveModel(added_models.back());
      added_models.pop_back();
    }
    wait_for_change(static_cast<unsigned int>(expected));

    if (!default_locations) {
      gzdbg << "[PLUGIN] Adding " << num_spawn << " new obstacles." << std::endl;
      ignition::math::Quaterniond q_rot;
      q_rot.Axis(ignition::math::Vector3d(0, 0, 1), -M_PI_4);
      auto expected_model_count = WorldModelCount() + num_spawn;
      for (int p_i = 0; p_i < num_spawn; ++p_i) {
        gzdbg << obstacle_types[p_i] << std::endl;
        std::string obs_name = obstacle_prefix + obstacle_types[p_i] + "_" + (appended_name) + std::to_string(p_i);
        _parent->InsertModelString(ObstacleFactory::make_obstacle_sdf(
          sample_track().RotatePositionAboutOrigin(q_rot), obs_name, obstacle_types[p_i]));
        added_models.push_back(obs_name);
      }
      wait_for_change(static_cast<unsigned int>(expected_model_count));
    } else {
      auto expected_model_count = WorldModelCount() + default_poses.size();
      gzdbg << "[PLUGIN] Spawning Default Obstacles." << std::endl;
      for (int p_i = 0; p_i < default_poses.size(); ++p_i) {
        std::string obs_name = obstacle_prefix + default_obstacle_type + "_" + (appended_name) + std::to_string(p_i);
        gzdbg << "[PLUGIN] Adding " << obs_name << std::endl;
        _parent->InsertModelString(ObstacleFactory::make_obstacle_sdf(default_poses[p_i], obs_name,
                                                                      default_obstacle_type));
        added_models.push_back(obs_name);
      }
      wait_for_change(static_cast<unsigned int>(expected_model_count));
    }

    autorally_msgs::obstacleStatus status_msg;
    status_msg.header.stamp = ros::Time::now();
    status_msg.num_obstacles = num_spawn;
    status_msg.obstacle_names = added_models;
    obstacle_name_pub_.publish(status_msg);

    remove_mutex.unlock();
  }

  unsigned int WorldModelCount() const {
#ifdef GAZEBO_9
    return _parent->ModelCount();
#else
    return _parent->GetModelCount();
#endif
  }

  ///
  /// \param expected the expected number of elements in the world after waiting
  void wait_for_change(unsigned int expected) {
    while (WorldModelCount() != expected) {
      gazebo::common::Time::MSleep(10);
    }
  }

  ignition::math::Pose3d sample_rounds() {
    //actual min/max values commented. Added a margin to avoid obstacles too close to the edge
    //Sample a ring using polar coordiantes. Centered on the origin. Then convert to cartesian and add an offset on x.
    constexpr double min_r = 4.7;//= 4.5;
    constexpr double max_r = 7.5;//= 7.75;
    constexpr double min_angle = M_PI;
    constexpr double max_angle = -M_PI;
    double rolled_angle = ignition::math::Rand::DblUniform(min_angle, max_angle);
    double rolled_r = ignition::math::Rand::DblUniform(min_r, max_r);
    double x_offset = 6 * (rolled_angle < M_PI_2 && rolled_angle > -M_PI_2 ? 1 : -1);
    return ignition::math::Pose3d(
    rolled_r * cos(rolled_angle) + x_offset, //x
    rolled_r * sin(rolled_angle), //y
    0, 0, 0, 0);//z, i, j, k

  }

  ignition::math::Pose3d sample_straights() {
    //actual min/max values commented. Added a margin to avoid obstacles too close to the edge
    //Sample a rectangle cententered on the origin then offset the points.
    constexpr double min_y = -3;//= -3.3;
    constexpr double max_y = 3;//= 3.3;
    constexpr double min_x = -6;
    constexpr double max_x = 6;

    double rolled_x = ignition::math::Rand::DblUniform(min_x, max_x);
    double rolled_y = ignition::math::Rand::DblUniform(min_y, max_y);
    double y_offset = 4.7 * (rolled_y < 0 ? -1 : 1);
    return ignition::math::Pose3d(
    rolled_x,
    rolled_y + y_offset,
    0, 0, 0, 0);

  }

  ignition::math::Pose3d sample_track() {
    constexpr double straight_area = 2 * 3.3 * 12;
    constexpr double rounds_area = (7.8 * 7.8 - 4.45 * 4.45) * M_PI;
    constexpr double total_area = straight_area + rounds_area;

    double area_roll = ignition::math::Rand::DblUniform(0, total_area);
    //Sample coordinates in the round and straight sections with probability proportional to their area.
    if (area_roll <= rounds_area) {
      // adding rounds cone
      return sample_rounds();
    } else {
      // adding straights cone
      return sample_straights();
    }
  }

private:

  void queue_thread() {
    constexpr double timeout = 0.01;
    ROS_INFO("Obstacle Thread Running");
    while (this->rosNode->ok()) {
      if (!this->rosQueue.isEmpty()) {
        ROS_INFO("Plugin saw something");
      }
      this->rosQueue.callAvailable(ros::WallDuration(timeout));
      ros::spinOnce();
    }
  }
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(ObstacleFactory)

}