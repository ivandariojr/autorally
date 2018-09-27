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
#include <random>

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

  struct ObstacleModel{
    std::string name;
    ignition::math::Pose3d pose;
  };

  ros::Subscriber rosSub;
  ros::Publisher obstacle_name_pub_;

  ros::CallbackQueue rosQueue;
  std::thread rosQueueThread;
  std::unique_ptr<ros::NodeHandle> rosNode;
  physics::WorldPtr _parent;
  std::vector<ObstacleModel> added_models;
  std::mutex remove_mutex;
  bool appended = false;
  long int prev_num_obstacles = -1;
  std::default_random_engine generator_;

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
    for (auto &m_name: added_models) {
      m_name.pose = safe_obstacle_sample(q_rot);
      gzdbg << "[MOVING] obstacle " << m_name.name
      << " to poisiton " << m_name.pose << std::endl;
#ifdef GAZEBO_9
      _parent->ModelByName(m_name.name)->SetWorldPose(m_name.pose);
#else
      _parent->GetModel(m_name.name)->SetWorldPose(m_name.pose);
#endif
//      gazebo::common::Time::MSleep(100);
      gzdbg << "[MOVING] Sucessful" << std::endl;
    }
    return true;
  }

  void spawn_obstacles(const autorally_msgs::spawnObstaclesConstPtr &_msg) {
    _parent->SetPaused(true);
#ifdef GAZEBO_9
    _parent->SetPhysicsEnabled(false);
#endif

    //Change Seed
    //if (ignition::math::Rand::Seed() != _msg->seed || _msg->reseed) {
    if (_msg->reseed) {
      generator_.seed(_msg->seed);
      //ignition::math::Rand::Seed(_msg->seed);
      //gzdbg << "[PLUGIN] Changed Seed value from: " << ignition::math::Rand::Seed() << " to " <<
      //      ignition::math::Rand::Seed() << std::endl;
      gzdbg << "[PLUGIN] Changed Seed value to " << _msg->seed << std::endl;
    }

    auto num_spawn = static_cast<long unsigned int>(_msg->num_obstacles);
    bool default_locations = _msg->default_locations;
    std::vector<std::string> obstacle_types = _msg->obstacle_types;
    // Move Existing Objects if the number spawned is the same from last time

//    if (move_if_same_number(num_spawn, default_locations)) {
//      gzdbg << "[PLUGIN] Only Moving Obstacle" << std::endl;
//      return;
//    }
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
    for(const auto & model : added_models){
      gzdbg << "[PLUGIN] Removing " << model.name << std::endl;
      _parent->RemoveModel(model.name);
      gazebo::common::Time::MSleep(100);
    }
    while (!added_models.empty()) { added_models.pop_back(); }
    ignition::math::Quaterniond q_rot;
    auto expected_model_count = expected;
    long unsigned int models_added;
    if (!default_locations) {
      gzdbg << "[PLUGIN] Adding " << num_spawn << " new obstacles." << std::endl;
      q_rot.Axis(ignition::math::Vector3d(0, 0, 1), -M_PI_4);
      expected_model_count += num_spawn;
      models_added = num_spawn;
     } else {
      expected_model_count += default_poses.size();
      q_rot.Axis(ignition::math::Vector3d(1, 0, 0), 0);
      gzdbg << "[PLUGIN] Spawning Default Obstacles." << std::endl;
      models_added = default_poses.size();
    }
    wait_for_change(static_cast<unsigned int>(expected));
    for (unsigned int p_i = 0; p_i < models_added; ++p_i) {
      std::string added_type = default_locations ? default_obstacle_type : obstacle_types[p_i];
      gzdbg << added_type << ": ";
      safe_obstacle_insert(
        q_rot,
        obstacle_prefix + added_type + "_" + (appended_name) + std::to_string(p_i),
        added_type, default_locations ? p_i : -1);
      gazebo::common::Time::MSleep(100);
      gzdbg << added_models.back().name << " at " << added_models.back().pose <<  std::endl;
    }
    wait_for_change(static_cast<unsigned int>(expected_model_count));
    autorally_msgs::obstacleStatus status_msg;
    status_msg.header.stamp = ros::Time::now();
    status_msg.num_obstacles = num_spawn;
    status_msg.obstacle_names = std::vector<std::string>(added_models.size());
    for (int m_idx = 0; m_idx < added_models.size(); ++m_idx) {
      status_msg.obstacle_names[m_idx] = added_models[m_idx].name;
    }
    obstacle_name_pub_.publish(status_msg);
#ifdef GAZEBO_9
    _parent->SetPhysicsEnabled(true);
#endif
    _parent->SetPaused(false);
  }

  void safe_obstacle_insert(const ignition::math::Quaterniond &frame,
                            const std::string &obs_name,
                            const std::string &obs_type,
                            const int default_idx) {
    ignition::math::Pose3d pose_added;
    if(default_idx != -1){
      gzdbg << "Adding default obstacle with index: " << default_idx << std::endl;
      pose_added = default_poses[default_idx];
    }
    else{
      ignition::math::Pose3d returns = safe_obstacle_sample(frame);
      pose_added = returns;
    }
    added_models.push_back(ObstacleModel{
          obs_name, pose_added});
    _parent->InsertModelString(make_obstacle_sdf(
      added_models.back().pose,
      added_models.back().name, obs_type));
  }
    ignition::math::Pose3d safe_obstacle_sample(const ignition::math::Quaterniond &frame) {
    ignition::math::Pose3d returns;
//    gzdbg << "Sampling Random Pose" << std::endl;
    const double min_dist = 5;
    bool pose_is_safe;
    do{
        pose_is_safe = true;
        returns = sample_track().RotatePositionAboutOrigin(frame);
//        gzdbg << "Sampled the following pose: " << returns << std::endl;
        for(const auto & model : added_models){
          double distance = model.pose.Pos().Distance(returns.Pos());
//          gzdbg << "Distance to previously added obstacle " << model.pose.Pos() << "is : " << distance << std::endl;
          if(distance < min_dist){
//            gzdbg << "Distance Threshold Violated. Resampling." << std::endl;
            pose_is_safe = false;
            break;
          }
        }
      } while(!pose_is_safe);
    return returns;
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
      gazebo::common::Time::MSleep(100);
    }
  }

  ignition::math::Pose3d sample_rounds() {
    //actual min/max values commented. Added a margin to avoid obstacles too close to the edge
    //Sample a ring using polar coordiantes. Centered on the origin. Then convert to cartesian and add an offset on x.
    constexpr double min_r = 4.7;//= 4.5;
    constexpr double max_r = 7.5;//= 7.75;
    constexpr double min_angle = M_PI;
    constexpr double max_angle = -M_PI;
    double rolled_angle = std::uniform_real_distribution<double>(min_angle, max_angle)(generator_);
    //ignition::math::Rand::DblUniform(min_angle, max_angle);
    double rolled_r = std::uniform_real_distribution<double>(min_r, max_r)(generator_);
    //ignition::math::Rand::DblUniform(min_r, max_r);
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

    double rolled_x = std::uniform_real_distribution<double>(min_x, max_x)(generator_);
    //ignition::math::Rand::DblUniform(min_x, max_x);
    double rolled_y = std::uniform_real_distribution<double>(min_y, max_y)(generator_);
    //ignition::math::Rand::DblUniform(min_y, max_y);
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

    double area_roll = std::uniform_real_distribution<double>(0, total_area)(generator_);
    //ignition::math::Rand::DblUniform(0, total_area);
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