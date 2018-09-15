/**********************************************
 * @file hello_world.cpp
 * @author Ivan Dario Jimenez <ijimenez3@gatech.edu>
 * @date Sept 12, 2018
 * @copyright 2018 Georgia Institute of Technology
 * @brief Test file for creating a Gazebo plugin.
 *
 ***********************************************/

#include <ignition/math/Pose3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

namespace gazebo {
class SpawnObstacles : public WorldPlugin {
  const std::string obstacle_prefix = "cone_obstacle_";
  const std::vector<ignition::math::Pose3d> default_poses{
    ignition::math::Pose3d(1.8, -6.3, 0, 0, 0, 0),
    ignition::math::Pose3d(5.09, -4.72, 0, 0, 0, 0),
//    ignition::math::Pose3d(6.53, -1.65, 0, 0, 0, 0),
    ignition::math::Pose3d(3.33, 10.79, 0, 0, 0, 0),
    ignition::math::Pose3d(-4.52, -5.32, 0, 0, 0, 0),
    ignition::math::Pose3d(-6.41, 1.05, 0, 0, 0, 0),
    ignition::math::Pose3d(-10.29, -2.26, 0, 0, 0, 0)
  };

 public:
  static const std::string make_obstacle_sdf(const ignition::math::Pose3d &pose, const std::string &name) {
    std::ostringstream stream;
    stream << "<sdf version='1.6'>"
           << "<model name=\"" << name << "\" >"
           << "<pose>" << pose << "</pose>"
           << "<static>true</static>"
           << "<include>"
           << "<name>" << name << "</name>"
           << "<uri>model://construction_cone</uri>"
           << "</include>"
           << "</model>"
           << "</sdf>";
    return stream.str();
  }
  void Load(physics::WorldPtr _parent, sdf::ElementPtr /* _sdf*/) override {
    bool random = false;
    int  num_spawn = 500; //large to verify the distribution is correct.
    gzdbg << "[PLUGIN] spawn_obstacles loaded correctly" << std::endl;
//    ignition::math::Rand::Seed(0);

    if (random) {
      gzdbg << "[PLUGIN] Adding "<< num_spawn << "new obstacles." << std::endl;
      ignition::math::Matrix3d rot;
      ignition::math::Quaterniond q_rot;
      rot.Axis(ignition::math::Vector3d(0, 0, 1), -M_PI_4);
      q_rot.Matrix(rot);
      for (int p_i = 0; p_i < num_spawn; ++p_i) {
        _parent->InsertModelString(SpawnObstacles::make_obstacle_sdf(
            sample_track().RotatePositionAboutOrigin(q_rot),
            obstacle_prefix + std::to_string(p_i)));
      }
    } else {
      gzdbg << "[PLUGIN] Spawning Default Obstacles." << std::endl;
      for (int p_i = 0; p_i < default_poses.size(); ++p_i) {
        _parent->InsertModelString(SpawnObstacles::make_obstacle_sdf(
            default_poses[p_i],
            obstacle_prefix + std::to_string(p_i)));
      }
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
    double x_offset = 6 * (rolled_angle < M_PI_2 &&  rolled_angle > -M_PI_2 ? 1 : -1);
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

};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(SpawnObstacles)

}
