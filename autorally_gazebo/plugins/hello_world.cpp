/**********************************************
 * @file hello_world.cpp
 * @author Jake Sacks <jsacks6@gatech.edu>
 * @date July 18, 2018
 * @copyright 2018 Georgia Institute of Technology
 * @brief Test file for creating a Gazebo plugin.
 *
 ***********************************************/

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

namespace gazebo
{
  class GetBoundingBox : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override
    {
      // make sure the ROS node for Gazebo has already been initialized
      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the "
                         << "gazebo_ross package");
        return;
      }

      // store pointer to model
      this->model = _parent;

      // list to update event
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&GetBoundingBox::OnUpdate, this)
      );

      // create the publisher
      this->box_pub = this->node.advertise<std_msgs::String>("obstacle_box", 10);
    }

    void OnUpdate()
    {
#ifdef GAZEBO_9
      ignition::math::Box bounding_box = this->model->BoundingBox();
      ignition::math::Vector3d box_size = bounding_box.Size();
#else
      gazebo::math::Box bounding_box = this->model->GetBoundingBox();
      ignition::math::Vector3d box_size = bounding_box.GetSize().Ign();
#endif


      std_msgs::String msg;
      std::stringstream msg_stream;
      msg_stream << "Box size ("<<box_size.X()<<", "<<box_size.Y()<<", "<<box_size.Z()<<")";
      msg.data = msg_stream.str();
      ROS_INFO("%s", msg.data.c_str());

      box_pub.publish(msg);
    }

  private:
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
    ros::NodeHandle node;
    ros::Publisher box_pub;
  };
  GZ_REGISTER_MODEL_PLUGIN(GetBoundingBox)
}
