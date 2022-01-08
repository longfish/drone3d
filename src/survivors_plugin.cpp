#include "ignition/math/Pose3.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

// Insert model from file via message passing.

namespace gazebo
{
  class Flag : public WorldPlugin
  {
  public:
    void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      transport::NodePtr node(new transport::Node()); // Create a new transport node
      node->Init(_parent->Name());                    // Initialize the node with the world name

      transport::PublisherPtr person_pub =
          node->Advertise<msgs::Factory>("~/factory");

      msgs::Factory msg;
      msg.set_sdf_filename("model://person_standing"); // load the person model

      // Pose to initialize the model to
      msgs::Set(msg.mutable_pose(),
                ignition::math::Pose3d(
                    ignition::math::Vector3d(1, -2, 0),
                    ignition::math::Quaterniond(0, 0, 0)));

      person_pub->Publish(msg);
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(Flag)
}