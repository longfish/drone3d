#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float64MultiArray.h"

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/transport/Node.hh"

#include <geometry_msgs/Pose.h>
#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>
#include <ignition/math/Pose3.hh>

// Insert model from file

namespace gazebo
{
  class SurvivorsPlugin : public WorldPlugin
  {
  public:
    void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      gzmsg << "Loading Survivors plugin\n";
      this->gzNode = transport::NodePtr(new transport::Node()); // Create a new transport node
      this->gzNode->Init(_parent->Name());                      // Initialize the node with the world name

      // Create a topic name
      std::string topicName = "/" + _parent->Name() + "/pose";

      // Subscribe to the topic, and register a callback
      this->pose_sub = this->gzNode->Subscribe(topicName, &SurvivorsPlugin::OnMsg, this);

      /// ROS settings
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
      }

      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so =
          ros::SubscribeOptions::create<std_msgs::Float64MultiArray>(
              "/survivors/pose",
              1,
              boost::bind(&SurvivorsPlugin::OnRosMsg, this, _1),
              ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      // Spin up the queue helper thread.
      this->rosQueueThread =
          std::thread(std::bind(&SurvivorsPlugin::QueueThread, this));
    }

    /// \brief Handle an incoming message from ROS
    void OnRosMsg(const std_msgs::Float64MultiArray::ConstPtr &_msg)
    {
      this->PoseSurvivor(_msg->data.at(0), _msg->data.at(1));
    }

    /// \brief ROS helper function that processes messages
    void QueueThread()
    {
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration());
      }
    }

    /// \brief Publish a message about where to pose a survivor
    void PoseSurvivor(const float _x, const float _y)
    {
      survivors_pub = this->gzNode->Advertise<msgs::Factory>("~/factory");

      msgs::Factory survivors_msg;
      survivors_msg.set_sdf_filename("model://person_standing"); // load the person model

      // pose to initialize the model
      msgs::Set(survivors_msg.mutable_pose(),
                ignition::math::Pose3d(ignition::math::Vector3d(_x, _y, 0),
                                       ignition::math::Quaterniond(0, 0, 0)));

      survivors_pub->Publish(survivors_msg);
    }

  private:
    // Member variables for ROS
    std::unique_ptr<ros::NodeHandle> rosNode; /// \brief A node use for ROS transport
    ros::Subscriber rosSub;                   /// \brief A ROS subscriber
    ros::CallbackQueue rosQueue;              /// \brief A ROS callbackqueue that helps process messages
    std::thread rosQueueThread;               /// \brief A thread the keeps running the rosQueue

    transport::NodePtr gzNode;
    transport::PublisherPtr survivors_pub;
    transport::SubscriberPtr pose_sub;

    void OnMsg(ConstVector3dPtr &_msg)
    {
      this->PoseSurvivor(_msg->x(), _msg->y());
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(SurvivorsPlugin)
}