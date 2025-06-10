#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/float64.hpp>

namespace gazebo
{
  class CustomBuoyancyPlugin : public ModelPlugin
  {
  public:
    CustomBuoyancyPlugin() : ModelPlugin()
    {
      this->fluidDensity = 1000.0; // Default fluid density (kg/m^3, water)
      this->verticalForceAdjustment = 0.0; // Default adjustment force
    }

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      this->model = _model;

      // Read fluid density from SDF if specified
      if (_sdf->HasElement("fluid_density"))
      {
        this->fluidDensity = _sdf->Get<double>("fluid_density");
      }

      // Parse link elements for buoyancy properties
      if (_sdf->HasElement("link"))
      {
        sdf::ElementPtr linkElem = _sdf->GetElement("link");
        while (linkElem)
        {
          // Get link name
          std::string linkName = linkElem->Get<std::string>("name");
          physics::LinkPtr link = this->model->GetLink(linkName);
          if (!link)
          {
            gzerr << "Link '" << linkName << "' not found in model\n";
            linkElem = linkElem->GetNextElement("link");
            continue;
          }

          // Get volume
          double volume = 0.0;
          if (linkElem->HasElement("volume"))
          {
            volume = linkElem->Get<double>("volume");
          }
          else
          {
            gzerr << "No volume specified for link '" << linkName << "'\n";
            linkElem = linkElem->GetNextElement("link");
            continue;
          }

          // Get center of volume
          ignition::math::Vector3d centerOfVolume(0, 0, 0);
          if (linkElem->HasElement("center_of_volume"))
          {
            centerOfVolume = linkElem->Get<ignition::math::Vector3d>("center_of_volume");
          }

          // Store buoyancy info
          BuoyancyInfo info;
          info.link = link;
          info.volume = volume;
          info.centerOfVolume = centerOfVolume;
          this->buoyancyInfo.push_back(info);

          // Identify the base link (assuming it's named "link_base")
          if (linkName == "link_base")
          {
            this->baseLink = link;
            this->baseCenterOfVolume = centerOfVolume;
          }

          linkElem = linkElem->GetNextElement("link");
        }
      }

      // Check if base link was found
      if (!this->baseLink)
      {
        gzerr << "Base link 'link_base' not found in model. Vertical force adjustment will not be applied.\n";
      }

      // Initialize ROS 2 node
      this->rosNode = gazebo_ros::Node::Get(_sdf);
      // Subscribe to the vertical_force topic
      this->sub = this->rosNode->create_subscription<std_msgs::msg::Float64>(
          "vertical_force", 10,
          std::bind(&CustomBuoyancyPlugin::OnVerticalForceMsg, this, std::placeholders::_1));

      // Connect to the update event
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&CustomBuoyancyPlugin::OnUpdate, this));
    }

    // Callback for vertical_force topic
    void OnVerticalForceMsg(const std_msgs::msg::Float64::SharedPtr msg)
    {
      this->verticalForceAdjustment = msg->data;
      gzmsg << "Received vertical force adjustment: " << this->verticalForceAdjustment << " N\n";
    }

    void OnUpdate()
    {
      // Gravity vector
      ignition::math::Vector3d gravity = this->model->GetWorld()->Gravity();

      // Apply buoyant force to all links
      for (const auto& info : this->buoyancyInfo)
      {
        physics::LinkPtr link = info.link;
        double volume = info.volume;
        ignition::math::Vector3d centerOfVolume = info.centerOfVolume;

        // Get the link's pose in the world frame
        ignition::math::Pose3d linkPose = link->WorldPose();
        ignition::math::Vector3d covWorld = linkPose.Rot().RotateVector(centerOfVolume) + linkPose.Pos();

        // Buoyant force = -ρ * V * g
        ignition::math::Vector3d buoyantForce = -this->fluidDensity * volume * gravity;

        // Apply only the buoyant force to this link
        link->AddForceAtWorldPosition(buoyantForce, covWorld);
      }

      // Apply vertical force adjustment only to the base link, if it exists
      if (this->baseLink)
      {
        // Get the base link's pose in the world frame
        ignition::math::Pose3d basePose = this->baseLink->WorldPose();
        ignition::math::Vector3d baseCovWorld = basePose.Rot().RotateVector(this->baseCenterOfVolume) + basePose.Pos();

        // Vertical force adjustment in the Z direction
        ignition::math::Vector3d verticalForce = ignition::math::Vector3d(0, 0, this->verticalForceAdjustment);

        // Apply the vertical force adjustment at the center of volume of the base link
        this->baseLink->AddForceAtWorldPosition(verticalForce, baseCovWorld);
      }
    }

  private:
    struct BuoyancyInfo
    {
      physics::LinkPtr link;
      double volume;
      ignition::math::Vector3d centerOfVolume;
    };

    physics::ModelPtr model;
    double fluidDensity;
    double verticalForceAdjustment;
    std::vector<BuoyancyInfo> buoyancyInfo;
    physics::LinkPtr baseLink; // Pointer to the base link
    ignition::math::Vector3d baseCenterOfVolume; // Center of volume for the base link
    event::ConnectionPtr updateConnection;
    std::shared_ptr<gazebo_ros::Node> rosNode;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(CustomBuoyancyPlugin)
}

// Command to send vertical force (run in a terminal after launching your ROS 2 and Gazebo environment):
// Publish a positive value (e.g., +2) to make the fish move up, or a negative value (e.g., -2) to make it sink:
//   ros2 topic pub /vertical_force std_msgs/msg/Float64 "data: 2.0"
//   ros2 topic pub /vertical_force std_msgs/msg/Float64 "data: -2.0"