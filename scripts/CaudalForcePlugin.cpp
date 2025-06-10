     #include <functional>
     #include <gazebo/common/common.hh>
     #include <gazebo/physics/physics.hh>
     #include <sdf/sdf.hh>
    #include <ignition/math/Pose3.hh>
    #include <ignition/math/Vector3.hh>
     #include <gazebo_ros/node.hpp>
     #include <std_msgs/msg/string.hpp>

     namespace gazebo
     {
       class CustomPitchControlPlugin : public ModelPlugin
       {
       public:
         CustomPitchControlPlugin() : ModelPlugin()
         {
           this->targetPitch = 0.0; // Default target pitch (radians)
           this->maxAngularVel = 0.5; // Max angular velocity (rad/s) for slow rotation
           this->pitchGain = 10.0; // Proportional gain for torque control
           this->dampingCoeff = 0.5; // Damping coefficient to reduce oscillation
         }

         void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
         {
           this->model = _model;

           // Get the base link (assuming it's named "link_base" as per your URDF)
           this->baseLink = this->model->GetLink("link_base");
           if (!this->baseLink)
           {
             gzerr << "Base link 'link_base' not found in model. Plugin will not function.\n";
             return;
           }

           // Read parameters from SDF if specified
           if (_sdf->HasElement("max_angular_velocity"))
           {
             this->maxAngularVel = _sdf->Get<double>("max_angular_velocity");
           }
           if (_sdf->HasElement("pitch_gain"))
           {
             this->pitchGain = _sdf->Get<double>("pitch_gain");
           }
           if (_sdf->HasElement("damping_coeff"))
           {
             this->dampingCoeff = _sdf->Get<double>("damping_coeff");
           }

           // Initialize ROS 2 node
           this->rosNode = gazebo_ros::Node::Get(_sdf);
           // Subscribe to the pitch_command topic
           this->sub = this->rosNode->create_subscription<std_msgs::msg::String>(
               "pitch_command", 10,
               std::bind(&CustomPitchControlPlugin::OnPitchCommandMsg, this, std::placeholders::_1));

           // Connect to the update event
           this->updateConnection = event::Events::ConnectWorldUpdateBegin(
               std::bind(&CustomPitchControlPlugin::OnUpdate, this));

           gzmsg << "CustomPitchControlPlugin loaded. Commands: 'up' for +90 deg pitch, 'down' for -90 deg pitch.\n";
         }

         // Callback for pitch_command topic
         void OnPitchCommandMsg(const std_msgs::msg::String::SharedPtr msg)
         {
           std::string command = msg->data;
           if (command == "up")
           {
             this->targetPitch = M_PI / 2.0; // +90 degrees in radians
             gzmsg << "Received pitch command: 'up' -> target pitch = +90 degrees\n";
           }
           else if (command == "down")
           {
             this->targetPitch = -M_PI / 2.0; // -90 degrees in radians
             gzmsg << "Received pitch command: 'down' -> target pitch = -90 degrees\n";
           }
           else
           {
             gzerr << "Invalid pitch command: '" << command << "'. Use 'up' or 'down'.\n";
           }
         }

         void OnUpdate()
         {
           if (!this->baseLink)
           {
             return;
           }

           // Get the current pose of the base link in the world frame
           ignition::math::Pose3d pose = this->baseLink->WorldPose();
           // Extract current pitch (rotation around Y-axis in world frame)
           ignition::math::Vector3d rpy = pose.Rot().Euler();
           double currentPitch = rpy.Y(); // Pitch in radians

           // Calculate pitch error
           double pitchError = this->targetPitch - currentPitch;

           // If error is small (within 0.01 radians ~ 0.57 degrees), hold position and stop
           if (std::abs(pitchError) < 0.01)
           {
             // Apply damping to prevent oscillation
             ignition::math::Vector3d angularVel = this->baseLink->WorldAngularVel();
             ignition::math::Vector3d dampingTorque = -this->dampingCoeff * angularVel;
             this->baseLink->AddTorque(dampingTorque);
             return;
           }

           // Calculate desired angular velocity (proportional control, capped for slow movement)
           double desiredAngularVel = this->pitchGain * pitchError;
           desiredAngularVel = std::max(std::min(desiredAngularVel, this->maxAngularVel), -this->maxAngularVel);

           // Get current angular velocity
           ignition::math::Vector3d currentAngularVel = this->baseLink->WorldAngularVel();

           // Calculate torque to achieve desired angular velocity (simple P control + damping)
           double torque = this->pitchGain * (desiredAngularVel - currentAngularVel.Y());
           ignition::math::Vector3d controlTorque(0, torque, 0); // Torque around Y-axis for pitch

           // Add damping to reduce oscillation
           ignition::math::Vector3d dampingTorque = -this->dampingCoeff * currentAngularVel;

           // Combine torques and apply to the base link
           ignition::math::Vector3d totalTorque = controlTorque + dampingTorque;
           this->baseLink->AddTorque(totalTorque);
         }

       private:
         physics::ModelPtr model;
         physics::LinkPtr baseLink;
         double targetPitch; // Target pitch angle in radians
         double maxAngularVel; // Maximum angular velocity (rad/s) for slow rotation
         double pitchGain; // Proportional gain for torque control
         double dampingCoeff; // Damping coefficient to reduce oscillation
         event::ConnectionPtr updateConnection;
         std::shared_ptr<gazebo_ros::Node> rosNode;
         rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
       };

       // Register this plugin with the simulator
       GZ_REGISTER_MODEL_PLUGIN(CustomPitchControlPlugin)
     }