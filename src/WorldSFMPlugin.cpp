/***********************************************************************/
/**                                                                    */
/** WorldSFMPlugin.cpp                                                 */
/**                                                                    */
/**                                                                    */
/***********************************************************************/

#include <functional>
#include <stdio.h>
#include <string>

#include <gazebo_sfm_plugin/WorldSFMPlugin.h>

using namespace gazebo;
GZ_REGISTER_WORLD_PLUGIN(WorldSFMPlugin)

#define WALKING_ANIMATION "walking"

/////////////////////////////////////////////////
WorldSFMPlugin::WorldSFMPlugin() {}

/////////////////////////////////////////////////
void WorldSFMPlugin::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) {
  this->sdf = _sdf;
  this->world = _world;

  // Get the actors parameters
  this->actorParamsNode = std::make_shared<rclcpp::Node>(this->nodeName);
  this->actorParamsClient = this->actorParamsNode->create_client<rcl_interfaces::srv::GetParameters>("/agent_params_loader/get_parameters");

  // Load agents parameters from Yaml file
  this->LoadAgentsFromYaml();

  // Set the initial pose of each actor and load all params on the SFM plugin
  this->InitializeActors();

  // Node to publish forces
  this->actorForcesNode = std::make_shared<rclcpp::Node>("publish_Forces_Node");

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
      std::bind(&WorldSFMPlugin::OnUpdate, this, std::placeholders::_1)));

  // Reset settings (goals and animations)
  this->Reset();
}

/////////////////////////////////////////////////
void WorldSFMPlugin::Reset() {
  this->lastUpdate = 0;
  
  for (unsigned int i = 0; i < this->sfmActors.size(); ++i) {
    // Initialize Goals 
    for (unsigned int j = 0; j < this->agentGoals[i].size(); ++j) {
      sfm::Goal sfmGoal;
      sfmGoal.center.set(std::get<0>(this->agentGoals[i][j]),std::get<1>(this->agentGoals[i][j]));
      sfmGoal.radius = 0.3;
      this->sfmActors[i].cyclicGoals = true;
      this->sfmActors[i].goals.push_back(sfmGoal);
    }
    // Initialize Trajectory of each agent
    auto skelAnims = this->actors[i]->SkeletonAnimations();
    if (skelAnims.find(this->agentAnimName[i]) == skelAnims.end()) {
      gzerr << "Skeleton animation " << this->agentAnimName[i] << " not found.\n";
    } else {
      physics::TrajectoryInfoPtr trajectory;
      trajectory.reset(new physics::TrajectoryInfo());
      trajectory->type = this->agentAnimName[i];
      trajectory->duration = 1.0;
      this->trajectoryInfo.push_back(trajectory);
      this->actors[i]->SetCustomTrajectory(this->trajectoryInfo[i]);
    }
    // Create publisher for each agent
    rclcpp::Publisher<gazebo_sfm_plugin::msg::Forces>::SharedPtr pub = this->actorForcesNode->create_publisher<gazebo_sfm_plugin::msg::Forces>(this->agentTopicName[i], 10);
    this->actorForcesPub.push_back(pub);
  }
}

/////////////////////////////////////////////////
void WorldSFMPlugin::HandleObstacles() {
  for (unsigned int k = 0; k < this->sfmActors.size(); ++k) {
    double minDist;
    ignition::math::Vector2d closest_obs;
    ignition::math::Vector2d closest_obs2;
    this->sfmActors[k].obstacles1.clear();

    for (unsigned int i = 0; i < this->world->ModelCount(); ++i) {
      physics::ModelPtr model = this->world->ModelByIndex(i);
      if (std::find(this->agentIgnoreObs[k].begin(), this->agentIgnoreObs[k].end(), model->GetName()) == this->agentIgnoreObs[k].end()) {
        ignition::math::Vector3d actorPos = this->actors[k]->WorldPose().Pos();
        ignition::math::Vector3d modelPos = model->WorldPose().Pos();
        
        ignition::math::Vector2d minBB(model->CollisionBoundingBox().Min().X(),model->CollisionBoundingBox().Min().Y());
        ignition::math::Vector2d maxBB(model->CollisionBoundingBox().Max().X(),model->CollisionBoundingBox().Max().Y());
        ignition::math::Vector2d thirdV(minBB.X(),maxBB.Y());
        ignition::math::Vector2d fourthV(maxBB.X(),minBB.Y());

        std::vector<std::vector<ignition::math::Vector2d>> segments = {{minBB,fourthV},{fourthV,maxBB},{thirdV,maxBB},{minBB,thirdV}};

        ignition::math::Vector2d a;
        ignition::math::Vector2d b;
        ignition::math::Vector2d h;
        double dist;

        minDist = 10000;

        for(unsigned int j = 0; j < segments.size(); ++j) {
          a = std::min(segments[j][0], segments[j][1]);
          b = std::max(segments[j][0], segments[j][1]);
          double t = ((actorPos.X() - a.X()) * (b.X() - a.X()) + (actorPos.Y() - a.Y()) * (b.Y() - a.Y())) / (std::pow(b.X() - a.X(), 2) + std::pow(b.Y() - a.Y(), 2));
          double t_star = std::min(std::max(0.0,t),1.0);
          h = a + t_star * (b - a);
          dist = std::sqrt(std::pow(h.X() - actorPos.X(), 2) + std::pow(h.Y() - actorPos.Y(),2));
          if (dist < minDist) {
            minDist = dist;
            closest_obs = h;
          }
          // At the last segment,the closest point of the obstacle is passed to the lightSFM library if its distance is lower than 2 meters
          if (j == segments.size() - 1 && minDist < 2) {
            utils::Vector2d ob(closest_obs.X(), closest_obs.Y());
            this->sfmActors[k].obstacles1.push_back(ob);
          }
        }
      }
    }
  }
}

/////////////////////////////////////////////////
void WorldSFMPlugin::OnUpdate(const common::UpdateInfo &_info) {
  double dt = (_info.simTime - this->lastUpdate).Double();
  // If sampling time is passed from last update
  if (dt >= this->samplingTime){
    // Update closest obstacle
    HandleObstacles();

    this->sfmActors = sfm::SFM.computeForces(this->sfmActors);

    // Update model
    if (!this->rungeKutta45) {
      this->sfmActors = sfm::SFM.updatePosition(this->sfmActors, dt);
    } else {
      this->sfmActors = sfm::SFM.updatePositionRKF45(this->sfmActors, _info.simTime.Double(), dt);
    }

    // Publish forces
    PublishForces();

    for (unsigned int i = 0; i < this->sfmActors.size(); ++i) {
      ignition::math::Pose3d actorPose = this->actors[i]->WorldPose();

      utils::Angle h = this->sfmActors[i].yaw;
      utils::Angle add = utils::Angle::fromRadian(1.5707);
      h = h + add;
      double yaw = h.toRadian();

      actorPose.Pos().X(this->sfmActors[i].position.getX());
      actorPose.Pos().Y(this->sfmActors[i].position.getY());
      actorPose.Pos().Z(1.01);
      actorPose.Rot() = ignition::math::Quaterniond(1.5707, 0, yaw);

      double distanceTraveled = (actorPose.Pos() - this->actors[i]->WorldPose().Pos()).Length();

      this->actors[i]->SetWorldPose(actorPose, false, false);
      this->actors[i]->SetScriptTime(this->actors[i]->ScriptTime() + (distanceTraveled * this->agentAnimFact[i]));
    }

    this->lastUpdate = _info.simTime;
  }
}

////////////////////////////////////////////////
void WorldSFMPlugin::LoadAgentsFromYaml() {
  // FIRST REQUEST to get the agents names and other global parameters
  auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  request->names = {"agents","sampling_time","runge_kutta_45","robot_name"};
  this->actorParamsClient->wait_for_service();
  auto future = this->actorParamsClient->async_send_request(request);

  if (rclcpp::spin_until_future_complete(this->actorParamsNode, future) == rclcpp::FutureReturnCode::SUCCESS) {
    auto results = future.get()->values;
    // Sampling time
    if (results[1].type == rclcpp::PARAMETER_NOT_SET) {
      this->samplingTime = 0.001;
      std::cout<<"Sampling time not set, setting it to the default value: 0.001"<<std::endl;
    } else {
      this->samplingTime = results[1].double_value;
    }
    // Integration method
    if (results[2].type == rclcpp::PARAMETER_NOT_SET) {
      this->rungeKutta45 = false;
      std::cout<<"Integration method not set, setting it to the default value: Euler"<<std::endl;
    } else {
      this->rungeKutta45 = results[2].bool_value;
    }
    // Robot name
    if (results[3].type == rclcpp::PARAMETER_NOT_SET) {
      this->robotName = "robot";
      std::cout<<"Robot name not set, setting it to the default value: robot"<<std::endl;
    } else {
      this->robotName = results[3].string_value;
    }
    // Agent names and Agent model initialization
    for(unsigned int i = 0; i < results[0].string_array_value.size(); ++i){
      this->agentNames.push_back(results[0].string_array_value[i]);
      this->agentModel.push_back(this->world->ModelByName(this->agentNames[i]));
      this->actors.push_back(boost::dynamic_pointer_cast<physics::Actor>(this->agentModel[i]));
    }
    // std::cout<<"Sampling time: "<<this->samplingTime<<" - Runge-Kutta-45: "<<this->rungeKutta45<<std::endl;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service get_parameters()");
  }

  // SECOND ROUND OF REQUESTS to get the parameters of each agent
  for(unsigned int i = 0; i < this->agentNames.size(); ++i){
    request->names.clear();
    request->names = {this->agentNames[i] + ".mass", this->agentNames[i] + ".radius", this->agentNames[i] + ".goal_weight", \
    this->agentNames[i] + ".obstacle_weight", this->agentNames[i] + ".social_weight", this->agentNames[i] + ".group_gaze_weight", \
    this->agentNames[i] + ".group_coh_weight", this->agentNames[i] + ".group_rep_weight", this->agentNames[i] + ".velocity", \
    this->agentNames[i] + ".animation_factor", this->agentNames[i] + ".animation_name", this->agentNames[i] + ".people_distance", \
    this->agentNames[i] + ".ignore_obstacles", this->agentNames[i] + ".waypoints", this->agentNames[i] + ".publish_forces", \
    this->agentNames[i] + ".group", this->agentNames[i] + ".initial_orientation", this->agentNames[i] + ".initial_position"};

    auto future = this->actorParamsClient->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->actorParamsNode, future) == rclcpp::FutureReturnCode::SUCCESS) {
      auto results = future.get()->values;
      // Mass
      if (results[0].type == rclcpp::PARAMETER_NOT_SET) {
        this->agentMass.push_back(75);
        std::cout<<"Mass for " + this->agentNames[i] + " not set, setting it to the default value: 75"<<std::endl;
      } else {
        this->agentMass.push_back(results[0].integer_value);
      }
      // Radius
      if (results[1].type == rclcpp::PARAMETER_NOT_SET) {
        this->agentRadius.push_back(0.35);
        std::cout<<"Radius for " + this->agentNames[i] + " not set, setting it to the default value: 0.35"<<std::endl;
      } else {
        this->agentRadius.push_back(results[1].double_value);
      }
      // Goal Weight
      if (results[2].type == rclcpp::PARAMETER_NOT_SET) {
        this->agentGoalWeight.push_back(2.0);
        std::cout<<"Goal weight for " + this->agentNames[i] + " not set, setting it to the default value: 2.0"<<std::endl;
      } else {
        this->agentGoalWeight.push_back(results[2].double_value);
      }
      // Obstacle Weight
      if (results[3].type == rclcpp::PARAMETER_NOT_SET) {
        this->agentObstacleWeight.push_back(10.0);
        std::cout<<"Obstacle weight for " + this->agentNames[i] + " not set, setting it to the default value: 10.0"<<std::endl;
      } else {
        this->agentObstacleWeight.push_back(results[3].double_value);
      }
      // Social Weight
      if (results[4].type == rclcpp::PARAMETER_NOT_SET) {
        this->agentSocialWeight.push_back(15.0);
        std::cout<<"Social weight for " + this->agentNames[i] + " not set, setting it to the default value: 15.0"<<std::endl;
      } else {
        this->agentSocialWeight.push_back(results[4].double_value);
      }
      // Group Gaze Weight
      if (results[5].type == rclcpp::PARAMETER_NOT_SET){
        this->agentGroupGaze.push_back(0.0);
        std::cout<<"Group gaze weight for " + this->agentNames[i] + " not set, setting it to the default value: 0.0"<<std::endl;
      } else {
        this->agentGroupGaze.push_back(results[5].double_value);
      }
      // Group Cohesion Weight
      if (results[6].type == rclcpp::PARAMETER_NOT_SET) {
        this->agentGroupCoh.push_back(0.0);
        std::cout<<"Group cohesion weight for " + this->agentNames[i] + " not set, setting it to the default value: 0.0"<<std::endl;
      } else {
        this->agentGroupCoh.push_back(results[6].double_value);
      }
      // Group Repulsion Weight
      if (results[7].type == rclcpp::PARAMETER_NOT_SET) {
        this->agentGroupRep.push_back(0.0);
        std::cout<<"Group repulsion weight for " + this->agentNames[i] + " not set, setting it to the default value: 0.0"<<std::endl;
      } else {
        this->agentGroupRep.push_back(results[7].double_value);
      }
      // Desired Velocity
      if (results[8].type == rclcpp::PARAMETER_NOT_SET) {
        this->agentDesVelocity.push_back(0.9);
        std::cout<<"Desired velocity for " + this->agentNames[i] + " not set, setting it to the default value: 0.9"<<std::endl;
      } else {
        this->agentDesVelocity.push_back(results[8].double_value);
      }
      // Animation Factor
      if (results[9].type == rclcpp::PARAMETER_NOT_SET) {
        this->agentAnimFact.push_back(5.1);
        std::cout<<"Animation factor for " + this->agentNames[i] + " not set, setting it to the default value: 5.1"<<std::endl;
      } else {
        this->agentAnimFact.push_back(results[9].double_value);
      }
      // Animation Name
      if (results[10].type == rclcpp::PARAMETER_NOT_SET) {
        this->agentAnimName.push_back(WALKING_ANIMATION);
        std::cout<<"Animation name for " + this->agentNames[i] + " not set, setting it to the default value: walking"<<std::endl;
      } else {
        this->agentAnimName.push_back(results[10].string_value);
      }
      // People Distance
      if (results[11].type == rclcpp::PARAMETER_NOT_SET) {
        this->agentPeopleDist.push_back(6.0);
        std::cout<<"People distance for " + this->agentNames[i] + " not set, setting it to the default value: 6.0"<<std::endl;
      } else {
        this->agentPeopleDist.push_back(results[11].double_value);
      }
      // Publish Forces Boolean
      if (results[14].type == rclcpp::PARAMETER_NOT_SET) {
        this->agentPubForces.push_back(false);
        std::cout<<"Publish forces for " + this->agentNames[i] + " not set, setting it to the default value: false"<<std::endl;
      } else {
        this->agentPubForces.push_back(results[14].bool_value);
      }
      // Topic and publisher used to publish forces
      this->agentTopicName.push_back("/forces/" + this->agentNames[i]);
      // Obstacles to ignore
      std::vector<std::string> obs;
      for (unsigned int j = 0; j < this->agentNames.size(); ++j){
        obs.push_back(this->agentNames[j]); // All the actor's models must be ignored
      }
      if (results[12].type == rclcpp::PARAMETER_NOT_SET) {
        std::cout<<"No obstacles to ignore for " + this->agentNames[i]<<std::endl;
        this->agentIgnoreObs.push_back(obs);
      } else {
        for (unsigned int j = 0; j <results[12].string_array_value.size(); ++j){
          obs.push_back(results[12].string_array_value[j]);
        }
        this->agentIgnoreObs.push_back(obs);
      }
      obs.clear();
      // Waypoints - THESE MUST BE LOADED
      std::vector<std::tuple<double,double>> goals;
      for (unsigned int j = 0; j <results[13].double_array_value.size(); j+=2){
        goals.push_back(std::make_tuple(results[13].double_array_value[j],results[13].double_array_value[j+1]));
      }
      this->agentGoals.push_back(goals);
      goals.clear();
      // Initial Orientation
      if (results[16].type == rclcpp::PARAMETER_NOT_SET) {
        this->agentInitYaw.push_back(0.0);
        std::cout<<"Initial orientation for " + this->agentNames[i] + " not set, setting it to the default value: 0.0 radians"<<std::endl;
      } else {
        this->agentInitYaw.push_back(results[16].double_value * (M_PI_2 / 90.0));
      }
      // Initial Position
      if (results[17].type == rclcpp::PARAMETER_NOT_SET) {
        this->agentInitPos.push_back(this->agentGoals[i][0]);
        std::cout<<"Initial position for " + this->agentNames[i] + " not set, setting it as the first waypoint: "<<std::get<0>(this->agentGoals[i][0])<<","<<std::get<1>(this->agentGoals[i][0])<<std::endl;
      } else {
        std::tuple<double,double> pos(results[17].double_array_value[0],results[17].double_array_value[1]);
        this->agentInitPos.push_back(pos);
      }
      // Agents in group
      if (results[12].type == rclcpp::PARAMETER_NOT_SET) {
        std::cout<<"No group for " + this->agentNames[i]<<std::endl;
        this->agentGroup.push_back(std::vector<std::string>());
      } else {
        std::vector<std::string> group;
        for (unsigned int j = 0; j <results[15].string_array_value.size(); ++j){
          group.push_back(results[15].string_array_value[j]);
        }
        this->agentGroup.push_back(group);
        group.clear();
      }
      //std::cout<<this->agentNames[i]<<": [mass: "<<this->agentMass[i]<<", radius: "<<this->agentRadius[i]<<", goal_weight: "<<this->agentGoalWeight[i]<<", obstacle_weight: "<<this->agentObstacleWeight[i]<<", social_weight: "<<this->agentSocialWeight[i]<<", group_gaze: "<<this->agentGroupGaze[i]<<", group_coh: "<<this->agentGroupCoh[i]<<", group_rep: "<<this->agentGroupRep[i]<<", des_velocity: "<<this->agentDesVelocity[i]<<", anim_fact: "<<this->agentAnimFact[i]<<", anim_name: "<<this->agentAnimName[i]<<", people_dist: "<<this->agentPeopleDist[i]<<", first_ignored_obs: "<<this->agentIgnoreObs[i][0]<<", first_goal: ("<<std::get<0>(this->agentGoals[i][0])<<", "<<std::get<1>(this->agentGoals[i][0])<<") "<<", publish_forces: "<<this->agentPubForces[i]<<", topic_name: "<<this->agentTopicName[i]<<", first_agent_in_group: "<<this->agentGroup[i][0]<<", initial_orientation: "<<this->agentInitYaw[i]<<", initial_position: "<<std::get<0>(this->agentInitPos[i])<<","<<std::get<1>(this->agentInitPos[i])<<"]"<<std::endl;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service get_parameters()");
    }
  }
}

////////////////////////////////////////////////
void WorldSFMPlugin::InitializeActors() {
  for (unsigned int i = 0; i < this->actors.size(); ++i) {
    ignition::math::Pose3d actorPose = this->actors[i]->WorldPose();

    // TODO: REVIEW THIS PART - Set the initial pose of each actor
    // actorPose.Pos().X(std::get<0>(this->agentInitPos[i]));
    // actorPose.Pos().Y(std::get<1>(this->agentInitPos[i]));
    // actorPose.Pos().Z(1.01);
    // actorPose.Rot() = ignition::math::Quaterniond(1.5707, 0, this->agentInitYaw[i] + M_PI_2);
    // this->actors[i]->SetWorldPose(actorPose, false, false);

    // Initialize the SFM actors
    ignition::math::Pose3d pose = this->actors[i]->WorldPose();
    ignition::math::Vector3d linvel = this->actors[i]->WorldLinearVel();
    ignition::math::Vector3d angvel = this->actors[i]->WorldAngularVel();
    sfm::Agent agent;
    agent.id = this->actors[i]->GetId();
    agent.position.set(pose.Pos().X(),pose.Pos().Y());
    agent.yaw = pose.Rot().Euler().Z();
    agent.velocity.set(linvel.X(),linvel.Y());
    agent.linearVelocity = linvel.Length();
    agent.angularVelocity = angvel.Z();
    agent.desiredVelocity = this->agentDesVelocity[i];
    agent.params.forceFactorDesired = this->agentGoalWeight[i];
    agent.params.forceFactorObstacle = this->agentObstacleWeight[i];
    agent.params.forceFactorSocial = this->agentSocialWeight[i];
    agent.params.forceFactorGroupGaze = this->agentGroupGaze[i];
    agent.params.forceFactorGroupCoherence = this->agentGroupCoh[i];
    agent.params.forceFactorGroupRepulsion = this->agentGroupRep[i];
    if (this->agentGroup[i].size() > 0) {
      agent.groupId = agent.id;
    } else {
      agent.groupId = -1;
    }
    this->sfmActors.push_back(agent);
  }
}

////////////////////////////////////////////////
void WorldSFMPlugin::PublishForces() {
  for (unsigned int i = 0; i < this->actors.size(); ++i) {
    if (this->agentPubForces[i] == true) {
      gazebo_sfm_plugin::msg::Forces msg = gazebo_sfm_plugin::msg::Forces();
      // Global force
      msg.global_force.x = this->sfmActors[i].forces.globalForce.getX();
      msg.global_force.y = this->sfmActors[i].forces.globalForce.getY();
      // Desired force
      msg.desired_force.x = this->sfmActors[i].forces.desiredForce.getX();
      msg.desired_force.y = this->sfmActors[i].forces.desiredForce.getY();
      // Obstacle force
      msg.obstacle_force.x = this->sfmActors[i].forces.obstacleForce.getX();
      msg.obstacle_force.y = this->sfmActors[i].forces.obstacleForce.getY();
      // Social force
      msg.social_force.x = this->sfmActors[i].forces.socialForce.getX();
      msg.social_force.y = this->sfmActors[i].forces.socialForce.getY();
      // Group force
      msg.group_force.x = this->sfmActors[i].forces.groupForce.getX();
      msg.group_force.y = this->sfmActors[i].forces.groupForce.getY();
      // // Torque force - not implemented
      // msg.torque_force = 0.0;
      // Linear velocity
      msg.linear_velocity.x = this->sfmActors[i].velocity.getX();
      msg.linear_velocity.y = this->sfmActors[i].velocity.getY();
      // // Angular velocity - not implemented
      // msg.angular_velocity = 0.0;
      // Pose
      msg.pose.x = this->sfmActors[i].initPosition.getX();
      msg.pose.y = this->sfmActors[i].initPosition.getY();
      msg.pose.theta = this->sfmActors[i].initYaw.toDegree();

      this->actorForcesPub[i]->publish(msg);
    }
  }
}