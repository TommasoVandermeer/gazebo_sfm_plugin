/***********************************************************************/
/**                                                                    */
/** PedestrianSFMPlugin.cpp                                            */
/**                                                                    */
/** Copyright (c) 2022, Service Robotics Lab (SRL).                    */
/**                     http://robotics.upo.es                         */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** Noé Pérez-Higueras (maintainer)                                    */
/** email: noeperez@upo.es                                             */
/**                                                                    */
/** This software may be modified and distributed under the terms      */
/** of the BSD license. See the LICENSE file for details.              */
/**                                                                    */
/** http://www.opensource.org/licenses/BSD-3-Clause                    */
/**                                                                    */
/***********************************************************************/

#include <functional>
#include <stdio.h>
#include <string>

//#include <ignition/math.hh>
//#include <ignition/math/gzmath.hh>
#include <gazebo_sfm_plugin/PedestrianSFMPlugin.h>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(PedestrianSFMPlugin)

#define WALKING_ANIMATION "walking"

/////////////////////////////////////////////////
PedestrianSFMPlugin::PedestrianSFMPlugin() {}

/////////////////////////////////////////////////
void PedestrianSFMPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();

  this->sfmActor.id = this->actor->GetId();

  // Initialize sfmActor position
  ignition::math::Vector3d pos = this->actor->WorldPose().Pos();
  ignition::math::Vector3d rpy = this->actor->WorldPose().Rot().Euler();
  this->sfmActor.position.set(pos.X(), pos.Y());
  this->sfmActor.yaw = utils::Angle::fromRadian(rpy.Z()); // yaw
  ignition::math::Vector3d linvel = this->actor->WorldLinearVel();
  this->sfmActor.velocity.set(linvel.X(), linvel.Y());
  this->sfmActor.linearVelocity = linvel.Length();
  ignition::math::Vector3d angvel = this->actor->WorldAngularVel();
  this->sfmActor.angularVelocity = angvel.Z(); // Length()

  // Read in the maximum velocity of the pedestrian
  if (_sdf->HasElement("velocity"))
    this->sfmActor.desiredVelocity = _sdf->Get<double>("velocity");
  else
    this->sfmActor.desiredVelocity = 0.8;

  // Read in the target weight
  if (_sdf->HasElement("goal_weight"))
    this->sfmActor.params.forceFactorDesired = _sdf->Get<double>("goal_weight");
  // Read in the obstacle weight
  if (_sdf->HasElement("obstacle_weight"))
    this->sfmActor.params.forceFactorObstacle =
        _sdf->Get<double>("obstacle_weight");
  // Read in the social weight
  if (_sdf->HasElement("social_weight"))
    this->sfmActor.params.forceFactorSocial =
        _sdf->Get<double>("social_weight");
  // Read in the group gaze weight
  if (_sdf->HasElement("group_gaze_weight"))
    this->sfmActor.params.forceFactorGroupGaze =
        _sdf->Get<double>("group_gaze_weight");
  // Read in the group coherence weight
  if (_sdf->HasElement("group_coh_weight"))
    this->sfmActor.params.forceFactorGroupCoherence =
        _sdf->Get<double>("group_coh_weight");
  // Read in the group repulsion weight
  if (_sdf->HasElement("group_rep_weight"))
    this->sfmActor.params.forceFactorGroupRepulsion =
        _sdf->Get<double>("group_rep_weight");

  // Read in the animation factor (applied in the OnUpdate function).
  if (_sdf->HasElement("animation_factor"))
    this->animationFactor = _sdf->Get<double>("animation_factor");
  else
    this->animationFactor = 4.5;

  if (_sdf->HasElement("animation_name")) {
    this->animationName = _sdf->Get<std::string>("animation_name");
  } else
    this->animationName = WALKING_ANIMATION;

  if (_sdf->HasElement("people_distance"))
    this->peopleDistance = _sdf->Get<double>("people_distance");
  else
    this->peopleDistance = 5.0;

  // Read in the pedestrians in your walking group
  if (_sdf->HasElement("group")) {
    this->sfmActor.groupId = this->sfmActor.id;
    sdf::ElementPtr modelElem = _sdf->GetElement("group")->GetElement("model");
    while (modelElem) {
      this->groupNames.push_back(modelElem->Get<std::string>());
      modelElem = modelElem->GetNextElement("model");
    }
    this->sfmActor.groupId = this->sfmActor.id;
  } else
    this->sfmActor.groupId = -1;

  // Read in the other obstacles to ignore
  if (_sdf->HasElement("ignore_obstacles")) {
    sdf::ElementPtr modelElem =
        _sdf->GetElement("ignore_obstacles")->GetElement("model");
    while (modelElem) {
      this->ignoreModels.push_back(modelElem->Get<std::string>());
      modelElem = modelElem->GetNextElement("model");
    }
  }
  // Add our own name to models we should ignore when avoiding obstacles.
  this->ignoreModels.push_back(this->actor->GetName());
  // Add the other pedestrians to the ignored obstacles
  for (unsigned int i = 0; i < this->world->ModelCount(); ++i) {
    physics::ModelPtr model = this->world->ModelByIndex(i); // GetModel(i);

    if (model->GetId() != this->actor->GetId() &&
        ((int)model->GetType() == (int)this->actor->GetType())) {
      this->ignoreModels.push_back(model->GetName());
    }
  }

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
      std::bind(&PedestrianSFMPlugin::OnUpdate, this, std::placeholders::_1)));

  this->Reset();
}

/////////////////////////////////////////////////
void PedestrianSFMPlugin::Reset() {
  // this->velocity = 0.8;
  this->lastUpdate = 0;

  // Read in the goals to reach
  if (this->sdf->HasElement("trajectory")) {
    sdf::ElementPtr modelElemCyclic =
        this->sdf->GetElement("trajectory")->GetElement("cyclic");

    if (modelElemCyclic)
      this->sfmActor.cyclicGoals = modelElemCyclic->Get<bool>();

    sdf::ElementPtr modelElem =
        this->sdf->GetElement("trajectory")->GetElement("waypoint");
    while (modelElem) {
      ignition::math::Vector3d g = modelElem->Get<ignition::math::Vector3d>();
      sfm::Goal goal;
      goal.center.set(g.X(), g.Y());
      goal.radius = 0.3;
      this->sfmActor.goals.push_back(goal);
      modelElem = modelElem->GetNextElement("waypoint");
    }
  }

  auto skelAnims = this->actor->SkeletonAnimations();
  if (skelAnims.find(this->animationName) == skelAnims.end()) {
    gzerr << "Skeleton animation " << this->animationName << " not found.\n";
  } else {
    // Create custom trajectory
    this->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo->type = this->animationName;
    this->trajectoryInfo->duration = 1.0;

    this->actor->SetCustomTrajectory(this->trajectoryInfo);
  }
}

/////////////////////////////////////////////////
void PedestrianSFMPlugin::HandleObstacles() {
  double minDist = 10000.0;
  // ignition::math::Vector3d closest_obs;
  // ignition::math::Vector3d closest_obs2;
  ignition::math::Vector2d closest_obs;
  ignition::math::Vector2d closest_obs2;
  // bool same_obs = false;
  this->sfmActor.obstacles1.clear();

  for (unsigned int i = 0; i < this->world->ModelCount(); ++i) {
    physics::ModelPtr model = this->world->ModelByIndex(i); // GetModel(i);
    if (std::find(this->ignoreModels.begin(), this->ignoreModels.end(), model->GetName()) == this->ignoreModels.end()) {
      ignition::math::Vector3d actorPos = this->actor->WorldPose().Pos();
      ignition::math::Vector3d modelPos = model->WorldPose().Pos();
      
      ignition::math::Vector2d minBB(model->CollisionBoundingBox().Min().X(),model->CollisionBoundingBox().Min().Y());
      ignition::math::Vector2d maxBB(model->CollisionBoundingBox().Max().X(),model->CollisionBoundingBox().Max().Y());
      ignition::math::Vector2d thirdV(minBB.X(),maxBB.Y());
      ignition::math::Vector2d fourthV(maxBB.X(),minBB.Y());
      // std::cout<<"MinBB: ["<<minBB<<"]"<<std::endl;
      // std::cout<<"MaxBB: ["<<maxBB<<"]"<<std::endl;
      // std::cout<<"ThirdV: ["<<thirdV<<"]"<<std::endl;
      // std::cout<<"FourthV: ["<<fourthV<<"]"<<std::endl;

      // The closest point lays between the two edges whose vetices include the closest vertex to the actor
      // Map(vertexID, vertexCoordinates, vertexDistance)
      // std::map<int, std::pair<ignition::math::Vector2d, double>> vertex_sequence = {
      //   {0, std::make_pair(minBB, std::sqrt(std::pow(minBB.X() - actorPos.X(), 2) + std::pow(minBB.Y() - actorPos.Y(),2)))},
      //   {1, std::make_pair(thirdV, std::sqrt(std::pow(thirdV.X() - actorPos.X(), 2) + std::pow(thirdV.Y() - actorPos.Y(),2)))},
      //   {2, std::make_pair(maxBB, std::sqrt(std::pow(maxBB.X() - actorPos.X(), 2) + std::pow(maxBB.Y() - actorPos.Y(),2)))},
      //   {3, std::make_pair(fourthV, std::sqrt(std::pow(fourthV.X() - actorPos.X(), 2) + std::pow(fourthV.Y() - actorPos.Y(),2)))}
      // };
      // double minDist_vertex = 10000;
      // ignition::math::Vector2d closest_vertex;
      // int closest_vertex_id;
      // for (unsigned int i; i < vertex_sequence.size(); ++i) {
      //   if(vertex_sequence[i].second < minDist_vertex) {
      //     minDist_vertex = vertex_sequence[i].second;
      //     closest_vertex = vertex_sequence[i].first;
      //     closest_vertex_id = i;
      //   }
      // }
      // int v2_id;
      // int v3_id;
      // if(closest_vertex_id - 1 < 0){
      //   v2_id = 3;
      // }
      // else {
      //   v2_id = closest_vertex_id - 1;
      // }
      // if(closest_vertex_id + 1 > 3){
      //   v3_id = 0;
      // }
      // else {
      //   v3_id = closest_vertex_id + 1;
      // }
      // std::vector<std::vector<ignition::math::Vector2d>> segments = {{closest_vertex,vertex_sequence[v2_id].first},{closest_vertex,vertex_sequence[v3_id].first}};
      std::vector<std::vector<ignition::math::Vector2d>> segments = {{minBB,fourthV},{fourthV,maxBB},{thirdV,maxBB},{minBB,thirdV}};
      // std::cout<<"Segment0: ["<<segments[0][0]<<","<<segments[0][1]<<"]"<<std::endl;
      // std::cout<<"Segment1: ["<<segments[1][0]<<","<<segments[1][1]<<"]"<<std::endl;
      // std::cout<<"Segment2: ["<<segments[2][0]<<","<<segments[2][1]<<"]"<<std::endl;
      // std::cout<<"Segment3: ["<<segments[3][0]<<","<<segments[3][1]<<"]"<<std::endl;

      ignition::math::Vector2d a;
      ignition::math::Vector2d b;
      ignition::math::Vector2d h;
      double dist;

      // same_obs = false;

      for(unsigned int i = 0; i < segments.size(); ++i) {
        a = std::min(segments[i][0], segments[i][1]);
        b = std::max(segments[i][0], segments[i][1]);
        double t = ((actorPos.X() - a.X()) * (b.X() - a.X()) + (actorPos.Y() - a.Y()) * (b.Y() - a.Y())) / (std::pow(b.X() - a.X(), 2) + std::pow(b.Y() - a.Y(), 2));
        double t_star = std::min(std::max(0.0,t),1.0);
        h = a + t_star * (b - a);
        dist = std::sqrt(std::pow(h.X() - actorPos.X(), 2) + std::pow(h.Y() - actorPos.Y(),2));
        if (dist < minDist) {
          minDist = dist;
          // if (!same_obs) {
          //   closest_obs2 = closest_obs;
          // }
          closest_obs = h;
          // same_obs = true;
        }
      }
    }
  }

  if (minDist <= 10.0) {
    utils::Vector2d ob(closest_obs.X(), closest_obs.Y());
    // utils::Vector2d ob2(closest_obs2.X(), closest_obs2.Y());
    // std::cout<<"Closest obstacle: "<<ob<<std::endl;
    // std::cout<<"Min dist: "<<minDist<<std::endl;
    this->sfmActor.obstacles1.push_back(ob);
    // this->sfmActor.obstacles1.push_back(ob2);
  }
}

/////////////////////////////////////////////////
void PedestrianSFMPlugin::HandlePedestrians() {
  this->otherActors.clear();

  for (unsigned int i = 0; i < this->world->ModelCount(); ++i) {
    physics::ModelPtr model = this->world->ModelByIndex(i); // GetModel(i);

    if (model->GetId() != this->actor->GetId() &&
        ((int)model->GetType() == (int)this->actor->GetType())) {
      // printf("Actor %i has detected actor %i!\n", this->actor->GetId(),
      //        model->GetId());

      ignition::math::Pose3d modelPose = model->WorldPose();
      ignition::math::Vector3d pos =
          modelPose.Pos() - this->actor->WorldPose().Pos();
      if (pos.Length() < this->peopleDistance) {
        sfm::Agent ped;
        ped.id = model->GetId();
        ped.position.set(modelPose.Pos().X(), modelPose.Pos().Y());
        ignition::math::Vector3d rpy = modelPose.Rot().Euler();
        ped.yaw = utils::Angle::fromRadian(rpy.Z());

        ped.radius = this->sfmActor.radius;
        ignition::math::Vector3d linvel = model->WorldLinearVel();
        ped.velocity.set(linvel.X(), linvel.Y());
        ped.linearVelocity = linvel.Length();
        ignition::math::Vector3d angvel = model->WorldAngularVel();
        ped.angularVelocity = angvel.Z(); // Length()

        // check if the ped belongs to my group
        if (this->sfmActor.groupId != -1) {
          std::vector<std::string>::iterator it;
          it = find(groupNames.begin(), groupNames.end(), model->GetName());
          if (it != groupNames.end())
            ped.groupId = this->sfmActor.groupId;
          else
            ped.groupId = -1;
        }
        this->otherActors.push_back(ped);
      }
    }
  }
  // printf("Actor %s has detected %i actors!\n",
  // this->actor->GetName().c_str(),
  //        (int)this->otherActors.size());
}

/////////////////////////////////////////////////
void PedestrianSFMPlugin::OnUpdate(const common::UpdateInfo &_info) {
  // Time delta
  double dt = (_info.simTime - this->lastUpdate).Double();

  ignition::math::Pose3d actorPose = this->actor->WorldPose();

  // update closest obstacle
  HandleObstacles();

  // update pedestrian around
  HandlePedestrians();

  // Compute Social Forces
  sfm::SFM.computeForces(this->sfmActor, this->otherActors);

  // Update model
  sfm::SFM.updatePosition(this->sfmActor, dt);

  utils::Angle h = this->sfmActor.yaw;
  utils::Angle add = utils::Angle::fromRadian(1.5707);
  h = h + add;
  double yaw = h.toRadian();
  // double yaw = this->sfmActor.yaw.toRadian();
  // Rotate in place, instead of jumping.
  // if (std::abs(yaw.Radian()) > IGN_DTOR(10))
  //{
  //  ActorPose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+
  //      yaw.Radian()*0.001);
  //}
  // else
  //{
  ignition::math::Vector3d rpy = actorPose.Rot().Euler();
  utils::Angle current = utils::Angle::fromRadian(rpy.Z());
  double diff = (h - current).toRadian();
  if (std::fabs(diff) > IGN_DTOR(10)) {
    current = current + utils::Angle::fromRadian(diff * 0.005);
    yaw = current.toRadian();
  }
  actorPose.Pos().X(this->sfmActor.position.getX());
  actorPose.Pos().Y(this->sfmActor.position.getY());
  actorPose.Rot() = ignition::math::Quaterniond(1.5707, 0, yaw); // rpy.Z()+yaw.Radian());
  //}

  // Make sure the actor stays within bounds
  // actorPose.Pos().X(std::max(-3.0, std::min(3.5, actorPose.Pos().X())));
  // actorPose.Pos().Y(std::max(-10.0, std::min(2.0, actorPose.Pos().Y())));
  actorPose.Pos().Z(1.01); //1.2138

  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distanceTraveled =
      (actorPose.Pos() - this->actor->WorldPose().Pos()).Length();

  this->actor->SetWorldPose(actorPose, false, false);
  this->actor->SetScriptTime(this->actor->ScriptTime() +
                             (distanceTraveled * this->animationFactor));
  this->lastUpdate = _info.simTime;
}
