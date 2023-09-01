/***********************************************************************/
/**                                                                    */
/** PedestrianHSFMPlugin.h                                              */
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

#ifndef GAZEBO_PLUGINS_PEDESTRIANHSFMPLUGIN_HH_
#define GAZEBO_PLUGINS_PEDESTRIANHSFMPLUGIN_HH_

// C++
#include <algorithm>
#include <string>
#include <vector>

// Gazebo
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"
#include "gazebo/transport/transport.hh"
#include <rclcpp/rclcpp.hpp>

// Msgs
#include "gazebo_sfm_plugin/msg/forces.hpp"
#include "gazebo_sfm_plugin/msg/vector2.hpp"
#include "gazebo_sfm_plugin/msg/pose2.hpp"

// Social Force Model
#include <gazebo_sfm_plugin/hsfm.hpp>

namespace gazebo {
class GZ_PLUGIN_VISIBLE PedestrianHSFMPlugin : public ModelPlugin {
  /// \brief Constructor
public:
  PedestrianHSFMPlugin();

  /// \brief Load the actor plugin.
  /// \param[in] _model Pointer to the parent model.
  /// \param[in] _sdf Pointer to the plugin's SDF elements.
public:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  // Documentation Inherited.
public:
  virtual void Reset();

  /// \brief Function that is called every update cycle.
  /// \param[in] _info Timing information
private:
  void OnUpdate(const common::UpdateInfo &_info);

  // private: void InitializePedestrians();

  /// \brief Helper function to detect the closest obstacles.
private:
  void HandleObstacles();

  /// \brief Helper function to detect the nearby pedestrians (other actors).
private:
  void HandlePedestrians();

  /// \brief Helper function to publish the computed forces using the HSFM
private:
  void PublishForces();

  //-------------------------------------------------

  /// \brief this actor as a SFM agent
private:
  hsfm::Agent hsfmActor;

  /// \brief names of the other models in my walking group.
private:
  std::vector<std::string> groupNames;

  /// \brief vector of pedestrians detected.
private:
  std::vector<hsfm::Agent> otherActors;

  /// \brief Maximum distance to detect nearby pedestrians.
private:
  double peopleDistance;

  /// \brief Pointer to the parent actor.
private:
  physics::ActorPtr actor;

  /// \brief Pointer to the world, for convenience.
private:
  physics::WorldPtr world;

  /// \brief Pointer to the sdf element.
private:
  sdf::ElementPtr sdf;

  /// \brief Velocity of the actor
private:
  ignition::math::Vector3d velocity;

  /// \brief List of connections
private:
  std::vector<event::ConnectionPtr> connections;

  /// \brief Time scaling factor. Used to coordinate translational motion
  /// with the actor's walking animation.
private:
  double animationFactor = 1.0;

  /// \brief Time of the last update.
private:
  common::Time lastUpdate;

  /// \brief List of models to ignore. Used for vector field
private:
  std::vector<std::string> ignoreModels;

  /// \brief Animation name of this actor
private:
  std::string animationName;

  /// \brief Custom trajectory info.
private:
  physics::TrajectoryInfoPtr trajectoryInfo;

  /// \brief Boolean to decide wether to publish forces or not
private:
  bool publishForces;

  /// \brief Stores the topic name
private:
  std::string topicName;

  /// \brief Stores the node name
private:
  std::string nodeName;

  /// \brief Stores the node used to publish forces
private:
  rclcpp::Node::SharedPtr forcesNode;

  /// \brief Stores the forces publisher
private:
  rclcpp::Publisher<gazebo_sfm_plugin::msg::Forces>::SharedPtr forcesPub;
};
} // namespace gazebo
#endif
