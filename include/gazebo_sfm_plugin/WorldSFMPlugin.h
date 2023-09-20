/***********************************************************************/
/**                                                                    */
/** WorldSFMPlugin.h                                                   */
/**                                                                    */
/**                                                                    */
/***********************************************************************/

#ifndef GAZEBO_PLUGINS_WORLDSFMPLUGIN_HH_
#define GAZEBO_PLUGINS_WORLDSFMPLUGIN_HH_

// C++
#include <algorithm>
#include <string>
#include <vector>

// Gazebo
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"
#include "gazebo/sensors/sensors.hh"
#include <gazebo/physics/World.hh>
#include <rclcpp/rclcpp.hpp>

// Msgs
#include "gazebo_sfm_plugin/msg/forces.hpp"
#include "gazebo_sfm_plugin/msg/vector2.hpp"
#include "gazebo_sfm_plugin/msg/pose2.hpp"

// Social Force Model
#include <gazebo_sfm_plugin/sfm.hpp>

namespace gazebo {
class GZ_PLUGIN_VISIBLE WorldSFMPlugin : public WorldPlugin {
  
  // METHODS -------------------------------------------------
public:
  /// \brief Constructor
  WorldSFMPlugin();

  /// \brief Load the actor plugin.
  /// \param[in] _world Pointer to the parent model.
  /// \param[in] _sdf Pointer to the plugin's SDF elements.
  virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

  // Documentation Inherited.
  virtual void Reset();

  /// \brief Function that is called every update cycle.
  /// \param[in] _info Timing information
  void OnUpdate(const common::UpdateInfo &_info);

private:
  /// \brief Helper function to detect the closest obstacles.
  void HandleObstacles();

  /// \brief Loads the agents' parameters from a config file
  void LoadAgentsFromYaml();

  /// \brief Initialize actors pose and params for SFM plugin
  void InitializeActors();

  /// \brief Method used to publish forces
  void PublishForces();

  // ATTIRBUTES -------------------------------------------------
private:
  /// \brief Pointer to the world, for convenience.
  physics::WorldPtr world;

  /// \brief Pointer to the sdf element.
  sdf::ElementPtr sdf;

  /// \brief List of connections
  std::vector<event::ConnectionPtr> connections;

  /// \brief Time of the last update.
  common::Time lastUpdate;

  /// \brief The sampling time of the plugin
  double samplingTime = 0.001;

  /// \brief The integration method, if true RKF45 is used, otherwise, first order Euler is used
  bool rungeKutta45 = false;

  /// \brief Node to get the parameters in the configuration file
  rclcpp::Node::SharedPtr actorParamsNode;

  /// \brief Node to get the parameters in the configuration file
  rclcpp::Node::SharedPtr actorForcesNode;

  /// \brief Stores the forces publisher
  std::vector<rclcpp::Publisher<gazebo_sfm_plugin::msg::Forces>::SharedPtr> actorForcesPub;

  /// \brief Client to call for the get_parameters() service
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr actorParamsClient;

  /// \brief Name of node used to get actors parameters
  std::string nodeName = "actor_params_grabber";

  /// \brief Variable to store agent names
  std::vector<std::string> agentNames;

  /// \brief Variable to store agent mass
  std::vector<int> agentMass;

  /// \brief Variable to store agent radius
  std::vector<double> agentRadius;

  /// \brief Variable to store agent goal weight
  std::vector<double> agentGoalWeight;

  /// \brief Variable to store agent obstacle weight
  std::vector<double> agentObstacleWeight;

  /// \brief Variable to store agent social weight
  std::vector<double> agentSocialWeight;

  /// \brief Variable to store agent group gaze weight
  std::vector<double> agentGroupGaze;

  /// \brief Variable to store agent group cohesion weight
  std::vector<double> agentGroupCoh;

  /// \brief Variable to store agent group repulsion weight
  std::vector<double> agentGroupRep;

  /// \brief Variable to store agent desired velocity
  std::vector<double> agentDesVelocity;

  /// \brief Variable to store agent animation factor
  std::vector<double> agentAnimFact;

  /// \brief Variable to store agent animation name
  std::vector<std::string> agentAnimName;

  /// \brief Variable to store agent people distance
  std::vector<double> agentPeopleDist;

  /// \brief Variable to store agent ignored obstacles
  std::vector<std::vector<std::string>> agentIgnoreObs;

  /// \brief Variable to store agent goals
  std::vector<std::vector<std::tuple<double,double>>> agentGoals;

  /// \brief Variable to store agent intial orientations
  std::vector<double> agentInitYaw;

  /// \brief Variable to store agent intial orientations
  std::vector<std::tuple<double,double>> agentInitPos;

  /// \brief Variable to store agent boolean publish forces
  std::vector<bool> agentPubForces;

  /// \brief Variable to store agent topic to publish forces names
  std::vector<std::string> agentTopicName;

  /// \brief Variable to store agents in the agent group
  std::vector<std::vector<std::string>> agentGroup;

  /// \brief Variable to store robot's name
  std::string robotName;

  /// \brief Vector of pointers to the actors model
  std::vector<physics::ModelPtr> agentModel;

  /// \brief Vector of pointers to the actor
  std::vector<physics::ActorPtr> actors;

  /// \brief Saves actor as a SFM agent
  std::vector<sfm::Agent> sfmActors;

  /// \brief Vector of actors trajectories
  std::vector<physics::TrajectoryInfoPtr> trajectoryInfo;

  /// \brief Saves sfmActors used for repulsive force from other pedestrians
  std::vector<std::vector<sfm::Agent>> otherActors;
};
} // namespace gazebo
#endif
