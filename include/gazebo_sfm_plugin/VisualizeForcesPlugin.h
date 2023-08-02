#ifndef GAZEBO_PLUGINS_VISUALIZEFORCESPLUGIN_HH_
#define GAZEBO_PLUGINS_VISUALIZEFORCESPLUGIN_HH_

// C++
#include <algorithm>
#include <string>
#include <vector>
#include <functional>
#include <stdio.h>

// Gazebo
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/util/system.hh"
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

// ROS
#include <rclcpp/rclcpp.hpp>

// Custom Msgs
#include "gazebo_sfm_plugin/msg/forces.hpp"
#include "gazebo_sfm_plugin/msg/vector2.hpp"
#include "gazebo_sfm_plugin/msg/pose2.hpp"

namespace gazebo {
    class GZ_PLUGIN_VISIBLE VisualizeForcesPlugin : public VisualPlugin {

        /// \brief Constructor
        public:
        VisualizeForcesPlugin();

        /// \brief Load the actor plugin.
        /// \param[in] _parent Pointer to the parent visual.
        /// \param[in] _sdf Pointer to the plugin's SDF elements.
        public:
        virtual void Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf);

        /// \brief Function that is called every update cycle.
        private:
        void OnUpdate();

        /// \brief Function that is called whenever the Forces subscriber receives a new message - HSFM
        /// \param[in] msg Message containing the forces data
        private:
        void VisualizeForcesHSFM(const gazebo_sfm_plugin::msg::Forces &msg);

        /// \brief Function that is called whenever the Forces subscriber receives a new message - SFM
        /// \param[in] msg Message containing the forces data
        private:
        void VisualizeForcesSFM(const gazebo_sfm_plugin::msg::Forces &msg);
        //-------------------------------------------------
        /// \brief Pointer to the sdf element.
        private:
        sdf::ElementPtr sdf;

        /// \brief pointer to the visual element used to visualized force
        private:
        rendering::VisualPtr visual;

        /// \brief pointer to the visual world pose
        private:
        ignition::math::Pose3d visual_pose;

        /// \brief pointer to the scene
        private:
        rendering::ScenePtr scene;
        
        /// \brief pointer to the visualize forces node
        private:
        rclcpp::Node::SharedPtr visualNode;

        /// \brief name of the visualize forces node
        private:
        std::string nodeName;

        /// \brief stores the topic name where forces are published
        private:
        std::string topicName;

        /// \brief boolean value to account for headed social force model
        private:
        bool headed;

        /// \brief line to visualize the global force
        private:
        rendering::DynamicLines *global_force;

        /// \brief line to visualize the obstacle force
        private:
        rendering::DynamicLines *obstacle_force;

        /// \brief line to visualize the desired force
        private:
        rendering::DynamicLines *desired_force;

        /// \brief line to visualize the social force
        private:
        rendering::DynamicLines *social_force;

        /// \brief line to visualize the group force
        private:
        rendering::DynamicLines *group_force;

        /// \brief line to visualize the linear velocity
        private:
        rendering::DynamicLines *linear_velocity;

        /// \brief subscriber to get forces data
        private:
        rclcpp::Subscription<gazebo_sfm_plugin::msg::Forces>::SharedPtr forcesSub;

        /// \brief List of connections
        private:
        std::vector<event::ConnectionPtr> connections;
    };
} // namespace gazebo
#endif
