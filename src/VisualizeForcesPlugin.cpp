#include <functional>
#include <stdio.h>
#include <string>

#include <gazebo_sfm_plugin/VisualizeForcesPlugin.h>

using namespace gazebo;
GZ_REGISTER_VISUAL_PLUGIN(VisualizeForcesPlugin)

/////////////////////////////////////////////////
VisualizeForcesPlugin::VisualizeForcesPlugin() {}

/////////////////////////////////////////////////
void VisualizeForcesPlugin::Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf) {
    this->sdf = _sdf;
    this->visual = _parent;
    this->scene = this->visual->GetScene();

    // Read in all the sdf elements
    if (_sdf->HasElement("topic_name"))
        this->topicName = _sdf->Get<std::string>("topic_name");
    if (_sdf->HasElement("node_name"))
        this->nodeName = _sdf->Get<std::string>("node_name");
    if (_sdf->HasElement("headed"))
        this->headed = _sdf->Get<bool>("headed");
    else
        this->headed = false;
    
    // If not started, initialize rclcpp
    if(!rclcpp::ok())
        rclcpp::init(0, nullptr);

    // Initialize the node and subscriber that receive forces data
    this->visualNode = std::make_shared<rclcpp::Node>(this->nodeName);
    if (this->headed)
        this->forcesSub = this->visualNode->create_subscription<gazebo_sfm_plugin::msg::Forces>(this->topicName, 10, std::bind(&VisualizeForcesPlugin::VisualizeForcesHSFM, this, std::placeholders::_1));
    else
        this->forcesSub = this->visualNode->create_subscription<gazebo_sfm_plugin::msg::Forces>(this->topicName, 10, std::bind(&VisualizeForcesPlugin::VisualizeForcesSFM, this, std::placeholders::_1));

    // Initialize forces lines
    this->global_force = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_LIST);
    this->global_force->setMaterial("Gazebo/Purple");
    this->global_force->setVisibilityFlags(GZ_VISIBILITY_ALL);

    this->obstacle_force = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_LIST);
    this->obstacle_force->setMaterial("Gazebo/Blue");
    this->obstacle_force->setVisibilityFlags(GZ_VISIBILITY_ALL);

    this->social_force = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_LIST);
    this->social_force->setMaterial("Gazebo/Yellow");
    this->social_force->setVisibilityFlags(GZ_VISIBILITY_ALL);

    this->desired_force = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_LIST);
    this->desired_force->setMaterial("Gazebo/Red");
    this->desired_force->setVisibilityFlags(GZ_VISIBILITY_ALL);

    this->group_force = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_LIST);
    this->group_force->setMaterial("Gazebo/Orange");
    this->group_force->setVisibilityFlags(GZ_VISIBILITY_ALL);

    this->linear_velocity = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_LIST);
    this->linear_velocity->setMaterial("Gazebo/Black");
    this->linear_velocity->setVisibilityFlags(GZ_VISIBILITY_ALL);

    // Set the visibility of our visual to TRUE
    this->visual->SetVisible(true);
    
    // Connect Render Gazebo update with our OnUpdate function
    this->connections.push_back(event::Events::ConnectRender(std::bind(&VisualizeForcesPlugin::OnUpdate, this)));
}

/////////////////////////////////////////////////
void VisualizeForcesPlugin::OnUpdate() {
    rclcpp::spin_some(this->visualNode);
}

/////////////////////////////////////////////////
void VisualizeForcesPlugin::VisualizeForcesHSFM(const gazebo_sfm_plugin::msg::Forces &msg) {
    // First we erase the previous points of each dynamic line
    this->global_force->Clear();
    this->obstacle_force->Clear();
    this->social_force->Clear();
    this->desired_force->Clear();
    this->group_force->Clear();
    this->linear_velocity->Clear();

    // Now we create the new lines
    this->visual_pose = this->visual->WorldPose();
    // Global force - it is already in the bodyframe reference system
    double glo_force_length = std::sqrt(std::pow(msg.global_force.x, 2) + std::pow(msg.global_force.y, 2));
    if(glo_force_length > 5) {    
        this->global_force->AddPoint(0, 0, 0); // origin pose of the visual element (attached to the actor)
        this->global_force->AddPoint(100 * msg.global_force.x / glo_force_length, 100 * msg.global_force.y / glo_force_length, 0); // force vector in the actor cartesian system
    }
    else {
        this->global_force->AddPoint(0, 0, 0);
        this->global_force->AddPoint(0, 0, 0);
    }
    // Obstacle force
    double obs_force_length = std::sqrt(std::pow(msg.obstacle_force.x, 2) + std::pow(msg.obstacle_force.y, 2));
    if(obs_force_length > 5) {    
        this->obstacle_force->AddPoint(0, 0, 0); // origin pose of the visual element (attached to the actor)
        double obs_x = std::cos(-this->visual_pose.Yaw()) * (msg.obstacle_force.x) - std::sin(-this->visual_pose.Yaw()) * (msg.obstacle_force.y);
        double obs_y = std::sin(-this->visual_pose.Yaw()) * (msg.obstacle_force.x) + std::cos(-this->visual_pose.Yaw()) * (msg.obstacle_force.y);
        this->obstacle_force->AddPoint(100 * obs_x / obs_force_length, 100 * obs_y / obs_force_length, 0); // force vector in the actor cartesian system
    }
    else {
        this->obstacle_force->AddPoint(0, 0, 0);
        this->obstacle_force->AddPoint(0, 0, 0);
    }
    // Desired force
    double des_force_length = std::sqrt(std::pow(msg.desired_force.x, 2) + std::pow(msg.desired_force.y, 2));
    if(des_force_length > 5) {    
        this->desired_force->AddPoint(0, 0, 0); // origin pose of the visual element (attached to the actor)
        double des_x = std::cos(-this->visual_pose.Yaw()) * (msg.desired_force.x) - std::sin(-this->visual_pose.Yaw()) * (msg.desired_force.y);
        double des_y = std::sin(-this->visual_pose.Yaw()) * (msg.desired_force.x) + std::cos(-this->visual_pose.Yaw()) * (msg.desired_force.y);
        this->desired_force->AddPoint(100 * des_x / des_force_length, 100 * des_y / des_force_length, 0); // force vector in the actor cartesian system
    }
    else {
        this->desired_force->AddPoint(0, 0, 0);
        this->desired_force->AddPoint(0, 0, 0);
    }
    // Social force
    double soc_force_length = std::sqrt(std::pow(msg.social_force.x, 2) + std::pow(msg.social_force.y, 2));
    if(soc_force_length > 5) {    
        this->social_force->AddPoint(0, 0, 0); // origin pose of the visual element (attached to the actor)
        double soc_x = std::cos(-this->visual_pose.Yaw()) * (msg.social_force.x) - std::sin(-this->visual_pose.Yaw()) * (msg.social_force.y);
        double soc_y = std::sin(-this->visual_pose.Yaw()) * (msg.social_force.x) + std::cos(-this->visual_pose.Yaw()) * (msg.social_force.y);
        this->social_force->AddPoint(100 * soc_x / soc_force_length, 100 * soc_y / soc_force_length, 0); // force vector in the actor cartesian system
    }
    else {
        this->social_force->AddPoint(0, 0, 0);
        this->social_force->AddPoint(0, 0, 0);
    }
    // Group force - it is already in the bodyframe reference system
    double gro_force_length = std::sqrt(std::pow(msg.group_force.x, 2) + std::pow(msg.group_force.y, 2));
    if(gro_force_length > 5) {    
        this->group_force->AddPoint(0, 0, 0); // origin pose of the visual element (attached to the actor)
        this->group_force->AddPoint(100 * msg.group_force.x / gro_force_length, 100 * msg.group_force.y / gro_force_length, 0); // force vector in the actor cartesian system
    }
    else {
        this->group_force->AddPoint(0, 0, 0);
        this->group_force->AddPoint(0, 0, 0);
    }
    // Linear velocity
    double vel_force_length = std::sqrt(std::pow(msg.linear_velocity.x, 2) + std::pow(msg.linear_velocity.y, 2));
    if(vel_force_length > 0.1) {    
        this->linear_velocity->AddPoint(0, 0, 0); // origin pose of the visual element (attached to the actor)
        double vel_x = std::cos(-this->visual_pose.Yaw()) * (msg.linear_velocity.x) - std::sin(-this->visual_pose.Yaw()) * (msg.linear_velocity.y);
        double vel_y = std::sin(-this->visual_pose.Yaw()) * (msg.linear_velocity.x) + std::cos(-this->visual_pose.Yaw()) * (msg.linear_velocity.y);
        this->linear_velocity->AddPoint(100 * vel_x / vel_force_length, 100 * vel_y / vel_force_length, 0); // force vector in the actor cartesian system
    }
    else {
        this->linear_velocity->AddPoint(0, 0, 0);
        this->linear_velocity->AddPoint(0, 0, 0);
    }
}

/////////////////////////////////////////////////
void VisualizeForcesPlugin::VisualizeForcesSFM(const gazebo_sfm_plugin::msg::Forces &msg) {
    // First we erase the previous points of each dynamic line
    this->global_force->Clear();
    this->obstacle_force->Clear();
    this->social_force->Clear();
    this->desired_force->Clear();
    this->group_force->Clear();
    this->linear_velocity->Clear();
    
    // Now we create the new lines
    this->visual_pose = this->visual->WorldPose();
    // Global force - it is already in the bodyframe reference system
    double glo_force_length = std::sqrt(std::pow(msg.global_force.x, 2) + std::pow(msg.global_force.y, 2));
    if(glo_force_length > 0.1) {    
        this->global_force->AddPoint(0, 0, 0); // origin pose of the visual element (attached to the actor)
        double glo_x = std::cos(-this->visual_pose.Yaw()) * (msg.global_force.x) - std::sin(-this->visual_pose.Yaw()) * (msg.global_force.y);
        double glo_y = std::sin(-this->visual_pose.Yaw()) * (msg.global_force.x) + std::cos(-this->visual_pose.Yaw()) * (msg.global_force.y);
        this->global_force->AddPoint(100 * glo_x / glo_force_length, 100 * glo_y / glo_force_length, 0); // force vector in the actor cartesian system
    }
    else {
        this->global_force->AddPoint(0, 0, 0);
        this->global_force->AddPoint(0, 0, 0);
    }
    // Obstacle force
    double obs_force_length = std::sqrt(std::pow(msg.obstacle_force.x, 2) + std::pow(msg.obstacle_force.y, 2));
    if(obs_force_length > 0.1) {    
        this->obstacle_force->AddPoint(0, 0, 0); // origin pose of the visual element (attached to the actor)
        double obs_x = std::cos(-this->visual_pose.Yaw()) * (msg.obstacle_force.x) - std::sin(-this->visual_pose.Yaw()) * (msg.obstacle_force.y);
        double obs_y = std::sin(-this->visual_pose.Yaw()) * (msg.obstacle_force.x) + std::cos(-this->visual_pose.Yaw()) * (msg.obstacle_force.y);
        this->obstacle_force->AddPoint(100 * obs_x / obs_force_length, 100 * obs_y / obs_force_length, 0); // force vector in the actor cartesian system
    }
    else {
        this->obstacle_force->AddPoint(0, 0, 0);
        this->obstacle_force->AddPoint(0, 0, 0);
    }
    // Desired force
    double des_force_length = std::sqrt(std::pow(msg.desired_force.x, 2) + std::pow(msg.desired_force.y, 2));
    if(des_force_length > 0.1) {    
        this->desired_force->AddPoint(0, 0, 0); // origin pose of the visual element (attached to the actor)
        double des_x = std::cos(-this->visual_pose.Yaw()) * (msg.desired_force.x) - std::sin(-this->visual_pose.Yaw()) * (msg.desired_force.y);
        double des_y = std::sin(-this->visual_pose.Yaw()) * (msg.desired_force.x) + std::cos(-this->visual_pose.Yaw()) * (msg.desired_force.y);
        this->desired_force->AddPoint(100 * des_x / des_force_length, 100 * des_y / des_force_length, 0); // force vector in the actor cartesian system
    }
    else {
        this->desired_force->AddPoint(0, 0, 0);
        this->desired_force->AddPoint(0, 0, 0);
    }
    // Social force
    double soc_force_length = std::sqrt(std::pow(msg.social_force.x, 2) + std::pow(msg.social_force.y, 2));
    if(soc_force_length > 0.1) {    
        this->social_force->AddPoint(0, 0, 0); // origin pose of the visual element (attached to the actor)
        double soc_x = std::cos(-this->visual_pose.Yaw()) * (msg.social_force.x) - std::sin(-this->visual_pose.Yaw()) * (msg.social_force.y);
        double soc_y = std::sin(-this->visual_pose.Yaw()) * (msg.social_force.x) + std::cos(-this->visual_pose.Yaw()) * (msg.social_force.y);
        this->social_force->AddPoint(100 * soc_x / soc_force_length, 100 * soc_y / soc_force_length, 0); // force vector in the actor cartesian system
    }
    else {
        this->social_force->AddPoint(0, 0, 0);
        this->social_force->AddPoint(0, 0, 0);
    }
    // Group force - it is already in the bodyframe reference system
    double gro_force_length = std::sqrt(std::pow(msg.group_force.x, 2) + std::pow(msg.group_force.y, 2));
    if(gro_force_length > 0.1) {    
        this->group_force->AddPoint(0, 0, 0); // origin pose of the visual element (attached to the actor)
        double gro_x = std::cos(-this->visual_pose.Yaw()) * (msg.group_force.x) - std::sin(-this->visual_pose.Yaw()) * (msg.group_force.y);
        double gro_y = std::sin(-this->visual_pose.Yaw()) * (msg.group_force.x) + std::cos(-this->visual_pose.Yaw()) * (msg.group_force.y);
        this->group_force->AddPoint(100 * gro_x / gro_force_length, 100 * gro_y / gro_force_length, 0); // force vector in the actor cartesian system
    }
    else {
        this->group_force->AddPoint(0, 0, 0);
        this->group_force->AddPoint(0, 0, 0);
    }
    // Linear velocity
    double vel_force_length = std::sqrt(std::pow(msg.linear_velocity.x, 2) + std::pow(msg.linear_velocity.y, 2));
    if(vel_force_length > 0.1) {    
        this->linear_velocity->AddPoint(0, 0, 0); // origin pose of the visual element (attached to the actor)
        double vel_x = std::cos(-this->visual_pose.Yaw()) * (msg.linear_velocity.x) - std::sin(-this->visual_pose.Yaw()) * (msg.linear_velocity.y);
        double vel_y = std::sin(-this->visual_pose.Yaw()) * (msg.linear_velocity.x) + std::cos(-this->visual_pose.Yaw()) * (msg.linear_velocity.y);
        this->linear_velocity->AddPoint(100 * vel_x / vel_force_length, 100 * vel_y / vel_force_length, 0); // force vector in the actor cartesian system
    }
    else {
        this->linear_velocity->AddPoint(0, 0, 0);
        this->linear_velocity->AddPoint(0, 0, 0);
    }
}