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
        // Debug prints
        // std::cout<<"Global Force: ["<<msg.global_force.x<<", "<<msg.global_force.y<<"]"<<std::endl;
        // std::cout<<"Global Force start point: "<<this->global_force->Point(0)<<std::endl;
        // std::cout<<"Global Force end point: "<<this->global_force->Point(1)<<std::endl;
        // std::cout<<"Global Force point count: "<<this->global_force->GetPointCount()<<std::endl;
        // std::cout<<"Visual rotation: ["<<this->visual->WorldPose().Roll()<<", "<<this->visual->WorldPose().Pitch()<<", "<<this->visual->WorldPose().Yaw()<<"]"<<std::endl;
        // std::cout<<"Global force length: "<<std::sqrt(std::pow(100 * msg.global_force.x / glo_force_length, 2) + std::pow(100 * msg.global_force.y / glo_force_length, 2))<<std::endl;
        // std::cout<<std::endl;
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
        double new_obs_length = std::sqrt(std::pow(obs_x, 2) + std::pow(obs_y, 2));
        this->obstacle_force->AddPoint(100 * obs_x / new_obs_length, 100 * obs_y / new_obs_length, 0); // force vector in the actor cartesian system
        // Debug prints
        // std::cout<<"Obstacle Force: ["<<msg.obstacle_force.x<<", "<<msg.obstacle_force.y<<"]"<<std::endl;
        // std::cout<<"Obstacle Force start point: "<<this->obstacle_force->Point(0)<<std::endl;
        // std::cout<<"Obstacle Force end point: "<<this->obstacle_force->Point(1)<<std::endl;
        // std::cout<<"Obstacle Force point count: "<<this->obstacle_force->GetPointCount()<<std::endl;
        // std::cout<<"Visual rotation: ["<<this->visual->WorldPose().Roll()<<", "<<this->visual->WorldPose().Pitch()<<", "<<this->visual->WorldPose().Yaw()<<"]"<<std::endl;
        // std::cout<<std::endl;
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
        double new_des_length = std::sqrt(std::pow(des_x, 2) + std::pow(des_y, 2));
        this->desired_force->AddPoint(100 * des_x / new_des_length, 100 * des_y / new_des_length, 0); // force vector in the actor cartesian system
        // Debug prints
        // std::cout<<"Desired Force: ["<<msg.desired_force.x<<", "<<msg.desired_force.y<<"]"<<std::endl;
        // std::cout<<"Desired Force start point: "<<this->desired_force->Point(0)<<std::endl;
        // std::cout<<"Desired Force end point: "<<this->desired_force->Point(1)<<std::endl;
        // std::cout<<"Desired Force point count: "<<this->desired_force->GetPointCount()<<std::endl;
        // std::cout<<"Visual rotation: ["<<this->visual->WorldPose().Roll()<<", "<<this->visual->WorldPose().Pitch()<<", "<<this->visual->WorldPose().Yaw()<<"]"<<std::endl;        std::cout<<"Length of Desired Force: "<<std::sqrt(std::pow(100 * des_x / des_force_length, 2) + std::pow(100 * des_y / des_force_length, 2))<<std::endl;
        // std::cout<<"Converted force length: "<<std::sqrt(std::pow(100 * des_x / new_des_length, 2) + std::pow(100 * des_y / new_des_length, 2))<<std::endl;
        // std::cout<<"X component: "<<des_x<<std::endl;
        // std::cout<<"Y component: "<<des_y<<std::endl;
        // std::cout<<"Actor position: ["<<this->visual_pose.X()<<", "<<this->visual_pose.Y()<<"]"<<std::endl;
        // std::cout<<std::endl;
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
        double new_soc_length = std::sqrt(std::pow(soc_x, 2) + std::pow(soc_y, 2));
        this->social_force->AddPoint(100 * soc_x / new_soc_length, 100 * soc_y / new_soc_length, 0); // force vector in the actor cartesian system
        // Debug prints
        // std::cout<<"Social Force: ["<<msg.social_force.x<<", "<<msg.social_force.y<<"]"<<std::endl;
        // std::cout<<"Social Force start point: "<<this->social_force->Point(0)<<std::endl;
        // std::cout<<"Social Force end point: "<<this->social_force->Point(1)<<std::endl;
        // std::cout<<"Social Force point count: "<<this->social_force->GetPointCount()<<std::endl;
        // std::cout<<"Visual rotation: ["<<this->visual->WorldPose().Roll()<<", "<<this->visual->WorldPose().Pitch()<<", "<<this->visual->WorldPose().Yaw()<<"]"<<std::endl;
        // std::cout<<std::endl;
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
        // Debug prints
        // std::cout<<"Group Force: ["<<msg.group_force.x<<", "<<msg.group_force.y<<"]"<<std::endl;
        // std::cout<<"Group Force start point: "<<this->group_force->Point(0)<<std::endl;
        // std::cout<<"Group Force end point: "<<this->group_force->Point(1)<<std::endl;
        // std::cout<<"Group Force point count: "<<this->group_force->GetPointCount()<<std::endl;
        // std::cout<<"Visual rotation: ["<<this->visual->WorldPose().Roll()<<", "<<this->visual->WorldPose().Pitch()<<", "<<this->visual->WorldPose().Yaw()<<"]"<<std::endl;
        // std::cout<<std::endl;
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
        double new_vel_length = std::sqrt(std::pow(vel_x, 2) + std::pow(vel_y, 2));
        this->linear_velocity->AddPoint(100 * vel_x / new_vel_length, 100 * vel_y / new_vel_length, 0); // force vector in the actor cartesian system
        // Debug prints
        // std::cout<<"Linear Velocity: ["<<msg.linear_velocity.x<<", "<<msg.linear_velocity.y<<"]"<<std::endl;
        // std::cout<<"Linear Velocity start point: "<<this->linear_velocity->Point(0)<<std::endl;
        // std::cout<<"Linear Velocity end point: "<<this->linear_velocity->Point(1)<<std::endl;
        // std::cout<<"Linear Velocity point count: "<<this->linear_velocity->GetPointCount()<<std::endl;
        // std::cout<<"Visual rotation: ["<<this->visual->WorldPose().Roll()<<", "<<this->visual->WorldPose().Pitch()<<", "<<this->visual->WorldPose().Yaw()<<"]"<<std::endl;
        // std::cout<<std::endl;
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
    this->linear_velocity->Clear(); // Not implemented
    
    // Now we create the new lines
    this->visual_pose = this->visual->WorldPose();
    // Global force - it is already in the bodyframe reference system
    double glo_force_length = std::sqrt(std::pow(msg.global_force.x, 2) + std::pow(msg.global_force.y, 2));
    if(glo_force_length > 0.1) {    
        this->global_force->AddPoint(0, 0, 0); // origin pose of the visual element (attached to the actor)
        double glo_x = std::cos(-this->visual_pose.Yaw()) * (msg.global_force.x) - std::sin(-this->visual_pose.Yaw()) * (msg.global_force.y);
        double glo_y = std::sin(-this->visual_pose.Yaw()) * (msg.global_force.x) + std::cos(-this->visual_pose.Yaw()) * (msg.global_force.y);
        double new_glo_length = std::sqrt(std::pow(glo_x, 2) + std::pow(glo_y, 2));
        this->global_force->AddPoint(100 * glo_x / new_glo_length, 100 * glo_y / new_glo_length, 0); // force vector in the actor cartesian system
        // Debug prints
        // std::cout<<"Global Force: ["<<msg.global_force.x<<", "<<msg.global_force.y<<"]"<<std::endl;
        // std::cout<<"Global Force start point: "<<this->global_force->Point(0)<<std::endl;
        // std::cout<<"Global Force end point: "<<this->global_force->Point(1)<<std::endl;
        // std::cout<<"Global Force point count: "<<this->global_force->GetPointCount()<<std::endl;
        // std::cout<<"Visual rotation: ["<<this->visual->WorldPose().Roll()<<", "<<this->visual->WorldPose().Pitch()<<", "<<this->visual->WorldPose().Yaw()<<"]"<<std::endl;
        // std::cout<<"Global force length: "<<std::sqrt(std::pow(100 * msg.global_force.x / glo_force_length, 2) + std::pow(100 * msg.global_force.y / glo_force_length, 2))<<std::endl;
        // std::cout<<std::endl;
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
        double new_obs_length = std::sqrt(std::pow(obs_x, 2) + std::pow(obs_y, 2));
        this->obstacle_force->AddPoint(100 * obs_x / new_obs_length, 100 * obs_y / new_obs_length, 0); // force vector in the actor cartesian system
        // Debug prints
        // std::cout<<"Obstacle Force: ["<<msg.obstacle_force.x<<", "<<msg.obstacle_force.y<<"]"<<std::endl;
        // std::cout<<"Obs x: "<<obs_x<<std::endl;
        // std::cout<<"Obs y: "<<obs_y<<std::endl;
        // std::cout<<"Obstacle Force start point: "<<this->obstacle_force->Point(0)<<std::endl;
        // std::cout<<"Obstacle Force end point: "<<this->obstacle_force->Point(1)<<std::endl;
        // std::cout<<"Obstacle Force point count: "<<this->obstacle_force->GetPointCount()<<std::endl;
        // std::cout<<"Visual rotation: ["<<this->visual->WorldPose().Roll()<<", "<<this->visual->WorldPose().Pitch()<<", "<<this->visual->WorldPose().Yaw()<<"]"<<std::endl;
        // std::cout<<"Visual position: ["<<this->visual->WorldPose().X()<<", "<<this->visual->WorldPose().Y()<<", "<<this->visual->WorldPose().Z()<<"]"<<std::endl;
        // std::cout<<std::endl;
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
        double new_des_length = std::sqrt(std::pow(des_x, 2) + std::pow(des_y, 2));
        this->desired_force->AddPoint(100 * des_x / new_des_length, 100 * des_y / new_des_length, 0); // force vector in the actor cartesian system
        // Debug prints
        // std::cout<<"Desired Force: ["<<msg.desired_force.x<<", "<<msg.desired_force.y<<"]"<<std::endl;
        // std::cout<<"Desired Force start point: "<<this->desired_force->Point(0)<<std::endl;
        // std::cout<<"Desired Force end point: "<<this->desired_force->Point(1)<<std::endl;
        // std::cout<<"Desired Force point count: "<<this->desired_force->GetPointCount()<<std::endl;
        // std::cout<<"Visual rotation: ["<<this->visual->WorldPose().Roll()<<", "<<this->visual->WorldPose().Pitch()<<", "<<this->visual->WorldPose().Yaw()<<"]"<<std::endl;        std::cout<<"Length of Desired Force: "<<std::sqrt(std::pow(100 * des_x / des_force_length, 2) + std::pow(100 * des_y / des_force_length, 2))<<std::endl;
        // std::cout<<"Converted force length: "<<std::sqrt(std::pow(100 * des_x / new_des_length, 2) + std::pow(100 * des_y / new_des_length, 2))<<std::endl;
        // std::cout<<"X component: "<<des_x<<std::endl;
        // std::cout<<"Y component: "<<des_y<<std::endl;
        // std::cout<<"Actor position: ["<<this->visual_pose.X()<<", "<<this->visual_pose.Y()<<"]"<<std::endl;
        // std::cout<<std::endl;
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
        double new_soc_length = std::sqrt(std::pow(soc_x, 2) + std::pow(soc_y, 2));
        this->social_force->AddPoint(100 * soc_x / new_soc_length, 100 * soc_y / new_soc_length, 0); // force vector in the actor cartesian system
        // Debug prints
        // std::cout<<"Social Force: ["<<msg.social_force.x<<", "<<msg.social_force.y<<"]"<<std::endl;
        // std::cout<<"Social Force start point: "<<this->social_force->Point(0)<<std::endl;
        // std::cout<<"Social Force end point: "<<this->social_force->Point(1)<<std::endl;
        // std::cout<<"Social Force point count: "<<this->social_force->GetPointCount()<<std::endl;
        // std::cout<<"Visual rotation: ["<<this->visual->WorldPose().Roll()<<", "<<this->visual->WorldPose().Pitch()<<", "<<this->visual->WorldPose().Yaw()<<"]"<<std::endl;
        // std::cout<<std::endl;
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
        double new_gro_length = std::sqrt(std::pow(gro_x, 2) + std::pow(gro_y, 2));
        this->group_force->AddPoint(100 * gro_x / new_gro_length, 100 * gro_y / new_gro_length, 0); // force vector in the actor cartesian system
        // Debug prints
        // std::cout<<"Group Force: ["<<msg.group_force.x<<", "<<msg.group_force.y<<"]"<<std::endl;
        // std::cout<<"Group Force start point: "<<this->group_force->Point(0)<<std::endl;
        // std::cout<<"Group Force end point: "<<this->group_force->Point(1)<<std::endl;
        // std::cout<<"Group Force point count: "<<this->group_force->GetPointCount()<<std::endl;
        // std::cout<<"Visual rotation: ["<<this->visual->WorldPose().Roll()<<", "<<this->visual->WorldPose().Pitch()<<", "<<this->visual->WorldPose().Yaw()<<"]"<<std::endl;
        // std::cout<<std::endl;
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
        double new_vel_length = std::sqrt(std::pow(vel_x, 2) + std::pow(vel_y, 2));
        this->linear_velocity->AddPoint(100 * vel_x / new_vel_length, 100 * vel_y / new_vel_length, 0); // force vector in the actor cartesian system
        // Debug prints
        // std::cout<<"Linear Velocity: ["<<msg.linear_velocity.x<<", "<<msg.linear_velocity.y<<"]"<<std::endl;
        // std::cout<<"Linear Velocity start point: "<<this->linear_velocity->Point(0)<<std::endl;
        // std::cout<<"Linear Velocity end point: "<<this->linear_velocity->Point(1)<<std::endl;
        // std::cout<<"Linear Velocity point count: "<<this->linear_velocity->GetPointCount()<<std::endl;
        // std::cout<<"Visual rotation: ["<<this->visual->WorldPose().Roll()<<", "<<this->visual->WorldPose().Pitch()<<", "<<this->visual->WorldPose().Yaw()<<"]"<<std::endl;
        // std::cout<<std::endl;
    }
    else {
        this->linear_velocity->AddPoint(0, 0, 0);
        this->linear_velocity->AddPoint(0, 0, 0);
    }
}