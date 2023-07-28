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
    
    // If not started, initialize rclcpp
    if(!rclcpp::ok())
        rclcpp::init(0, nullptr);

    // Initialize the node and subscriber that receive forces data
    this->visualNode = std::make_shared<rclcpp::Node>(this->nodeName);
    this->forcesSub = this->visualNode->create_subscription<gazebo_sfm_plugin::msg::Forces>(this->topicName, 10, std::bind(&VisualizeForcesPlugin::VisualizeForces, this, std::placeholders::_1));

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
void VisualizeForcesPlugin::VisualizeForces(const gazebo_sfm_plugin::msg::Forces &msg) {
    // First we erase the previous points of each dynamic line
    this->global_force->Clear();
    this->obstacle_force->Clear();
    this->social_force->Clear();
    this->desired_force->Clear();
    this->group_force->Clear();
    this->linear_velocity->Clear(); // Linear velocity is already expressed in the global reference frame
    
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
        double obs_x = std::cos(-this->visual_pose.Yaw()) * (msg.obstacle_force.x - this->visual_pose.X()) - std::sin(-this->visual_pose.Yaw()) * (msg.obstacle_force.y - this->visual_pose.Y());
        double obs_y = std::sin(-this->visual_pose.Yaw()) * (msg.obstacle_force.x - this->visual_pose.X()) + std::cos(-this->visual_pose.Yaw()) * (msg.obstacle_force.y - this->visual_pose.Y());
        this->obstacle_force->AddPoint(100 * obs_x / obs_force_length, 100 * obs_y / obs_force_length, 0); // force vector in the actor cartesian system
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
        double des_x = std::cos(-this->visual_pose.Yaw()) * (msg.desired_force.x - this->visual_pose.X()) - std::sin(-this->visual_pose.Yaw()) * (msg.desired_force.y - this->visual_pose.Y());
        double des_y = std::sin(-this->visual_pose.Yaw()) * (msg.desired_force.x - this->visual_pose.X()) + std::cos(-this->visual_pose.Yaw()) * (msg.desired_force.y - this->visual_pose.Y());
        this->desired_force->AddPoint(100 * des_x / des_force_length, 100 * des_y / des_force_length, 0); // force vector in the actor cartesian system
        // Debug prints
        // std::cout<<"Desired Force: ["<<msg.desired_force.x<<", "<<msg.desired_force.y<<"]"<<std::endl;
        // std::cout<<"Desired Force start point: "<<this->desired_force->Point(0)<<std::endl;
        // std::cout<<"Desired Force end point: "<<this->desired_force->Point(1)<<std::endl;
        // std::cout<<"Desired Force point count: "<<this->desired_force->GetPointCount()<<std::endl;
        // std::cout<<"Visual rotation: ["<<this->visual->WorldPose().Roll()<<", "<<this->visual->WorldPose().Pitch()<<", "<<this->visual->WorldPose().Yaw()<<"]"<<std::endl;
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
        double soc_x = std::cos(-this->visual_pose.Yaw()) * (msg.social_force.x - this->visual_pose.X()) - std::sin(-this->visual_pose.Yaw()) * (msg.social_force.y - this->visual_pose.Y());
        double soc_y = std::sin(-this->visual_pose.Yaw()) * (msg.social_force.x - this->visual_pose.X()) + std::cos(-this->visual_pose.Yaw()) * (msg.social_force.y - this->visual_pose.Y());
        this->social_force->AddPoint(100 * soc_x / soc_force_length, 100 * soc_y / soc_force_length, 0); // force vector in the actor cartesian system
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
}