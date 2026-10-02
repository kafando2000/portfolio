#include "mm_behaviors/mm_moveit_actions.hpp"
#include <stdexcept>

BT::NodeStatus PlanForTheArm::onStart(){

    /* Process to acquire necessary data*/
    
    if (!node_ptr_) {
        std::cout << "ROS2 node not registered via init() method" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    if(getInput<bool>("is_container_full").value()){
        return BT::NodeStatus::FAILURE;
    }
    if(!getInput<double>("circle_radius",radius)){
        RCLCPP_INFO(node_ptr_->get_logger(),"radius not defined ! setting it to default value 0.3");
        radius = 0.3;
    }
   
    shift_amplitude = this->node_ptr_->get_parameter("theta_shift_amplitude").as_double();
   
    if (!getInput<std::string>("plan_key",plan_key_))
        return BT::NodeStatus::FAILURE;
    // for joint space planning
     
    if(plan_key_ == ""){
        return BT::NodeStatus::FAILURE;
    }
    if(plan_key_ == "arm_drop_prod_pose" || plan_key_ == "arm_initial_pose" || plan_key_ == "before_drop_prod_pose"){

        move_group->setStartStateToCurrentState();
        move_group->setNamedTarget(plan_key_);
        // Plan and execute the trajectory to that predefined state
        status_ = move_group->plan(my_plan);
        return BT::NodeStatus::RUNNING;
        

    }


    // get at most five of objects to avoid full copy
    bool data_valid = false;
    if(auto any_locked = getLockedPortContent("objects_poses_vect")){
            if(!any_locked->empty()){   
                std::vector<Pose3D>* Poses = any_locked->castPtr<std::vector<Pose3D>>();
                if(Poses && !Poses->empty()){
                    auto target_end = std::min<size_t>(5, (*Poses).size());
                    for(size_t i = 0;i<target_end;++i)
                        Objects_poses.push_back(Poses->at(i));
                    data_valid = true;
                }
            }
           
    }
    // if not pose 
    if(!data_valid){
        return BT::NodeStatus::FAILURE;
    }

    // get the robot model to perform move group start_state setting and also for nav2 
   
    if (!robot_model_){
      
        return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(this->node_ptr_->get_logger(), "first plan");
    getInput<double>("theta_shift",shift_theta);
   
    bool  tf_found = false;
     // geometry message
    geometry_msgs::msg::TransformStamped transformStamped;
    geometry_msgs::msg::TransformStamped transformStamped2;

    
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

   

    while ( !tf_found ) {    
        try {
           transformStamped = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
           transformStamped2 = tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
            tf_found = true;
        } catch (const tf2::TransformException & ex) {
           
        }
    }
   
    index = 0; 
   
        
    // get robot current pose in map frame 
    robot_pose_in_map.set__x(transformStamped.transform.translation.x);
    robot_pose_in_map.set__y(transformStamped.transform.translation.y);
    robot_pose_in_map.set__z(transformStamped.transform.translation.z);
    // get robot current pose in odom frame 
    robot_pose_in_odom.set__x(transformStamped2.transform.translation.x);
    robot_pose_in_odom.set__y(transformStamped2.transform.translation.y);
    robot_pose_in_odom.set__z(transformStamped2.transform.translation.z);

    // get near object
    index = get_near_pose_index(Objects_poses,robot_pose_in_map);
    object_pose = this->Objects_poses.at(index);

    geometry_msgs::msg::PointStamped object_map_pose;
    object_map_pose.header.frame_id = "map";
    object_map_pose.header.stamp = this->node_ptr_->now();
   
    object_map_pose.point.x = object_pose.x;
    object_map_pose.point.y = object_pose.y;
    object_map_pose.point.z = object_pose.z;
   
  
    tf_found = false;
    while ( !tf_found ) {    
        try {
            object_odom_pose = tf_buffer_->transform(object_map_pose,"odom");
            tf_found = true;
        } catch (const tf2::TransformException & ex) {
           
        }
    }
   
    



    /* Process to plan the arm*/

    // set the planning group
    // compute optimal postion and orientation to pick the object
    theta = atan2((object_pose.y-robot_pose_in_map.y),(object_pose.x-robot_pose_in_map.x))+shift_theta;
    theta2 = atan2((object_odom_pose.point.y-robot_pose_in_odom.y),(object_odom_pose.point.x-robot_pose_in_odom.x))+shift_theta;

    this->nav_target_pose.position.x =  object_map_pose.point.x - radius*(std::cos(theta));
    this->nav_target_pose.position.y  =  object_map_pose.point.y - radius*(std::sin(theta));
    this->nav_target_pose.position.z =  robot_pose_in_map.z;
    this->nav_target_pose.orientation  = theta;
     RCLCPP_INFO(this->node_ptr_->get_logger(), "theta : %f %f",theta,theta2);
    this->nav_target_in_odom.position.x = object_odom_pose.point.x - radius*(std::cos(theta2));
    this->nav_target_in_odom.position.y  = object_odom_pose.point.y - radius*(std::sin(theta2));
    this->nav_target_in_odom.position.z = robot_pose_in_odom.z;
    this->nav_target_in_odom.orientation  = theta2;
    tf2::Quaternion q1;
    q1.setRPY(0.0,0.0,theta);
    q1.normalize();
    tf2::Quaternion q2;
    q2.setRPY(0.0,0.0,theta2);
    q2.normalize();
   /* // 2. Set the virtual joint transform via Eigen
    Eigen::Isometry3d virtual_joint_transform = Eigen::Isometry3d::Identity();
    virtual_joint_transform.translation() << object_odom_pose.point.x, object_odom_pose.point.y,object_odom_pose.point.z; 

    // convert quaternion to rotation matrix
   
    
    tf2::Quaternion q2;
    q2.setRPY(0.0,0.0,theta2);
    q2.normalize();

    



    Eigen::Quaterniond eigen_quatern2(q2);
    virtual_joint_transform.linear() = eigen_quatern2.toRotationMatrix();
    // set virtual joint stateS
    start_state->setJointPositions("virtual_joint", virtual_joint_transform);
    // 3. Convert the core RobotState to a message type 
    moveit_msgs::msg::RobotState start_state_msg;
    moveit::core::robotStateToRobotStateMsg(*start_state, start_state_msg);
    // 4. Assign the state to the MoveGroup planner
    */
    

    //auto state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
   // state_->setToDefaultValues();
    //const moveit::core::JointModel* virtual_joint = state_->getJointModel("virtual_joint");

    
    //RCLCPP_INFO(this->node_ptr_->get_logger(),
    //        "Joint type: %s",
    //        virtual_joint->getTypeName().c_str());
    //RCLCPP_INFO(this->node_ptr_->get_logger(),
     //       "Variable count: %zu",
     //       virtual_joint->getVariableCount());

    //state_->setJointPositions(virtual_joint, {nav_target_in_odom.position.x,
    //    nav_target_in_odom.position.y
     //   ,theta2});

   //state_->update();
     
    /* bool valid = state_->satisfiesBounds();

    RCLCPP_INFO(this->node_ptr_->get_logger(), "Valid: %d", valid); */
   //state_->printStatePositions(std::cout); 
    
   // move_group->setStartState(*state_);
    
  

    //geometry_msgs::msg::Pose ee_target_pose;
    //std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::PoseStamped ee_target_pose;

    ee_target_pose.header.frame_id = "base_footprint";
    ee_target_pose.header.stamp = node_ptr_->now();


    ee_target_pose.pose.orientation.w = 1;
    ee_target_pose.pose.orientation.x = 0; 
    ee_target_pose.pose.orientation.y = 0;
    ee_target_pose.pose.orientation.z =  0; 

   
    ee_target_pose.pose.position.x = 0.193; 
    ee_target_pose.pose.position.y = 0.032;
    ee_target_pose.pose.position.z = 0.239;
    //Eigen::Isometry3d virtual_joint_transform ;

    //tf2::fromMsg(ee_target_pose,virtual_joint_transform);

   // const moveit::core::JointModelGroup* jmg = state_->getJointModelGroup("arm");   
    

    //waypoints.push_back(ee_target_pose);

    //const double jump_th = 0.0;
    //const double step = 0.01;
    
    move_group->setGoalPositionTolerance(0.001);
    move_group->setGoalOrientationTolerance(0.05);
    move_group->setWorkspace(
    -20.0, -20.0, 0.0,
     20.0,  20.0, 1.5); 
     RCLCPP_INFO(this->node_ptr_->get_logger(),
    "Planning frame: %s",
    move_group->getPlanningFrame().c_str());    
    bool result = move_group->setJointValueTarget(ee_target_pose,"grasping_frame");
   //bool result = move_group->setApproximateJointValueTarget(ee_target_pose,"grasping_frame");
    if(!result){
        RCLCPP_ERROR(this->node_ptr_->get_logger(), "IK not found !");
        return BT::NodeStatus::FAILURE;
    }
   // move_group->setPositionTarget(-0.022, 0.050, 0.291);

   
    move_group->setPlanningTime(5.0);
    move_group->setNumPlanningAttempts(5);

    status_ = move_group->plan(my_plan);
    
    return BT::NodeStatus::RUNNING;

}
BT::NodeStatus PlanForTheArm::onRunning(){
    
    if( status_ == moveit::core::MoveItErrorCode::SUCCESS){
        if(plan_key_ != "for_object"){
            setOutput("arm_joints_traject",my_plan.trajectory);
            return BT::NodeStatus::SUCCESS;
        }
        // forward blackboard data
        setOutput("base_target",nav_target_pose);
        setOutput("arm_joints_traject",my_plan.trajectory);
        shift_theta  = 0.0;
          // stop if reset shift_theta
        setOutput("theta_shift",0.0);
        // unstack object
        if(auto any_locked = getLockedPortContent("objects_poses_vect")){
            if(!any_locked->empty()){ 
                std::vector<Pose3D>* Poses = any_locked->castPtr<std::vector<Pose3D>>();
                if(Poses && (static_cast<int>(Poses->size()))>index){
                    RCLCPP_INFO(node_ptr_->get_logger(),"Successful deletion of object pose !");
                    Poses->erase(Poses->begin()+index);
                }
            }
        }
        return BT::NodeStatus::SUCCESS;
    }
    else{

        if(plan_key_ != "for_object"){
            RCLCPP_ERROR(this->node_ptr_->get_logger(), "Trajectory calculation failed");
            return BT::NodeStatus::FAILURE;
        }
        /*
        shift_theta += shift_amplitude;
        if(shift_theta <= (2*M_PI)){
            setOutput("theta_shift",shift_theta);
            RCLCPP_INFO(this->node_ptr_->get_logger(), "replanning with other configuration");
            return BT::NodeStatus::RUNNING;
        }
            */
        setOutput("theta_shift",0.0);
        // unstack object
        RCLCPP_ERROR(this->node_ptr_->get_logger(), "Trajectory calculation failed");
        if(auto any_locked = getLockedPortContent("objects_poses_vect")){
            if(!any_locked->empty()){ 
                std::vector<Pose3D>* Poses = any_locked->castPtr<std::vector<Pose3D>>();
                if(Poses && (static_cast<int>(Poses->size()))>index){
                    RCLCPP_INFO(node_ptr_->get_logger(),"Successful deletion of object pose !");
                    Poses->erase(Poses->begin()+index);
                }
            }
         }
        return BT::NodeStatus::FAILURE;  
    }
   
}

void PlanForTheArm::onHalted(){
    return ;
}


BT::NodeStatus ExecuteArmPlan::onStart(){
    if (!node_ptr_) {
        std::cout << "ROS2 node not registered via init() method" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    if(getInput<bool>("is_container_full").value()){
        return BT::NodeStatus::FAILURE;
    }
    if(!getInput<moveit_msgs::msg::RobotTrajectory>("arm_joints_traject",my_plan)){
        return BT::NodeStatus::FAILURE;
    }


    // send plan goal and check if okay
   /* auto status = move_group->asyncExecute(my_plan);
    if(status == moveit::core::MoveItErrorCode::SUCCESS){
       
        return BT::NodeStatus::FAILURE;
    }*/

    //auto goal_msg = moveit_msgs::action::MoveGroup::Goal();
    

    using namespace std::placeholders;
    client_ptr_ = rclcpp_action::create_client<ExecuteTrajectory>(node_ptr_,"/execute_trajectory");

    auto options = rclcpp_action::Client<moveit_msgs::action::ExecuteTrajectory>::SendGoalOptions();
    options.result_callback = std::bind(&ExecuteArmPlan::result_callback, this, _1);
    auto goal =moveit_msgs::action::ExecuteTrajectory::Goal();
    goal.trajectory = my_plan;
    
    is_over = false;
    auto future = client_ptr_->async_send_goal(goal,options);
    client_goal_handle = future.get();
   

    return BT::NodeStatus::RUNNING;

}

BT::NodeStatus ExecuteArmPlan::onRunning(){

    if (is_over) {
            if (moveit_result_ == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_ERROR(this->node_ptr_->get_logger(), "arm goal reached successfully !");
                ++products_counter;
                return BT::NodeStatus::SUCCESS;   
            } 
            RCLCPP_ERROR(this->node_ptr_->get_logger(), "arm goal was maybe cancelled !");
            return BT::NodeStatus::FAILURE;   
    }
    RCLCPP_ERROR(this->node_ptr_->get_logger(), "arm goal on execution ... ");
    return BT::NodeStatus::RUNNING;


}
void ExecuteArmPlan::onHalted(){
    if (!this->client_goal_handle) {
        RCLCPP_ERROR(this->node_ptr_->get_logger(), "Goal was rejected by server");
    }
    else {
        RCLCPP_INFO(this->node_ptr_->get_logger(), "Goal accepted! Handle captured.");
        this->client_ptr_->async_cancel_goal(client_goal_handle);
        client_goal_handle.reset();
    }
}

void ExecuteArmPlan::result_callback(const GoalHandleMoveit::WrappedResult& result){
    if (result.result){
        moveit_result_ = result.code;
        is_over = true;
    }
}

 void ExecuteArmPlan::read_text_in_speak(std::string t){
            espeak_Initialize(AUDIO_OUTPUT_PLAYBACK, 0, NULL, 0);
            espeak_Synth(
                t.c_str(),
                strlen(t.c_str()) + 1,
                0,
                POS_CHARACTER,
                0,
                espeakCHARS_AUTO,
                NULL,
                NULL);

            espeak_Synchronize();

         }
void PlanForTheArm::read_text_in_speak(std::string t){
            espeak_Initialize(AUDIO_OUTPUT_PLAYBACK, 0, NULL, 0);
            espeak_Synth(
                t.c_str(),
                strlen(t.c_str()) + 1,
                0,
                POS_CHARACTER,
                0,
                espeakCHARS_AUTO,
                NULL,
                NULL);

            espeak_Synchronize();

}