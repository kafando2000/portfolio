#include "mm_behaviors/mm_moveit_bt_nodes.hpp"


namespace WSTTA{
/* Implementation of the arm plannning class' methods */
// constructor
PlanForTheArm::PlanForTheArm(const std::string& name, const BT::NodeConfig& config, 
    const rclcpp::Node::SharedPtr& node_ptr, 
    const std::shared_ptr<moveit_cpp::MoveItCpp>& moveit_cpp_ptr)
    :StatefulActionNode(name,config), node_ptr_{node_ptr},moveit_cpp_ptr{moveit_cpp_ptr}

{
        RCLCPP_INFO(this->node_ptr_->get_logger(), "Loading arm!");
        arm_planning_component = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_ARM_GROUP_NAME, moveit_cpp_ptr);

        robot_model_ = moveit_cpp_ptr->getRobotModel();
        planning_frame = robot_model_->getModelFrame();
        plan_key_ = "";
}

// get object pose and station index
bool PlanForTheArm::get_plan_object_info(){
    if(!getInput<PlanObjectInfo>("plan_object_info",plan_object_info_)){
        return false;
    }
    return true;
}

// bt onStart virtual function 
BT::NodeStatus PlanForTheArm::onStart(){
    // resource check
    if(!node_ptr_) {
        std::cout << "ROS2 node not registered via init() method" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    
    if(getInput<bool>("is_container_full").value()){
        return BT::NodeStatus::FAILURE;
    }

    // some information for next process
    navigation_frame = this->node_ptr_->get_parameter("navigation_frame").as_string();
    arm_predefined_states = this->node_ptr_->get_parameter("arm_predefined_states").as_string_array();
   
    if (!getInput<std::string>("plan_key",plan_key_)){
        return BT::NodeStatus::FAILURE;
    }
    // for joint space planning
    if(plan_key_ == ""){
        return BT::NodeStatus::FAILURE;
    }
    found_key = false;
    // query group names and check if one matches plan_key
    for(auto t:arm_predefined_states){
        if(t == plan_key_){
             found_key =  true;
        }
    }
    //state_ = arm_planning_component->getStartState();

    // plan for the joint space already defined sates
    if(found_key){
        RCLCPP_INFO(node_ptr_->get_logger(),"Okay there !");
        arm_planning_component->setStartStateToCurrentState();
        arm_planning_component->setGoal(plan_key_);
        plan_solution = arm_planning_component->plan();
        return BT::NodeStatus::RUNNING;   
    }
    
    if(plan_key_ != "for_object"){
            return BT::NodeStatus::FAILURE;

    }
  
    // get the robot model to perform  start_state setting and also for nav2 
   
    if (!robot_model_){
      
        return BT::NodeStatus::FAILURE;
    }
    if(!this->get_plan_object_info()){
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;

}


// bt onRunning  virtual function 
BT::NodeStatus PlanForTheArm::onRunning(){
    // immediately return if it is in predifined states
    if(plan_key_ != "for_object"){
        if(plan_solution){
            setOutput("arm_joints_traject",plan_solution.trajectory);
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }

    // arm planning to grasp object poses
    if(getInput<bool>("is_container_full").value()){
        return BT::NodeStatus::FAILURE;
    }
    

    arm_planning_component->setWorkspace(-10.0, -10.0, 0.0, 10.0, 10.0, 1.5);
    // set virtual joint future pose
    /*
     auto joint_model = state_->getJointModel("virtual_joint");
    auto jmg = state_->getJointModelGroup("arm");
    state_->setJointPositions(joint_model,{nav_target_in_odom.position.x,
        nav_target_in_odom.position.y,theta2});
    Eigen::VectorXd arm_joints_initials(4);
    for(int i=0;i<4;i++){
        arm_joints_initials(i) = 0.000000;
    }
    state_->setJointGroupPositions(jmg,arm_joints_initials);
    state_->update();
    
    arm_planning_component->setStartState(*state_);
    //arm_planning_component->setStartStateToCurrentState();
    
    
    const kinematics::KinematicsBaseConstPtr& solver =
    jmg->getSolverInstance();
    if (solver)
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "IK solver loaded: %s",
                    solver->getBaseFrame().c_str());
    }
    else
    {
        RCLCPP_ERROR(node_ptr_->get_logger(), "No IK solver loaded");
    }
    state_ = arm_planning_component->getStartState();
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    auto planning_scene = moveit_cpp_ptr->getPlanningSceneMonitor()->getPlanningScene();
    planning_scene->checkCollision(collision_request,collision_result);
    auto col = collision_result.collision ? "in" : "not in";
    RCLCPP_INFO(node_ptr_->get_logger(),"Test 2: Current state is %s collision" , col );
    
    */
   
    //auto ss = *(state_);
    //ss.printStatePositions(std::cout);
    //bool x = state_->setFromIK(jmg,ee_target_pose2,"grasping_frame",2.0);
    //if(x){
    //    RCLCPP_INFO(this->node_ptr_->get_logger(), "IK found is okay!");
    //}
    //else{
    //    RCLCPP_INFO(this->node_ptr_->get_logger(), "IK found is not okay!");
    //}
    //ss = *(state_);
    //ss.printStatePositions(std::cout);
    moveit_cpp::PlanningComponent::PlanRequestParameters plan_params;
    plan_params.planner_id = "RRTConnectkConfigDefault";
    plan_params.planning_pipeline = "ompl"; 
    plan_params.planning_time = 2.0;
    plan_params.max_acceleration_scaling_factor = 0.4;
    plan_params.max_velocity_scaling_factor = 0.4;
    std::vector<double> tolerance_position(3, 0.01);
    std::vector<double> tolerance_orientation(3, 0.9);

    geometry_msgs::msg::PoseStamped ee_target_pose_near;
    ee_target_pose_near = plan_object_info_.pose;
    ee_target_pose_near.pose.position.z = plan_object_info_.pose.pose.position.z + 0.05;

    geometry_msgs::msg::PoseStamped ee_target_pose = plan_object_info_.pose;

    // 4. Construct the goal constraints message
     moveit_msgs::msg::Constraints goal_constraints = 
        kinematic_constraints::constructGoalConstraints("grasping_frame",
            ee_target_pose_near, tolerance_position, tolerance_orientation);


    //auto request = arm_planning_component->getMotionPlanRequest(plan_params);

    /*
    auto t = request.workspace_parameters.max_corner.x;
    auto t1 = request.workspace_parameters.max_corner.y;
    auto t2 = request.workspace_parameters.max_corner.z;
    auto t3 = request.workspace_parameters.min_corner.x;
    auto t4 = request.workspace_parameters.min_corner.y;
    auto t5 = request.workspace_parameters.min_corner.z;
    */
    //RCLCPP_INFO(this->node_ptr_->get_logger(), "(%f,%f,%f),(%f,%f,%f)",t3,t4,t5,t,t1,t2);
    // arm_planning_component->setGoal(ee_target_pose_near,"grasping_frame");
   
    arm_planning_component->setGoal({goal_constraints});
    RCLCPP_INFO(this->node_ptr_->get_logger(), "computing solution 1 !");
    auto solution1 = arm_planning_component->plan(plan_params);

    if(solution1){
        RCLCPP_INFO(this->node_ptr_->get_logger(), "solution 1 is okay!");
        moveit::core::RobotState solution1_end = solution1.trajectory->getLastWayPoint();
         moveit_msgs::msg::Constraints goal_constraint2 = 
        kinematic_constraints::constructGoalConstraints("grasping_frame",
             ee_target_pose, tolerance_position, tolerance_orientation);
        arm_planning_component->setStartState(solution1_end);
        arm_planning_component->setGoal({goal_constraint2});
       // arm_planning_component->setGoal(ee_target_pose,"grasping_frame");
        auto solution2 = arm_planning_component->plan(plan_params);
        if(solution2){
            
            RCLCPP_INFO(this->node_ptr_->get_logger(), "Trajectory generated successfully!");
            // combine two trajectory approach and very target trajectories
            robot_trajectory::RobotTrajectory combined_trajectory(*solution1.trajectory);
            double dt = 0.02; 
            combined_trajectory.append(*solution2.trajectory, dt);
            auto trajectory_ptr = std::make_shared<robot_trajectory::RobotTrajectory>(combined_trajectory);
            setOutput("arm_joints_traject",trajectory_ptr);
            //setOutput("base_target",nav_target_pose);
            return BT::NodeStatus::SUCCESS;

            
        }
    }

   return BT::NodeStatus::FAILURE;

}

// bt onHlated virtual function 
void PlanForTheArm::onHalted(){
    return;
}


/* Implementation of the arm trajectory execution class' methods */

ExecuteArmPlan::ExecuteArmPlan(const std::string& name, const BT::NodeConfig& config, const rclcpp::Node::SharedPtr& node_ptr,
            const std::shared_ptr<moveit_cpp::MoveItCpp>& moveit_cpp_ptr):StatefulActionNode(name,config),
        node_ptr_{node_ptr},moveit_cpp_ptr{moveit_cpp_ptr}
         {
           arm_planning_component = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_ARM_GROUP_NAME, moveit_cpp_ptr);
         }

BT::NodeStatus ExecuteArmPlan::onStart(){
    if(!node_ptr_) {
        std::cout << "ROS2 node not registered via init() method" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    if(!getInput("arm_joints_traject",my_plan)){
        RCLCPP_INFO(this->node_ptr_->get_logger(), "Trajectory not found!");
        return BT::NodeStatus::FAILURE;
    }
    if(!getInput("exec_key",exec_key)){
        RCLCPP_INFO(this->node_ptr_->get_logger(), "key not found not found!");
        return BT::NodeStatus::FAILURE;
    }
    if(exec_key ==  "to_object_pose"){
        arm_planning_component->setGoal("arm_initial_pose");
        auto plan =arm_planning_component->plan();
        if(plan)
             moveit_cpp_ptr->execute(plan.trajectory,CONTROLLERS);
    }
    execution_future_ = std::async(std::launch::async, [this](){
        return moveit_cpp_ptr->execute(my_plan, CONTROLLERS);
    });
    // immediately start gripper plan and execution
   
    return BT::NodeStatus::RUNNING;

}

BT::NodeStatus ExecuteArmPlan::onRunning(){
   
    auto fut = execution_future_.wait_for(std::chrono::milliseconds(0));
    
    if (fut == std::future_status::ready) {
        // Retrieve the result from the thread
        status = execution_future_.get();

        if (status == moveit_controller_manager::ExecutionStatus::SUCCEEDED) {
             RCLCPP_INFO(this->node_ptr_->get_logger(), "Trajectory completed successfully!");
            return BT::NodeStatus::SUCCESS;
        } else {
             RCLCPP_INFO(this->node_ptr_->get_logger(), "Trajectory execution failed!");
            return BT::NodeStatus::FAILURE;
        }
    }
     
    return BT::NodeStatus::RUNNING;
}

void ExecuteArmPlan::onHalted(){
    manager = moveit_cpp_ptr->getTrajectoryExecutionManagerNonConst();
    // cancel execution if node goes to idly or if something else happen and seen as necessary
    if(manager){
        manager->stopExecution(true);
        return;
    }

}


PlanExecForGripper::PlanExecForGripper(const std::string& name, const BT::NodeConfig& config, const rclcpp::Node::SharedPtr& node_ptr,
            const std::shared_ptr<moveit_cpp::MoveItCpp>& moveit_cpp_ptr):StatefulActionNode(name,config),
        node_ptr_{node_ptr},moveit_cpp_ptr{moveit_cpp_ptr}
         {
           ee_planning_component = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_EE_GROUP_NAME, moveit_cpp_ptr);
           ee_predefined_states = node_ptr_->get_parameter("ee_predefined_states").as_string_array();
         }

BT::NodeStatus  PlanExecForGripper::onStart(){
    if(!node_ptr_) {
        std::cout << "ROS2 node not registered via init() method" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    if(!getInput("plan_key",plan_key_)){
        return BT::NodeStatus::FAILURE;

    }

    auto pose_found = false;
    for(auto t:ee_predefined_states){
        if(t == plan_key_){
            pose_found = true;
        }
    }
    if(!pose_found){
        return BT::NodeStatus::FAILURE;
    }
     moveit_cpp::PlanningComponent::PlanRequestParameters plan_params;

    // Set custom Planner ID (must match your ompl_planning.yaml config name)
    plan_params.planner_id = "RRTConnectkConfigDefault";

    // Optional: You can also explicitly set the pipeline ID if using multiple pipelines
    plan_params.planning_pipeline = "ompl"; 
    plan_params.planning_time = 2.0;
    plan_params.max_acceleration_scaling_factor = 0.9;
    plan_params.max_velocity_scaling_factor = 0.9;

    ee_planning_component->setStartStateToCurrentState();
    ee_planning_component->setGoal(plan_key_);

    my_plan = ee_planning_component->plan(plan_params);
    if(!my_plan){
       RCLCPP_INFO(this->node_ptr_->get_logger(), "planning for gripper unsuccessful!");
    }
    
    execution_future_ = std::async(std::launch::async, [this](){
        return moveit_cpp_ptr->execute(my_plan.trajectory, CONTROLLERS);
       });
    return BT::NodeStatus::RUNNING;
    
}
BT::NodeStatus  PlanExecForGripper::onRunning(){
    manager = moveit_cpp_ptr->getTrajectoryExecutionManagerNonConst();
    auto fut = execution_future_.wait_for(std::chrono::milliseconds(0));
    
    if (fut == std::future_status::ready) {
        // Retrieve the result from the thread
        status = execution_future_.get();

        if (status == moveit_controller_manager::ExecutionStatus::SUCCEEDED) {
             RCLCPP_INFO(this->node_ptr_->get_logger(), "Trajectory completed successfully!");
            return BT::NodeStatus::SUCCESS;
        } else {
             RCLCPP_INFO(this->node_ptr_->get_logger(), "Trajectory execution failed!");
            return BT::NodeStatus::FAILURE;
        }
    }
    else{
        if(plan_key_ == "grasp"){
           if(getInput<bool>("is_grasped").value()){
                if(manager){
                    manager->stopExecution(true);
                }
                return BT::NodeStatus::SUCCESS; 
            }
        }
        return BT::NodeStatus::RUNNING;
    }
    

}

void PlanExecForGripper::onHalted(){
   
                // cancel execution if node goes to idle or if something else happen and seen as necessary
    if(manager){
        manager->stopExecution(true);
    }
}


} // end of namespace WSTTA