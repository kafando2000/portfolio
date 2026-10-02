#include "mm_behaviors/mm_nav_actions.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
namespace WSTTA{
int Navigate::get_near_pose_index(std::vector<Pose3D>& t,Pose3D& robot_pose){
            int index = 0;
            TargetPose rob{robot_pose};
            double min = (rob-t.at(0)).norm();
            for(auto k:t){
                auto dist = (rob-k).norm();
                if(min > dist){
                    min = dist;
                    ++index;
                }
            }
            return index;
}

bool Navigate::check_if_target(TargetPose& t,std::string var_name){
   
    if(getInput<TargetPose>(var_name,t)){
        return true;
    }
    return false;
}


BT::NodeStatus Navigate::onStart() {
    // Validate that a node exists
   

    if (!node_ptr_) {
        std::cout << "ROS2 node not registered via init() method" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    // create nav2 action client
    using namespace std::placeholders;
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&Navigate::result_callback, this, _1);
    client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "/navigate_to_pose");
    navigation_frame = this->node_ptr_->get_parameter("navigation_frame").as_string();
    base_frame = this->node_ptr_->get_parameter("base_frame").as_string();
    
    if(!getInput<std::string>("key",key))
        return BT::NodeStatus::FAILURE;

    // process inputs and also insure the priority
    if(getInput<bool>("is_container_full").value()){
        if(this->key != "unload")
            return BT::NodeStatus::FAILURE;
        if (!this->check_if_target(this->target_pose,"unload_poses"))
                return BT::NodeStatus::FAILURE;

        RCLCPP_INFO(this->node_ptr_->get_logger(), "planning to go to unloading station !");
        
    }
    else{
        
        if(this->key != "initial"){
        // if no order to go to initial check for the key
            if (key == "in_track_object"){
                  // just pick five of the object poses that are in the vector
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
                // check if there is pose or not
                if(!data_valid){
                    return BT::NodeStatus::FAILURE;
                }
                RCLCPP_INFO(this->node_ptr_->get_logger(), "first plan");
                getInput<double>("theta_shift",shift_theta);

                if(!getInput<double>("circle_radius",radius)){
                    RCLCPP_INFO(node_ptr_->get_logger(),"radius not defined ! setting it to default value 0.3");
                    radius = 0.3;
                }
                        
                // geometry message
                geometry_msgs::msg::TransformStamped transformStamped;

                
                tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

                bool  tf_found = false;
                // loook for transforms
                while ( !tf_found ) {    
                    try {
                    transformStamped = tf_buffer_->lookupTransform(navigation_frame, base_frame, tf2::TimePointZero);
                        tf_found = true;
                    } catch (const tf2::TransformException & ex) {
                    
                    }
                }
                // get robot current pose in map frame 
                robot_pose_in_map.set__x(transformStamped.transform.translation.x);
                robot_pose_in_map.set__y(transformStamped.transform.translation.y);
                robot_pose_in_map.set__z(transformStamped.transform.translation.z);

                // get near object and clear the vector
                index = get_near_pose_index(Objects_poses,robot_pose_in_map);
                object_pose = this->Objects_poses.at(index);
                Objects_poses.clear();

                object_map_pose.header.frame_id = navigation_frame;
                object_map_pose.header.stamp = this->node_ptr_->now();
            
                object_map_pose.point.x = object_pose.x;
                object_map_pose.point.y = object_pose.y;
                object_map_pose.point.z = object_pose.z;
                // compute the goal pose for navigation
                theta = atan2((object_pose.y-robot_pose_in_map.y),(object_pose.x-robot_pose_in_map.x)) + shift_theta;
                target_pose.position.x = object_map_pose.point.x - radius*cos(theta);
                target_pose.position.y = object_map_pose.point.y - radius*sin(theta);
                target_pose.position.z = robot_pose_in_map.z;
                target_pose.orientation = theta;
                RCLCPP_INFO(this->node_ptr_->get_logger(), "planning to go near the object !");
            }
            // if it is just navigate check if there is a goal to navigate to 
            // if the first conditon work check if there is not an object pose to track if so cancel navigation
            else{ 
                
                if (!this->check_if_target(this->target_pose,"base_long_path_pose")){
                    return BT::NodeStatus::FAILURE;
                }
                RCLCPP_INFO(this->node_ptr_->get_logger(), "planning to take long path !");
            }
        }
        else{
            if(!this->check_if_target(this->target_pose,"base_initial_pose")){
                return BT::NodeStatus::FAILURE;
            }
            RCLCPP_INFO(this->node_ptr_->get_logger(), "planning to go to initial station !");
        }

    }
    
    // set goal pose and orientation
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = navigation_frame;
    goal_msg.pose.pose.position = target_pose.position;

    tf2::Quaternion q;
    q.setRPY(0, 0, this->target_pose.orientation);
    q.normalize();
    goal_msg.pose.pose.orientation = tf2::toMsg(q);

    // Send the navigation action goal.
    is_done = false;
    auto future = client_ptr_->async_send_goal(goal_msg, send_goal_options);
    this->client_goal_handle = future.get();
    std::cout << "[" << this->name() << "] Sent goal message" << std::endl;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Navigate::onRunning() {
    // If there is a result, we can check the status of the action directly.
    // Otherwise, the action is still running.
    if (is_done) {
        if (nav_result_ == rclcpp_action::ResultCode::SUCCEEDED) {
             RCLCPP_INFO(this->node_ptr_->get_logger(), "Great, goal reached !");
            if(key == "initial")
                setOutput("is_go_to_initials",false);
            if(key == "in_global_long_path")
                setOutput("is_go_to_initials",true);
            if(key == "unload"){
                setOutput("is_container_full",false);
            }
            if(key == "in_track_object"){
                tf2::Quaternion q;
                q.setRPY(0, 0, this->target_pose.orientation);
                q.normalize();
                plan_object_info_.index = index;
                plan_object_info_.pose.pose.position = object_map_pose.point;
                plan_object_info_.pose.header.frame_id = navigation_frame;
                plan_object_info_.pose.pose.orientation = tf2::toMsg(q);
                if(auto any_locked = getLockedPortContent("objects_poses_vect")){
                if(!any_locked->empty()){ 
                    std::vector<Pose3D>* Poses = any_locked->castPtr<std::vector<Pose3D>>();
                    RCLCPP_INFO(this->node_ptr_->get_logger(), "deletion now %d !",plan_object_info_.index);
                    if(Poses && ((static_cast<int>(Poses->size())) > plan_object_info_.index)){
                        RCLCPP_INFO(node_ptr_->get_logger(),"Successful deletion of object pose !");
                        Poses->erase(Poses->begin()+plan_object_info_.index);
                    }
                }
            }
                setOutput("plan_object_info",plan_object_info_);
            }
           
            return BT::NodeStatus::SUCCESS;   
        } 
        RCLCPP_ERROR(this->node_ptr_->get_logger(), "sorry, goal not reached !");
        return BT::NodeStatus::FAILURE;   
    }
    RCLCPP_INFO(this->node_ptr_->get_logger(), "Still in navigation !");
    return BT::NodeStatus::RUNNING;
}

void Navigate::result_callback(const GoalHandleNav::WrappedResult& result) {
    // If there is a result, we consider navigation completed and save the
    // result code to be checked in the `onRunning()` method.
    if (result.result) {
        is_done = true;
        nav_result_ = result.code;
    }
}
void Navigate::onHalted(){
    
    if (!this->client_goal_handle) {
        RCLCPP_ERROR(this->node_ptr_->get_logger(), "Goal was rejected by server");
    }
    if(!is_done) {
        RCLCPP_INFO(this->node_ptr_->get_logger(), "Goal accepted! Handle captured.");
        this->client_ptr_->async_cancel_goal(client_goal_handle);

    }

}
}// end of namespace WSTTA
