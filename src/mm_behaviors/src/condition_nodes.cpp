#include "mm_behaviors/condition_nodes.hpp"
namespace WSTTA{
BatteryChecker::BatteryChecker(std::string name,const BT::NodeConfig& config,const rclcpp::Node::SharedPtr& node)
:ConditionNode(name,config),node{node}{
        
       
}

bool BatteryChecker::battery_is_ok(){
   /*  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10))    // History QoS (Keep last 10 messages)
                               .reliable()                      // Reliability QoS (reliable communication)
                               .transient_local(); 
    batterystatesubscriber_ = this->node->create_subscription<double>("/battery_state", qos_profile, 
            std::bind(&BatteryChecker::recieve_battery_state, this, std::placeholders::_1)
        ); */
    return (this->batterypercentage > this->percentage_treshold);
}

// bt  node tick
BT::NodeStatus BatteryChecker::tick(){
    getInput("percentage_treshold",this->percentage_treshold);
    getInput("battery_percentage",this->batterypercentage);
    if(this->battery_is_ok()){
        RCLCPP_INFO(this->node->get_logger(),"battery is okay");
        return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(this->node->get_logger(),"battery is low_");
    return BT::NodeStatus::FAILURE;
}

// CheckGoToInitials class member functions implementation
CheckGoToInitials::CheckGoToInitials(std::string name,const BT::NodeConfig& config,const rclcpp::Node::SharedPtr& node)
                        :ConditionNode(name,config),node{node}{}
// bt  node tick
BT::NodeStatus CheckGoToInitials::tick(){
    this->is_go_to_initials = getInput<bool>("is_go_to_initials").value();
    if(this->is_go_to_initials){
        RCLCPP_INFO(this->node->get_logger(),"Go to initials !");
        return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(this->node->get_logger(),"Not go to initials");
    return BT::NodeStatus::FAILURE;
}

// container checker 
ContainerChecker::ContainerChecker(std::string name,const BT::NodeConfig& config,const rclcpp::Node::SharedPtr& node)
:ConditionNode(name,config),node{node}{}

BT::NodeStatus ContainerChecker::tick(){
    this->is_container_full = getInput<bool>("is_container_full").value();
    if(this->is_container_full){
        RCLCPP_INFO(this->node->get_logger(),"container is full");
        return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(this->node->get_logger(),"container is not full");
    return BT::NodeStatus::FAILURE;
}






JustNavigateChecker::JustNavigateChecker(std::string name,const BT::NodeConfig& config,const rclcpp::Node::SharedPtr& node)
    :ConditionNode(name,config),node{node}{}


BT::NodeStatus JustNavigateChecker::tick(){

    if(this->is_just_navigate()){
        RCLCPP_INFO(this->node->get_logger(),"No object pose found !");
        return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(this->node->get_logger(),"object pose found ! Getting back to arm preplanning");
    return BT::NodeStatus::FAILURE;
}

bool JustNavigateChecker::is_just_navigate(){
    bool is_empty = true;
    if(auto any_locked = getLockedPortContent("objects_poses_vect")){
        if(!any_locked->empty()){ 
            std::vector<Pose3D>* Poses = any_locked->castPtr<std::vector<Pose3D>>();
            if(Poses && !Poses->empty()){
                is_empty = false;
            }
        }
    }
    return is_empty;
           

}
}// end of namespace WSTTA
