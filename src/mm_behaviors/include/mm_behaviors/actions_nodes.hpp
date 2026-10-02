#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "yaml-cpp/yaml.h"
#include "mm_behaviors/utils.hpp"




class  GetLongPathGoalPose : public BT::SyncActionNode{
public:
     GetLongPathGoalPose(std::string name,const BT::NodeConfig& config,const rclcpp::Node::SharedPtr& node):SyncActionNode(name,config),node_ptr_{node}{
    }
     BT::NodeStatus tick() override;

     static BT::PortsList providedPorts() {
        return { BT::InputPort<TargetPose>("base_long_path_pose")};
    }

private:
         TargetPose long_path;
         rclcpp::Node::SharedPtr node_ptr_;
};


class  StackObjectPose : public BT::SyncActionNode{
public:
    StackObjectPose(std::string name,const BT::NodeConfig& config,const rclcpp::Node::SharedPtr& node):SyncActionNode(name,config),node_ptr_{node}{
        
    }

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::vector<Pose3D>>("objects_poses_vect")};
    }

private:

     std::vector<Pose3D> Objects_poses;
     rclcpp::Node::SharedPtr node_ptr_;

}; 



class  UnLoad : public BT::StatefulActionNode{
public:
    UnLoad(std::string name,const BT::NodeConfig& config,const rclcpp::Node::SharedPtr& node):StatefulActionNode(name,config),node_ptr_{node}{
         
    }

    BT::NodeStatus onStart() override{
        if(getInput<bool>("is_container_full").value()){
            start_time = std::chrono::steady_clock::time_point();
            RCLCPP_INFO(this->node_ptr_->get_logger()," starting unloading  .. !");
            return BT::NodeStatus::RUNNING;
        }
        return BT::NodeStatus::FAILURE;
    };
    BT::NodeStatus onRunning() override{
        current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
        if (elapsed >= 61) {
            RCLCPP_INFO(this->node_ptr_->get_logger(),"unloaded finished successfully !");
            setOutput("is_container_full",false);
            return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_INFO(this->node_ptr_->get_logger()," still on unloading  .. !");
        return BT::NodeStatus::RUNNING;
    }
    void onHalted() override{
        return;
    };

    static BT::PortsList providedPorts() {
        return { BT::BidirectionalPort<bool>("is_container_full")};
    }

private:
     rclcpp::Node::SharedPtr node_ptr_;
     std::chrono::steady_clock::time_point start_time;
     std::chrono::steady_clock::time_point current_time;

};



