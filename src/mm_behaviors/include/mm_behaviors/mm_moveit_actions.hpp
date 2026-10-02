#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include "tf2/exceptions.h"
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include <moveit/robot_state/conversions.hpp>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "mm_behaviors/utils.hpp"
#include <espeak/speak_lib.h>
#include "moveit/robot_model_loader/robot_model_loader.hpp"
#include "moveit/robot_model/robot_model.hpp"
#include "moveit/robot_state/robot_state.hpp"

class PlanForTheArm : public BT::StatefulActionNode{
    public:
         PlanForTheArm(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_ptr,
        const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group):StatefulActionNode(name,config),
        node_ptr_{node_ptr},move_group{move_group}
         {
           tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->node_ptr_->get_clock());
           robot_model_ = robot_model_loader_->getModel();
           plan_key_ = "";
         }

         static void read_text_in_speak(std::string t);
         BT::NodeStatus onStart() override;
         BT::NodeStatus onRunning() override;
         void onHalted() override;


         static BT::PortsList providedPorts(){
            return{
                BT::InputPort<std::string>("plan_key"),
                BT::InputPort<std::vector<Pose3D>>("objects_poses_vect"),
                BT::InputPort<double>("circle_radius"),
                BT::BidirectionalPort<double>("theta_shift"),
                BT::InputPort<bool>("is_container_full"),
                BT::OutputPort<TargetPose>("base_target"),
                BT::OutputPort<moveit_msgs::msg::RobotTrajectory>("arm_joints_traject"),

            };
         }

         int get_near_pose_index(std::vector<Pose3D>& t,Pose3D& robot_pose){
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


    private:
        // where the base should stay to pick object
        double theta,theta2, shift_theta,shift_amplitude;
        TargetPose nav_target_pose;
        TargetPose nav_target_in_odom;
        // the robot current positon
        Pose3D robot_pose_in_map;
        Pose3D robot_pose_in_odom;
        // selected near object
        Pose3D object_pose;
        // the at most five objects
        std::vector<Pose3D> Objects_poses;
        // raduis of security from object
        double radius;
        rclcpp::Node::SharedPtr node_ptr_;
        // tf buffer and listener to get robot pose
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
        std::shared_ptr<moveit::core::RobotState> state_;
        moveit::core::RobotModelPtr robot_model_;
        // the moveit plan
        //moveit_msgs::msg::RobotTrajectory my_plan;
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
        geometry_msgs::msg::PointStamped object_odom_pose;
        moveit::core::MoveItErrorCode status_;
        std::string plan_key_;
        int index;

        

};

using ExecuteTrajectory = moveit_msgs::action::ExecuteTrajectory;
using GoalHandleMoveit = rclcpp_action::ClientGoalHandle<ExecuteTrajectory>;

class ExecuteArmPlan : public BT::StatefulActionNode{
    public:

        ExecuteArmPlan(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_ptr,
         const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group):StatefulActionNode(name,config),
        node_ptr_{node_ptr},move_group{move_group}
         {
            
           
         }

        static void read_text_in_speak(std::string t);


        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

        static BT::PortsList providedPorts(){
            return{
                BT::InputPort<moveit_msgs::msg::RobotTrajectory>("arm_joints_traject"),
                BT::BidirectionalPort<bool>("is_container_full"),

            };
        }

        void result_callback(const GoalHandleMoveit::WrappedResult& result);

    private:
        rclcpp::Node::SharedPtr node_ptr_;
        // the moveit plan
        moveit_msgs::msg::RobotTrajectory my_plan;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
        rclcpp_action::ResultCode moveit_result_;
        bool is_over;
        rclcpp_action::Client<ExecuteTrajectory>::SharedPtr client_ptr_;
        GoalHandleMoveit::SharedPtr client_goal_handle;
        int products_counter = 0;


};