#include<iostream>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include <geometry_msgs/msg/point.hpp>
#include "yaml-cpp/yaml.h"
#include "mm_behaviors/utils.hpp"
#include <espeak/speak_lib.h>
#include "tf2/exceptions.h"
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include <moveit/robot_state/conversions.hpp>
#include "geometry_msgs/msg/point_stamped.hpp"
using Pose3D = geometry_msgs::msg::Point;
namespace WSTTA{
/**
 * @brief class for planning and executing robot base motion
 *  it inherits from BT::StatefulActionNode 
 * instantiates a Nav2 navigate_to_pose action client
 * takes the robot near the object before arm motion planning and execution
 */
class  Navigate : public BT::StatefulActionNode{
    public:
        using NavigateToPose = nav2_msgs::action::NavigateToPose;
        using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
        /**
         * @brief Constructor.
         * @param name Name of the Instance in the tree
         * @param config BT Node configuration for accessing BlackBoard ports
         * @param node_ptr  SharedPtr of ROS 2 node that coordinates the whole tree tick
         */
        Navigate(const std::string& name, const BT::NodeConfig& config,const rclcpp::Node::SharedPtr& node_ptr)
        :StatefulActionNode(name,config),node_ptr_{node_ptr} {
             tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->node_ptr_->get_clock());
        }
        /// process objects' stations poses to pick the nearest station
        int get_near_pose_index(std::vector<Pose3D>& t,Pose3D& robot_pose);
        /**
         * @brief BT overridden @fn onStart
         * it contains the logic of :
         * selecting the correct planning key,
         * obtaining the object or station pose
         * planning the for the planning key
         * @return BT::RUNNING if trajectory execution started correctly
         * @return BT::FAILURE otherwise
         */ 
        BT::NodeStatus onStart() override;
        /**
         * @brief BT overridden @fn onRunning
         * checks the state of the execution started by @fn onStart
         * @return BT::SUCCESS if goal execution was successful
         * @return BT::FAILURE otherwise
         */
        BT::NodeStatus onRunning() override;
        /// @brief frees ressources if necessary
        void onHalted() override;

        static BT::PortsList providedPorts() {
             /**
             * @brief BT Input Port that gives the key of the current instance of the class
             * the plan key represents, here, a name of  the robot base poses either predefined or 
             * special key which is "in_track_object" which indicates to plan to near
             * current object pose
             * one can add more key that has a pose to which to plan
            */ 
            return { BT::InputPort<std::string>("key"),
            
            BT::BidirectionalPort<bool>("is_go_to_initials"),
            /// BT Bidirectional port containing container status
            BT::BidirectionalPort<bool>("is_container_full"),

            BT::InputPort<TargetPose>("base_long_path_pose"),

            BT::InputPort<TargetPose>("base_initial_pose"),

            BT::BidirectionalPort<PlanObjectInfo>("plan_object_info"),
            /**
             * @brief BT Input Port that gives the poses of the objects(station).
            */ 
            BT::InputPort<std::vector<Pose3D>>("objects_poses_vect"),
            /**
             * BT Input Port that gives the radius of circle for good manipulation 
             * so that the robot can stop conveniently after navigation near the object
            */ 
            BT::InputPort<double>("circle_radius"),
            /**
             * @brief poses of the station of unloading.
             */
            BT::InputPort<TargetPose>("unload_poses"),};

        }

        /// Action client callbacks to receive the result of execution
        void result_callback(const GoalHandleNav::WrappedResult& result);
        
        /**
         * @brief checks the if the target can be obtained from the blackboard
         * port name that is given by @param var_name and
         * if possible then store the pose of the target in @param t
         * to be later used for Nav2 planning
        */
        bool check_if_target(TargetPose& t,std::string var_name);
    private:
        /// @brief member to store current goal pose for the base
        TargetPose target_pose;
        /// @brief stores the base goal key 
        std::string key;
        /// @brief member to indicate if Navigation is finished (result is returned)
        bool is_done;
        /**
         * @brief It stores the Nav2 goal pose near object.
         * concerns the process level to have the correct pose
         * it is different from @param target_pose which is 
         * more general.
         * 
         */
        TargetPose nav_target_pose;
        /** 
         * @brief stores the robot current position
        */
        Pose3D robot_pose_in_map;
        /** 
         * @brief store the selected nearest object pose
        */
        Pose3D object_pose;

        /// @name tf components
        /// @{ 
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_; ///< tf buffer 
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_; ///< tf listener to get robot state and transform
        /// @}

        /// @brief navigation frame (goal coordinates frame), default is "map"
        std::string navigation_frame;
        /// @brief robot base frame default is "base_link"
        std::string base_frame;
        /// @brief info about the object( index in vector and pose)
        PlanObjectInfo plan_object_info_;
        /// @brief store the object pose in map frame
        geometry_msgs::msg::PointStamped object_map_pose;
        /// @brief the at most five objects
        std::vector<Pose3D> Objects_poses;
        /// result returned by navigation
        rclcpp_action::ResultCode nav_result_;
        rclcpp::Node::SharedPtr node_ptr_;
        /// @brief store selected object index in the vector 
        int index;
        /// @brief stores raduis of good manipulation
        double radius;
        /**
         * @brief orientation variable
         * @param theta is the orientation of robot of the vector Robot->Object in planning frame
         * @param shift_theta stores the shifted angle in the blackboard
         * @param shift_amplitude stores the amplitude of shifting the angle 
         * if the plan for current theta is not successful
         */
        double theta, shift_theta,shift_amplitude;
        /// @brief is a Nav2 client of type **navigate_to_pose** that will carry out navigationq
        rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
        /// Nav2 Goal handdle
        GoalHandleNav::SharedPtr client_goal_handle;

};
}// end of namespace WSTTA

