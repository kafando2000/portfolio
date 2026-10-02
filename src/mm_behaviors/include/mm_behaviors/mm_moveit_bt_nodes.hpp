
#ifndef MM_MOVEIT_BT_NODES_HPP
#define MM_MOVEIT_BT_NODES_HPP

#include <memory>
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
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include "moveit/robot_model/robot_model.hpp"
#include "moveit/robot_state/robot_state.hpp"
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit_msgs/msg/constraints.hpp>
// MoveitCpp
#include "moveit/moveit_cpp/moveit_cpp.hpp"
#include "moveit/moveit_cpp/planning_component.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
//#include "visualization/moveit_visual_tools.h"

namespace WSTTA{
/**
 * @brief class for planning robot arm motion
 *  it inherits from BT::StatefulActionNode 
 * it uses moveItCpp api for more performance
 * it just plans and send the result of the plan to the @class ExecuteArmPlan
 * it for the moveIt group "arm"
 */
class PlanForTheArm : public BT::StatefulActionNode{
  
public:
    /**
     * @brief Constructor.
     * @param name Name of the Instance in the tree
     * @param config BT Node configuration for accessing BlackBoard ports
     * @param node_ptr  SharedPtr of ROS 2 node that coordinates the whole tree tick
     * @param moveit_cpp_ptr SharedPtr of MoveItCpp that will perform the instantiation 
     * of the planning component
     */
    PlanForTheArm(const std::string& name, const BT::NodeConfig& config, const rclcpp::Node::SharedPtr& node_ptr,
            const std::shared_ptr<moveit_cpp::MoveItCpp>& moveit_cpp_ptr);
            
    /**
     * @brief BT overridden @fn onStart
     * it contains the logic of :
     * selecting the correct planning key,
     * obtaining the object or station pose
     * planning the for the planning key
     * @return BT::RUNNING if all configuration for planning is okay.
     * @return BT::FAILURE otherwise
     */ 
    BT::NodeStatus onStart() override;
    /**
     * @brief BT overridden @fn onRunning
     * checks the state of the plan output given by @fn onStart
     * @return BT::SUCCESS if plan was successful
     * @return BT::FAILURE otherwise
     */
    BT::NodeStatus onRunning() override;
    /**
     * @brief BT overridden @fn onHalted
     */
    void onHalted() override;
     /**
     * @brief function to get the object info
     * @return true if object info is found correctly
     * @return false otherwise
     */
    bool get_plan_object_info();

    static BT::PortsList providedPorts(){
        return{
            /**
             * BT Input Port that gives the plan key of the current instance of the class
             * the plan key represents, here, a name of moveIt group state defined in the
             * srdf file
             * one exceptional plan key is "for_object" which indicates to plan for the 
             * current object pose
             * one can add more configuration(group state) and add it to the @param arm_predefined_states in 
             * the node parameters yaml file.
            */ 
            BT::InputPort<std::string>("plan_key"),
            /**
             * BT Input Port that gives the poses of the objects(station)
            */ 
            BT::InputPort<std::vector<Pose3D>>("objects_poses_vect"),
            /**
             * BT Input Port that gives the radius of circle for good manipulation 
             * so that the robot can stop conveniently after navigation near the object
            */ 
            BT::InputPort<double>("circle_radius"),
            /**
             * BT Bidirectional Port that shifts the angle @param theta to have robot best pose near
             * the object if the straight line (plan between robot current pose and object pose)
             * is not feasible.
            */ 
            BT::BidirectionalPort<double>("theta_shift"),
            /// the sensor data about the container level of fullness
            /// it BT Input port
            BT::InputPort<bool>("is_container_full"),
             /**
             * BT Bidirectional Port that gives the info about the object(the selected object
             * from the objects poses vector)
             * stores the index and the pose
            */ 
            BT::BidirectionalPort<PlanObjectInfo>("plan_object_info"),
            /**
             * BT Output port the store the trajectory information to be executed
             * robot trajectory object that will be sent to execution class
            */ 
            BT::OutputPort<std::shared_ptr<robot_trajectory::RobotTrajectory>>("arm_joints_traject"),};
    }

public:
    /// planning group
    const std::string PLANNING_ARM_GROUP_NAME = "arm";
private:
    /// @brief coordinator node (helps grabbing the resources for planning)
    rclcpp::Node::SharedPtr node_ptr_;
    /// @name tf components
    /// @{ 
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_; ///< tf buffer 
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_; ///< tf listener to get robot state and transform
    /// @}

    /// @name  shared ressources  for robot and planning
    /// @{
    std::shared_ptr<moveit::core::RobotState> state_; /// robot state
    moveit::core::RobotModelConstPtr robot_model_; /// robot mdoel
    std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_ptr; /// moveItcpp object
    std::shared_ptr<moveit_cpp::PlanningComponent> arm_planning_component; /// planning component for the planning group "arm"
    planning_interface::MotionPlanResponse plan_solution; /// solution of the plan
    moveit::core::MoveItErrorCode status_; /// status of the plan
    /// @}
    
    /// @brief plan key here is one defined state name : it will be among the <<group_predefined_states>>  otherwise 
    /// it's to track an object with the plan_key = "for_object"
    std::string plan_key_;
    /// @brief stores moveIt planning group states defined in srdf file 
    /// it is given by the parameters  yaml file
    std::vector<std::string> arm_predefined_states;
    /// @brief stores the first five objects poses(if less than five just the whole) given by 
    ////BT port objects_poses_vect
    std::vector<Pose3D> Objects_poses;
    /// @brief info about the object( index in vector and pose)
    PlanObjectInfo plan_object_info_;
    
    /// @brief  is obtained from robot model
    std::string planning_frame ;
    /// @brief defined in yaml file if not it is just map (it is the frame in which the objects poses are defined)
    std::string navigation_frame;
    /// it gives info about plan_key search if it is found or not
    bool found_key;
};
/**
 * @brief class for executing robot arm motion
 * it inherit from BT::StatefulActionNode 
 * it uses moveItCpp api for more performance
 * it just takes the result of the plan from the @class PlanForTheArm
 * it is for the moveIt group "arm"
 */
class ExecuteArmPlan : public BT::StatefulActionNode{
    public:
        /**
         * @brief Constructor.
         * @param name Name of the Instance in the tree
         * @param config BT Node configuration for accessing BlackBoard ports
         * @param node_ptr  SharedPtr of ROS 2 node that coordinates the whole tree tick
         * @param moveit_cpp_ptr SharedPtr of MoveItCpp that will perform the instantiation 
         * of the planning component
        */
        ExecuteArmPlan(const std::string& name, const BT::NodeConfig& config, const rclcpp::Node::SharedPtr& node_ptr,
            const std::shared_ptr<moveit_cpp::MoveItCpp>& moveit_cpp_ptr);
        // for debug purpose
        static void read_text_in_speak(std::string t);

        /**
         * @brief BT overridden @fn onStart
         * it grab from the blackboard the trajectory to execute 
         * and runs the execution of the trajectory
         * @return BT::RUNNING if all configuration for execution is okay.
         * @return BT::FAILURE otherwise
        */ 
        BT::NodeStatus onStart() override;
        /**
         * @brief BT overridden @fn onRunning
         * perform a follow-up of the execution status
         * @return BT::RUNNING if still on execution.
         * @return BT::SUCCESS if execution was successful
         * @return BT::FAILURE otherwise
        */ 
        BT::NodeStatus onRunning() override;
        /**
         * @brief BT overridden @fn onHalted
         * helps free ressources
         */
        void onHalted() override;

        static BT::PortsList providedPorts(){
            return{
                /// BT Input port that contains the trajectory info to be executed
                BT::InputPort<robot_trajectory::RobotTrajectoryPtr>("arm_joints_traject"),
                /// BT Bidirectional port containing container status
                BT::BidirectionalPort<bool>("is_container_full"),
                /**  
                 * @brief it helps identify which plan trajectory is being executed
                 * if "for_object", there is possibility to sync gripper execution by 
                 * setting the port is_exec_gripper to true
                */
                BT::InputPort<std::string>("exec_key"),
                /// if true then sync the gripper trajectory execution
                BT::BidirectionalPort<bool>("is_exec_gripper"),

            };
        }
    public:
        /// @brief  moveIt planning group
        const std::string PLANNING_ARM_GROUP_NAME = "arm";
    private:
        /// @brief Coordinator node (ROS 2 node)
        rclcpp::Node::SharedPtr node_ptr_;
        /// @brief the moveit plan trajectory
        robot_trajectory::RobotTrajectoryPtr my_plan;
        /// @brief MoveItCpp instance
        std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_ptr;
        /// @brief robot arm plamning component
        std::shared_ptr<moveit_cpp::PlanningComponent> arm_planning_component;
        /// @brief controllers forn the arm trajectory execution
        const std::vector<std::string> CONTROLLERS = {"arm_controller"};
        /// @brief store the value of the BT port exec_key
        std::string exec_key;
        /// @brief  controller manager execution status
        moveit_controller_manager::ExecutionStatus status;
        /// @brief a future for the  controller manager execution status
        std::future<moveit_controller_manager::ExecutionStatus> execution_future_;
        /// @brief MoveIt trajectory execution manager
        trajectory_execution_manager::TrajectoryExecutionManagerPtr manager;
        /// @brief in lack of level sensor for the container
        /// it is used for simulating the filling state of the container
        int products_counter;
        /// @brief products max number to consider container is full
        int products_max_number;
        


};

/**
 * @brief class for panning and executing robot gripper motion
 * it inherits from BT::StatefulActionNode 
 * it uses moveItCpp api for more performance
 * it is for the moveIt group "ee_group"
 */

class PlanExecForGripper : public BT::StatefulActionNode{
    public:
        /**
         * @brief Constructor.
         * @param name Name of the Instance in the tree
         * @param config BT Node configuration for accessing BlackBoard ports
         * @param node_ptr  SharedPtr of ROS 2 node that coordinates the whole tree tick
         * @param moveit_cpp_ptr SharedPtr of MoveItCpp that will perform the instantiation 
         * of the planning component
        */
        PlanExecForGripper(const std::string& name, const BT::NodeConfig& config, const rclcpp::Node::SharedPtr& node_ptr,
            const std::shared_ptr<moveit_cpp::MoveItCpp>& moveit_cpp_ptr);
        


        // for debug purpose
        static void read_text_in_speak(std::string t);

        /**
         * @brief BT overridden @fn onStart
         * it plan the matching plan key for the gripper
         *  and runs the execution of the trajectory
         * @return BT::RUNNING if all configuration for execution is okay.
         * @return BT::FAILURE otherwise
        */ 
        BT::NodeStatus onStart() override;
        /**
         * @brief BT overridden @fn onRunning
         * perform a follow-up of the execution status of gripper motion
         * @return BT::RUNNING if still on execution.
         * @return BT::SUCCESS if execution was successful
         * @return BT::FAILURE otherwise
        */ 
        BT::NodeStatus onRunning() override;
        /**
         * @brief BT overridden @fn onHalted
         * helps free ressources
         */
        void onHalted() override;
        
        static BT::PortsList providedPorts(){
            return{
                /**
                 * @brief  BT Bidirectional Port  representing a collision sensor info
                 * if it is true when the exution is on plan key  "grasp" then
                 * the execution should stop not to damage the actuator 
                 * owing to current pick
                 */
                BT::BidirectionalPort<bool>("is_grasped"),
                /**
                 * @brief Bt Input Port holding plan key for end-effector to select 
                 * from the predefined group states for "ee_group"
                 * for the current configuration there are the following keys:
                 * ("open","grasp","close")
                 */
                BT::InputPort<std::string>("plan_key"),
                /// BT Bidirectional Port for syncing purpose with the arm motion
                BT::BidirectionalPort<bool>("is_exec_gripper"),

            };
        }
    public:
        /// @brief end-effector planning group
        const std::string PLANNING_EE_GROUP_NAME = "ee_group";
    private:
        /// @brief Coordinator node (ROS 2 node)
        rclcpp::Node::SharedPtr node_ptr_;
        /// the moveit plan response
        planning_interface::MotionPlanResponse my_plan;
        /// @brief MoveItCpp instance
        std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_ptr;\
        /// end-effector planning component
        std::shared_ptr<moveit_cpp::PlanningComponent> ee_planning_component;
        /// controllers for the end-effector 
        const std::vector<std::string> CONTROLLERS = {"mm_gripper_controller"};
        /**
         * @brief store the BT port plan_key of the instance of the class in the tree
         * enable the selection of the correspondent key for planning
         * it is sety to empty string at the beginning 
         * if it stays empty then planning will fail
         */
        std::string plan_key_;
        /// @name moveit execution manager
        /// @{
        moveit_controller_manager::ExecutionStatus status; ///< controller manager status
        std::future<moveit_controller_manager::ExecutionStatus> execution_future_; ///< status future
        trajectory_execution_manager::TrajectoryExecutionManagerPtr manager;/// execution manager
        /// @}
        /**
         * @brief vector to store the end-effector group states defined in srdf
         * the program explores the vector to see of @param plan_key_ is in the vector
         */
        std::vector<std::string> ee_predefined_states;
        


};




} /// end of namespace WSTTA

#endif



