#include<chrono>
#include <cstdlib>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "yaml-cpp/yaml.h"
#include "sensor_msgs/sensor_msgs/msg/battery_state.hpp"
#include "mm_behaviors/utils.hpp"

using namespace std::chrono_literals;
using  Batterystate = sensor_msgs::msg::BatteryState;
/**
 * @authors KAFANDO THOMAS (thomass.kaf20@gmail.com)
 * @date 2026
 */
namespace WSTTA{
/**
 * @authors KAFANDO THOMAS
 * @brief class to check battery state : it reads battery state from the BlackBoard 
 * and decides where okay or not
 */
class BatteryChecker:public BT::ConditionNode{

    public:
    /**
     * @brief Constructor.
     * @param name Name of the Instance in the tree
     * @param config BT Node configuration for accessing BlackBoard ports
     * @param node  SharedPtr ROS 2 node that coordinates the whole tree tick
     */
    BatteryChecker(std::string name,const BT::NodeConfig& config,const rclcpp::Node::SharedPtr& node);
    /** 
     * @brief function to read battery state from blackboard and the thresold for specific node
     * in fact, the battery treshold sufficient for navigation for a little task exectuion can't be the same
     * @return true if @param percentage_treshold <  @param batterypercentage
    */
    bool battery_is_ok();
    /** 
     * @brief BT overridden function.
     * It call @fn battery_is_ok and 
     * @return BT::SUCCESSS  if  @fn battery_is_ok @return true
    */
    BT::NodeStatus tick() override;
    
    static BT::PortsList providedPorts() {
        return {
        BT::InputPort<double>("percentage_treshold"),
        BT::InputPort<double>("battery_percentage")};
    }

    void play_sound()
    { 
        system("aplay /home/user/ros2_ws/src/my_pkg/sounds/alert.wav");
    }

    private:
    /// ROS 2 node SharedPtr to point to the BT coordinator
    rclcpp::Node::SharedPtr node;
    /// @name 
    /// @{
    double percentage_treshold; ///< battery treshold in percentage to store the blackboard port percentage_treshold
    double batterypercentage; ///< battery state in percentage to store the blackboard port battery_percentage
    /// @}
};
/**
 * @authors KAFANDO THOMAS
 * @brief class to check container state : it reads sensor state from the BlackBoard 
 * to check if the container is full or not
 */
class ContainerChecker:public BT::ConditionNode{
    public:
    /**
     * @brief Constructor.
     * @param name Name of the Instance in the tree
     * @param config BT Node configuration for accessing BlackBoard ports
     * @param node  SharedPtr ROS 2 node that coordinates the whole tree tick
     */
    ContainerChecker(std::string name,const BT::NodeConfig& config,const rclcpp::Node::SharedPtr& node);

    /** 
     * @brief BT overridden function.
     * @return BT::SUCCESSS  if  @param is_container_full is true
    */
    BT::NodeStatus tick() override;
    
    static BT::PortsList providedPorts() {
        return {BT::InputPort<bool>("is_container_full")};
    }

    void play_sound()
    { 
        system("espeak the container is full !");
    }

    private:
    ///  @brief ROS 2 node SharedPtr to point to the BT coordinator
    rclcpp::Node::SharedPtr node;
    ///  @brief stores the the blackboard port is_container_full
    bool is_container_full;


};


/**
 * @authors KAFANDO THOMAS
 * @brief class to check if the robot must go back to initial state.
 * 
 */
class CheckGoToInitials:public BT::ConditionNode{
    public:
    /**
     * @brief Constructor.
     * @param name Name of the Instance in the tree
     * @param config BT Node configuration for accessing BlackBoard ports
     * @param node  SharedPtr ROS 2 node that coordinates the whole tree tick
     */
    CheckGoToInitials(std::string name,const BT::NodeConfig& config,const rclcpp::Node::SharedPtr& node);
    /** 
     * @brief BT overridden function.
     * @return BT::SUCCESSS  if member is_go_to_initials is true
    */
    BT::NodeStatus tick() override;
    
    static BT::PortsList providedPorts() {
        return { 
            /// blackboard port containing the boolean indicating if robot should go to initial state
            BT::InputPort<bool>("is_go_to_initials")};
    }
    private:
        /// @brief store the blackboard port is_go_to_initials
        bool is_go_to_initials;
        ///  @brief ROS 2 node SharedPtr to point to the BT coordinator
        rclcpp::Node::SharedPtr node;
};
/**
 * @authors KAFANDO THOMAS
 * @brief class to check if the robot must go back to initial state.
 * 
 */
class JustNavigateChecker:public BT::ConditionNode{
    public:
    /**
     * @brief Constructor.
     * @param name Name of the Instance in the tree
     * @param config BT Node configuration for accessing BlackBoard ports
     * @param node  SharedPtr ROS 2 node that coordinates the whole tree tick
     */
    JustNavigateChecker(std::string name,const BT::NodeConfig& config,const rclcpp::Node::SharedPtr& node);
    /** 
     * @brief BT overridden function.
     * @return BT::SUCCESSS  if  @param is_container_full is false and 
     * @param objects_poses_vect a blackboard port of the class is empty
    */
    BT::NodeStatus tick() override;
    bool is_just_navigate();
    
    static BT::PortsList providedPorts() {
        return { 
            /// @brief blackboard port containing the station poses(objects poses for minimal app)
            BT::InputPort<std::vector<Pose3D>>("objects_poses_vect"),
            ///  @brief blackboard port giving the sensor measure of the level (full or not) of objects in the container
            BT::InputPort<bool>("is_container_full")};
    }
    private:
        /// @brief ROS 2 node SharedPtr to point to the BT coordinator
        rclcpp::Node::SharedPtr node;
};
} // end of namespace WSTTA

