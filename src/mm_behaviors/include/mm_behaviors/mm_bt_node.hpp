#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "yaml-cpp/yaml.h"
#include "mm_behaviors/condition_nodes.hpp"
#include "mm_behaviors/mm_moveit_actions.hpp"
#include "mm_behaviors/mm_nav_actions.hpp"
#include "mm_behaviors/actions_nodes.hpp"
#include "mm_behaviors/mm_moveit_bt_nodes.hpp"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include <espeak/speak_lib.h>
using namespace std::chrono_literals;
namespace WSTTA{
/**
 * @brief Ros2 Node class that coordinates the BT nodes execution
 * and also ressources acquisition
 * it is shared with all the BT nodes accross the BT tree used
 * for the robot
 */
class WsttaBtPipelineNode : public rclcpp::Node {
    public:
        /**
         * @brief constructor with @param options
         * declares the parameters needed for the nodes
         */
        explicit WsttaBtPipelineNode(const rclcpp::NodeOptions & options);
        /// destructor
       ~WsttaBtPipelineNode()
            {
            
          tree_.haltTree();

        }
        /**
         * @brief tick function for the behavior tree
         * controls the signal pass frequency
         */
        void tick_function();
        /// @brief used for debug
        void read_text_in_speak(std::string t);
        /**
         * @brief creates the BT tree
         * initialize necessary variables
         * in the blackboard
         * instantiates the BT nodes
         * uses global blackboard 
         */
        void create_behavior_tree(); 
        /// @brief simulates battery decharge
        void battery_state_listener();
    

        /// @brief Configuration parameters file path
        std::string tree_xml_file_;
        /// @brief moveItCpp Instance to be shared with BT manipulation nodes
        std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_ptr;
        /// @brief ROS 2 callback group which will be Reentrant
        /// to enable other program to run concurrently with the BT tree tick
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        /// @brief BT tree
        BT::Tree tree_;
        /// @brief BT publisher for real time monitoring (if any premium version acquired)
        std::unique_ptr<BT::Groot2Publisher> publisher_ptr_;
        /// @brief  global blackboard used for global supervision
        BT::Blackboard::Ptr global_bb;
        /// @brief  battery state
        double bat;
        /// @brief check if first execution of tick function or not
        bool first_;


};
}// end of namespace WSTTA
int main(int argc, char **argv);
