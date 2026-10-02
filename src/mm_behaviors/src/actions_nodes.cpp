#include "mm_behaviors/actions_nodes.hpp"
BT::NodeStatus StackObjectPose::tick(){

   bool is_data_valid = false;
    Pose3D t ;
    t.set__x(0.2);
    t.set__y(0.0);
    t.set__z(0.1);
    Objects_poses.push_back(t);

    if(auto any_locked = getLockedPortContent("objects_poses_vect")){
        RCLCPP_INFO(this->node_ptr_->get_logger(),"stacking the object pose !");
        if(!any_locked->empty()){   
            std::vector<Pose3D>* Poses = any_locked->castPtr<std::vector<Pose3D>>();
            Poses->push_back(t);
            is_data_valid = true;
        }
        else{
            any_locked.assign(Objects_poses);
            is_data_valid = true;
        }
    }
    //
    if(!is_data_valid){
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GetLongPathGoalPose::tick(){
    long_path.position.x = 15;
    long_path.position.y = 20;
    long_path.position.z = 0.0;
    long_path.orientation = 0.0;
    
      auto res = setOutput("base_long_path_pose",long_path);
      if(!res)
          return BT::NodeStatus::FAILURE;
      return BT::NodeStatus::SUCCESS;
}