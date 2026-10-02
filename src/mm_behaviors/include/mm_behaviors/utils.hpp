#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#ifndef UTILS_HPP
#define UTILS_HPP
using Pose3D = geometry_msgs::msg::Point;
using PoseStamped = geometry_msgs::msg::PoseStamped;
/**
 * @brief struct containing 3D position and 
 * orientation about z-axis
 */
struct TargetPose{
   
    Pose3D position;
    double orientation;
    /// @brief default constructor.
    TargetPose(){}
    /** 
     * @brief constructor using position and orientation
     * @param x first postion variable
     * @param y second postion variable
     * @param z third postion variable
     * @param theta orientation
     */
    TargetPose(double x,double y,double z,double theta){
        position.set__x(x);
        position.set__y(y);
        position.set__z(z);
        orientation = theta;
    }
    /** 
     * @brief constructor using  @param L of type **geometry_msgs::msg::Point**
     */
    TargetPose(Pose3D L){
        position.set__x(L.x);
        position.set__y(L.y);
        position.set__z(L.z);
        orientation = 0.0;
    }
    /// @brief computes norm of TargetPose object
    /// @return  the norm as double
    double norm(){
        // just to have onground distance
        return std::pow(position.x,2)+std::pow(position.y,2);
    }
    
    TargetPose operator-(const TargetPose& t2){
        return TargetPose(this->position.x-t2.position.x,this->position.y-t2.position.y,
        this->position.z-t2.position.z,this->orientation-t2.orientation);
    }
    TargetPose operator+(const TargetPose& t2){
        return TargetPose(this->position.x+t2.position.x,this->position.y+t2.position.y,
        this->position.z+t2.position.z,this->orientation+t2.orientation);
    }
};

/**
 * @brief struct  used to stores an object pose
 * @param pose stores 3D position and 3D orientation of an object
 * @param index store objects index in vector of objects
 * 
 */
struct PlanObjectInfo{
    int index;
    PoseStamped pose;
};
#endif