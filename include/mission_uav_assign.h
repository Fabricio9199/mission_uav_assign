#pragma once

#ifndef PROJECT_MISSION_UAV_ASSIGN_H
#define PROJECT_MISSION_UAV_ASSIGN_H

#include <math.h>
#include <stdlib.h>
#include <cstdlib>
#include <vector>
#include <string>
#include <sys/stat.h>

//ROS Msgs
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"

#include "mission_uav_assign/setPoint.h"
#include "mission_uav_assign/cancelMission.h"

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ParamSet.h>

//Set Namespace
using namespace std;

class Mission {

    public:
        Mission();
        ~Mission();
        
        void Loop();
        void Init(ros::Rate rate);

    private:
        ros::NodeHandle private_nh;
        
        void callbackgetPose(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void callbackgetState(const mavros_msgs::State::ConstPtr& msg);
        void PersonalDelay(float t);
        void PubStatusMission();
        void PubUAVModel();
        void checkSetPointsReached();
        void checkConection();
        void goSetPoint();
        void setVelocityUAV(float vel);

        int resolveUri(std::string& uri);
        bool setSrvPoint(mission_uav_assign::setPoint::Request &req, mission_uav_assign::setPoint::Response &resp);
        bool cancelMission(mission_uav_assign::cancelMission::Request &req, mission_uav_assign::cancelMission::Response &resp);

        ros::Publisher setPoint, setStatusMission, pubModelUAV;
        ros::Subscriber getPose, getstateUAV;
        ros::ServiceServer mySrvSetPoint, mySrvCancelMission;
        ros::ServiceClient armingClient, setUAVMode, setUAVVelocity;

        geometry_msgs::PoseStamped curentPose;
        mavros_msgs::State currentState;
        mavros_msgs::SetMode offb_set_mode;
        mavros_msgs::CommandBool arm_cmd;
        mavros_msgs::ParamSet msgParam;
        ros::Time last_request;
        visualization_msgs::Marker modelUAV;

        vector<geometry_msgs::PoseStamped> storeSetPoints;

        std_msgs::Bool statusMission;

        bool retainLastSetPoint, cancelWaypointWorking, addWaypointWorking;
        
        float rangeToSetNewSetPoint, rangeToDefineSetPointReached;
        float xLimit, yLimit, zLimit; //Limits to accepet new waypoint
        float maxUAVVelocityXY; //Speed Max

        string servicePathSetVelocity;
};

#endif