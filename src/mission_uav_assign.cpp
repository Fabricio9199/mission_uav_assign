#include "mission_uav_assign.h"

/**
 * Class Constructor
 */

Mission::Mission() : private_nh("~")
{
    //Read Yaml
    private_nh.param<bool>("retainLastSetPoint", retainLastSetPoint, false);

    private_nh.param<float>("rangeToSetNewSetPoint", rangeToSetNewSetPoint, 0.5);
    private_nh.param<float>("rangeToDefineSetPointReached", rangeToDefineSetPointReached, 0.5);
    
    private_nh.param<string>("servicePathSetVelocity", servicePathSetVelocity, "/mavros/param/set");
    private_nh.param<float>("maxUAVVelocityXY", maxUAVVelocityXY, 5);
    
    private_nh.param<float>("xLimit", xLimit, 250);
    private_nh.param<float>("yLimit", yLimit, 250);
    private_nh.param<float>("zLimit", zLimit, 50);

    //Declare Our Subscribers
    getPose = private_nh.subscribe<geometry_msgs::PoseStamped>("getPose", 1, &Mission::callbackgetPose, this);
    getstateUAV = private_nh.subscribe<mavros_msgs::State>("stateUAV", 1, &Mission::callbackgetState, this);

    //Declare Our Publishers
    setPoint = private_nh.advertise<geometry_msgs::PoseStamped>("setPoint", 50);
    setStatusMission = private_nh.advertise<std_msgs::Bool>("statusMission", 50);
    pubModelUAV = private_nh.advertise<visualization_msgs::Marker>("droneModel", 50);

    //Declare Our Services
    mySrvSetPoint = private_nh.advertiseService("srvSetPoint", &Mission::setSrvPoint, this);
    mySrvCancelMission = private_nh.advertiseService("srvCancelMission", &Mission::cancelMission, this);

    //Declare Our Clients
    armingClient = private_nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    setUAVMode = private_nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    setUAVVelocity = private_nh.serviceClient<mavros_msgs::ParamSet>(servicePathSetVelocity);

    offb_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;
    last_request = ros::Time::now();
}

/** 
 * Class Destructor
 */

Mission::~Mission() {}

/**
 * Initialize
 */

void Mission::Init(ros::Rate rate)
{

    ROS_INFO("Waiting for FCU connection!");
    while (ros::ok() && !currentState.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected!");

    this->setVelocityUAV(maxUAVVelocityXY); //Set Velocity

}

/** 
 * Our Main Loop
 */
void Mission::Loop()
{
    //Publish status Mission
    this->PubStatusMission();

    //Check Conections
    this->checkConection();

    //Publish last setPoint
    this->goSetPoint();

    //Publish Drone Model
    PubUAVModel();

    PersonalDelay(0.01);
}

/** 
 * Do Delay
 * t is second
 */

void Mission::PersonalDelay(float t)
{
    double begin = ros::Time::now().toSec(); //Init Count Time
    double currentTime = 0;
    while (currentTime < t)
    {
        currentTime = ros::Time::now().toSec() - begin;
        ros::spinOnce();
    }
}

/** 
 * Publish Waypoint
 */

void Mission::goSetPoint()
{
    if(this->storeSetPoints.size()>0)
    {
        this->setPoint.publish(this->storeSetPoints[0]);
    }
}

/** 
 * Publish Status Mission
 */

void Mission::PubStatusMission()
{
    this->setStatusMission.publish(statusMission);
}

/** 
 * Set Velocity UAV
 */

void Mission::setVelocityUAV(float vel)
{
    msgParam.request.param_id = "MPC_XY_VEL_MAX";
    msgParam.request.value.real = vel;
    msgParam.response.success = false;
    while (!msgParam.response.success)
    {
        if (setUAVVelocity.call(msgParam) && msgParam.response.success)
        {  
            ROS_INFO("Max velocity ajusted to %lf", maxUAVVelocityXY);
        }
    }

}

/** 
 * Callback to get pose
 */

void Mission::callbackgetPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    curentPose = *msg;

    //Check Set Point Reached
    this->checkSetPointsReached();

    ros::spinOnce();
}

/** 
 * Callback to get state UAV
 */
void Mission::callbackgetState(const mavros_msgs::State::ConstPtr &msg)
{
    currentState = *msg;
    ros::spinOnce();
}

/** 
 * Callback to get state UAV
 */
void Mission::PubUAVModel()
{
    visualization_msgs::Marker drone;
    drone.header.frame_id = "local_origin";
    drone.header.stamp = ros::Time::now();
    drone.type = visualization_msgs::Marker::MESH_RESOURCE;
    drone.mesh_resource = "model://iris/meshes/iris.stl";
    if (drone.mesh_resource.find("model://") != std::string::npos) {
        if (resolveUri(drone.mesh_resource)) {
        ROS_ERROR("RVIZ world loader could not find drone model");        
        }
    }

    drone.mesh_use_embedded_materials = true;
    drone.scale.x = 1.5;
    drone.scale.y = 1.5;
    drone.scale.z = 1.5;
    drone.pose.position.x = curentPose.pose.position.x;
    drone.pose.position.y = curentPose.pose.position.y;
    drone.pose.position.z = curentPose.pose.position.z;
    drone.pose.orientation.x = curentPose.pose.orientation.x;
    drone.pose.orientation.y = curentPose.pose.orientation.y;
    drone.pose.orientation.z = curentPose.pose.orientation.z;
    drone.pose.orientation.w = curentPose.pose.orientation.w;
    drone.id = 0;
    drone.lifetime = ros::Duration();
    drone.action = visualization_msgs::Marker::ADD;

    pubModelUAV.publish(drone);
    ros::spinOnce();
}

/** 
 * Check Conection
 */
void Mission::checkConection()
{
    if (this->storeSetPoints.size() > 0)
    {
        if (currentState.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (setUAVMode.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("OFFBOARD enabled!");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (!currentState.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (armingClient.call(arm_cmd) &&
                    arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed!");
                }
                last_request = ros::Time::now();
            }
        }
    }
}

/** 
 * Check Set Points Reached and Set new waypoint
 */

void Mission::checkSetPointsReached()
{
    while(cancelWaypointWorking)
    {
        PersonalDelay(0.1);
    }

    if (this->storeSetPoints.size() > 0)
    {
        long double myDistance = sqrt(pow(curentPose.pose.position.x - storeSetPoints.front().pose.position.x, 2.0) + pow(curentPose.pose.position.y - storeSetPoints.front().pose.position.y, 2.0) + pow(curentPose.pose.position.z - storeSetPoints.front().pose.position.z, 2.0));
        if (myDistance < rangeToDefineSetPointReached)
        {
            if (!retainLastSetPoint || this->storeSetPoints.size()>1)
            {
                this->storeSetPoints.erase(this->storeSetPoints.begin());
                if (this->storeSetPoints.size() > 0)
                {
                    ROS_WARN("New Set Point Defined. Now, we have %ld waypoints...", storeSetPoints.size());
                    this->setPoint.publish(this->storeSetPoints[0]);
                    this->statusMission.data = true;
                }
                else
                {
                    ROS_WARN("Last Set Point Reached!");
                    this->statusMission.data = false;
                }
            }
            if(retainLastSetPoint && this->storeSetPoints.size() == 1 && this->statusMission.data)
            {
                ROS_WARN("Last Set Point Reached!");
                this->statusMission.data = false;
            }
        }
    }
}

/** 
 * Service to set a New Pose
 */

bool Mission::setSrvPoint(mission_uav_assign::setPoint::Request &req, mission_uav_assign::setPoint::Response &resp)
{
    geometry_msgs::PoseStamped newWaypoint;
    
    while(cancelWaypointWorking)
    {
        PersonalDelay(0.1);
    }

    addWaypointWorking = true;

    if (abs(req.Pose.position.x)<xLimit && abs(req.Pose.position.y)<yLimit && abs(req.Pose.position.z)<zLimit)
    {

    if (storeSetPoints.size() > 0)
    {
        long double myDistance = sqrt(pow(req.Pose.position.x - storeSetPoints.back().pose.position.x, 2.0) + pow(req.Pose.position.y - storeSetPoints.back().pose.position.y, 2.0) + pow(req.Pose.position.z - storeSetPoints.back().pose.position.z, 2.0));
        if (myDistance > rangeToSetNewSetPoint)
        {
            resp.response = true;
            newWaypoint.pose = req.Pose;
            if(!this->statusMission.data && this->storeSetPoints.size()==1 && this->retainLastSetPoint)
            {
                storeSetPoints[0] = newWaypoint;
            }else{
                storeSetPoints.push_back(newWaypoint);
            }
            this->statusMission.data = true;
            ROS_INFO("New SetPoint Added Mission! Actually we have %ld waypoints", storeSetPoints.size());
        }
        else
        {
            resp.response = false;
            ROS_ERROR("Impossible add new waypoint to Mission. Last waypoint is close to new waypoint!");
        }
    }
    else
    {
        resp.response = true;
        newWaypoint.pose = req.Pose;
        this->storeSetPoints.push_back(newWaypoint);
        this->setPoint.publish(this->storeSetPoints[0]);
        this->statusMission.data = true;
        ROS_INFO("New SetPoint Added Mission! Actually we have %ld waypoints", storeSetPoints.size());
    }
    }else{
        resp.response = false;
        ROS_ERROR("This waypoint is outside the predefined limits!");
    }
    
    addWaypointWorking = false;
    return true;
}

/** 
 * Service to set a New Pose
 */

bool Mission::cancelMission(mission_uav_assign::cancelMission::Request &req, mission_uav_assign::cancelMission::Response &resp)
{
    while(addWaypointWorking)
    {
        PersonalDelay(0.1);
    }

    cancelWaypointWorking = true;

    if (req.cancelMission and storeSetPoints.size()>0)
    {
        storeSetPoints.clear();
        if(retainLastSetPoint)        
            storeSetPoints.push_back(curentPose);
        this->statusMission.data = false;
        resp.response = true;
    }

    resp.response = true;
    ROS_INFO("Mission Canceled");
    cancelWaypointWorking = false;
    return true;
}

int Mission::resolveUri(std::string& uri) {
  // Iterate through all locations in GAZEBO_MODEL_PATH
  char* gazebo_model_path = getenv("GAZEBO_MODEL_PATH");
  char* home = getenv("HOME");
  uri = uri.substr(7, std::string::npos);
  std::stringstream all_locations(gazebo_model_path, std::ios_base::app | std::ios_base::out | std::ios_base::in);
  all_locations << ":" << home << "/.gazebo/models";
  std::string current_location;
  while (getline(all_locations, current_location, ':')) {
    struct stat s;
    std::string temp = current_location + uri;
    if (stat(temp.c_str(), &s) == 0) {
      if (s.st_mode & S_IFREG)  // this path describes a file
      {
        uri = "file://" + current_location + uri;
        return 0;
      }
    }
  }
  return 1;
}