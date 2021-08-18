#include "mission_uav_assign.h"

int main(int argc, char **argv)
{
    //////////////////// ROS INITIALIZATION //////////////////////////

    ros::init(argc, argv, "control_mission");
    ros::NodeHandle nh;
    ROS_INFO( "-----Initialize Mission UAV Assign -----" );
    Mission mission;

    ros::Rate loop_rate(10);

    mission.Init(loop_rate);

    while (ros::ok()) {
        
        mission.Loop();
        ros::spinOnce();
    }


    return 0;
}