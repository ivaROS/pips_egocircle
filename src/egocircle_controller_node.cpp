
#include <pips_egocircle/egocircle_controller.h>
#include <ros/ros.h>


int main(int argc, char **argv)
{
    std::string name= "egocircle_controller";
    ros::init(argc, argv, name);
    ros::start();
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    pips_egocircle::PipsEgoCircleTrajectoryController controller(nh, pnh);
    controller.init();


    ros::AsyncSpinner spinner(8); // This number should probably be configurable for different platforms
    spinner.start();
    ros::waitForShutdown();

    return 0;
}

