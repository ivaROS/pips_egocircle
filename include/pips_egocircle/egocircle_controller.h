#include <turtlebot_trajectory_testing/obstacle_avoidance_controller.h>
#include <pips_trajectory_testing/pips_cc_wrapper.h>


namespace pips_egocircle
{

class PipsEgoCircleTrajectoryController : public turtlebot_trajectory_testing::TurtlebotObstacleAvoidanceController
{
public:
  PipsEgoCircleTrajectoryController(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name=DEFAULT_NAME);
  ~PipsEgoCircleTrajectoryController(){};

  virtual bool init();
  
  static constexpr const char* DEFAULT_NAME= "EgoCircleController";
  
protected:
  bool isReady(const std_msgs::Header& header);
  
  void sensorCb(const std_msgs::Header& header);
  
  void generateTrajectories();
  
  virtual void setupTrajectoryTesters();

  
private:
  std::string name_;
  ros::NodeHandle pnh_;
  
  std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> cc_wrapper_;      
  
  typedef turtlebot_trajectory_testing::TurtlebotObstacleAvoidanceController Super;

};

} //ns pips_egocircle

