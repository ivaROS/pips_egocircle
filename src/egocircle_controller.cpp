#include <pips_egocircle/egocircle_cc_wrapper.h>
#include <pips_egocircle/egocircle_controller.h>


namespace pips_egocircle
{
 
  PipsEgoCircleTrajectoryController::PipsEgoCircleTrajectoryController(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name) :
    Super(nh, pnh, "obstacle_avoidance"),
    name_(name),
    pnh_(pnh)
  {

      
  }

  
  
  void PipsEgoCircleTrajectoryController::setupTrajectoryTesters()
  {
    traj_tester_ = std::make_shared<turtlebot_trajectory_testing::GenAndTest>(nh_, Super::pnh_);
    traj_tester_->init();
    traj_tester2_ = traj_tester_;
        
    cc_wrapper_ = std::make_shared<pips_egocircle::EgoCircleCCWrapper>(nh_, pnh_, tfm_);
    traj_tester_->setCollisionChecker(cc_wrapper_->getCC());
}
  
     



  // Note: I haven't fully thought through other implementations, but this may be generic after all...
  // If so, then this code will probably move back to the main controller but be renamed 'transformReady' or something
  bool PipsEgoCircleTrajectoryController::isReady(const std_msgs::Header& header)
  {
    if(cc_wrapper_->isReady(header))
    {
      cc_wrapper_->update();
      return true;
    }
    else
    {
      return false;
    }
  }
  
  void PipsEgoCircleTrajectoryController::generateTrajectories()
  {
      std_msgs::Header header = cc_wrapper_->getCurrentHeader();
      Super::sensorCb(header);
  }
  
  bool PipsEgoCircleTrajectoryController::init()
  {
    Super::init();
    
    cc_wrapper_->init();
    cc_wrapper_->setCallback(boost::bind(&PipsEgoCircleTrajectoryController::generateTrajectories, this));
    
    return true;
  }
  
}
