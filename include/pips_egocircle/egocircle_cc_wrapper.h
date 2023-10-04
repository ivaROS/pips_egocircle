#ifndef PIPS_EGOCIRCLE_CC_WRAPPER_H
#define PIPS_EGOCIRCLE_CC_WRAPPER_H

#include <pips_trajectory_testing/pips_cc_wrapper.h>
#include <egocircle_utils/interface_updater.h>
#include <pips_egocircle/egocircle_collision_checker_compact.h>

namespace pips_egocircle
{

  class EgoCircleCCWrapper: public pips_trajectory_testing::PipsCCWrapper, public egocircle_utils::InterfaceUpdater
  {
  public:
    EgoCircleCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name=DEFAULT_NAME, const tf2_utils::TransformManager& tfm=tf2_utils::TransformManager(false));
    
    EgoCircleCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, const tf2_utils::TransformManager& tfm, const std::string& name=DEFAULT_NAME);
    
    std::shared_ptr<pips::collision_testing::TransformingCollisionChecker> getCC()
    {
      return cc_;
    }
    
    bool isReadyImpl()
    {
      return InterfaceUpdater::isReady();
    }
    
    bool init();
    
    void update()
    {
      InterfaceUpdater::update();
    }
    
    std_msgs::Header getCurrentHeader()
    {
      return InterfaceUpdater::getCurrentHeader();
    }
    
  protected:
    virtual void scanCb(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    
  private:
    static constexpr const char* DEFAULT_NAME="egocircle_cc_wrapper";
    std::shared_ptr<EgoCircleCollisionChecker> cc_;
    
    
    using PipsCCWrapper=pips_trajectory_testing::PipsCCWrapper;
    using InterfaceUpdater=egocircle_utils::InterfaceUpdater;
  };

} //end namespace pips_egocircle

#endif //PIPS_EGOCIRCLE_CC_WRAPPER_H
