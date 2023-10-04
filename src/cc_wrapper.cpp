#include <pips_egocircle/egocircle_cc_wrapper.h>

namespace pips_egocircle
{

  EgoCircleCCWrapper::EgoCircleCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, const tf2_utils::TransformManager& tfm, const std::string& name) :
    PipsCCWrapper(nh,pnh,name,tfm),
    InterfaceUpdater(nh, pnh, name, tfm)
  {
    cc_ = std::make_shared<EgoCircleCollisionChecker>(nh, pnh, PipsCCWrapper::tfm_);
    InterfaceUpdater::setInterface(cc_);
  }

  EgoCircleCCWrapper::EgoCircleCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name, const tf2_utils::TransformManager& tfm) :
    PipsCCWrapper(nh,pnh,name,tfm),
    InterfaceUpdater(nh, pnh, name, tfm)
  {
    cc_ = std::make_shared<EgoCircleCollisionChecker>(nh, pnh, PipsCCWrapper::tfm_);
    InterfaceUpdater::setInterface(cc_);
  }
  
  bool EgoCircleCCWrapper::init()
  {
    return InterfaceUpdater::init() && PipsCCWrapper::init();
  }
  
  void EgoCircleCCWrapper::scanCb(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
  {
    InterfaceUpdater::scanCb(scan_msg);
    doCallback();
  }

} //end namespace pips_egocircle
