#ifndef PIPS_EGOCIRCLE_COLLISION_CHECKER_H
#define PIPS_EGOCIRCLE_COLLISION_CHECKER_H

#include <pips/collision_testing/general_collision_checker.h>
#include <egocircle_utils/updateable_interface.h>

#include <pips_egocircle/egocircle_camera_model.h>
#include <pips_egocircle/egocircle_geometry_converter.h>

#include <pips/collision_testing/robot_models/column_type.h>
#include <pips/utils/image_comparison_result.h>

#include <pips/utils/pose_conversions.h>


namespace pips_egocircle
{

    class EgoCircleCollisionChecker : public pips::collision_testing::GeneralCollisionChecker, public egocircle_utils::UpdateableInterface
    {
    public:
      
      EgoCircleCollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name=DEFAULT_NAME, const tf2_utils::TransformManager& tfm=tf2_utils::TransformManager(false));
      EgoCircleCollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh, const tf2_utils::TransformManager& tfm, const std::string& name=DEFAULT_NAME);
      
      void initImpl();
     
      void update(const sensor_msgs::LaserScan::ConstPtr& scan);
      
      ComparisonResult isLessThan(COLUMN_TYPE col);
      
      CCResult testCollisionImpl(geometry_msgs::Pose pose, CCOptions options);
      
      sensor_msgs::LaserScan::Ptr generateHallucinatedScan(geometry_msgs::Pose pose);
      
      static constexpr const char* DEFAULT_NAME="egocircle_collision_checker";

      
    private:
      std::shared_ptr<pips::utils::AbstractCameraModel> getCameraModel();
      
    private:
      typedef pips::collision_testing::GeneralCollisionChecker GeneralCollisionChecker;
      ros::NodeHandle pnh_;
      
      std::shared_ptr<pips_egocircle::EgoCircleCameraModel> cam_model_;
      ros::Publisher hallucinated_pub_;
      sensor_msgs::LaserScan::ConstPtr scan_;

      boost::mutex scan_mutex_;
    };

  
  
}

#endif //PIPS_EGOCIRCLE_COLLISION_CHECKER_H
