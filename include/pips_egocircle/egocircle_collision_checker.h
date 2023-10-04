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
      
      EgoCircleCollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name=DEFAULT_NAME, const tf2_utils::TransformManager& tfm=tf2_utils::TransformManager(false)):
        GeneralCollisionChecker(nh,pnh,name,tfm),
        pnh_(pnh, name)
      {
      }
      
      EgoCircleCollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh, const tf2_utils::TransformManager& tfm, const std::string& name=DEFAULT_NAME):
        GeneralCollisionChecker(nh,pnh,name,tfm),
        pnh_(pnh, name)
      {
      }

      void initImpl()
      {
        GeneralCollisionChecker::initImpl();
        ROS_INFO_STREAM("Initializing");
        getCameraModel();
        hallucinated_pub_ = pnh_.advertise<sensor_msgs::LaserScan>("hallucinated_scan", 5);
        
//         octomap_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("octomap",5);
//         visualization_pub_ = pnh_.advertise<visualization_msgs::Marker>("results",500);
//         collision_pub_ = nh_.advertise<PointCloud>("collisions",100);
        
//         reconfigure_server_.reset( new ReconfigureServer(pnh_));
//         reconfigure_server_->setCallback(boost::bind(&OctomapCollisionChecker::reconfigureCB, this, _1, _2));
//         
//         ros::NodeHandle octomap_pnh_(pnh_, "octomap");
//         octomap_reconfigure_server_.reset( new OctomapConstructionReconfigureServer(octomap_pnh_));
//         octomap_reconfigure_server_->setCallback(boost::bind(&OctomapCollisionChecker::octomapReconfigureCB, this, _1, _2));
        
      }
      
      void update(const sensor_msgs::LaserScan::ConstPtr& scan)
      {
        ROS_INFO_STREAM("Updating egocircle scan");
        if(cam_model_)
        {
          cam_model_->setScan(scan);
          cam_model_->update();
          scan_ = scan;
          
//           sensor_msgs::Image::Ptr image_msg = boost::make_shared<sensor_msgs::Image>();
//           image_msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
//           unsigned int data_size = scan->ranges.size()*sizeof(float);
//           image_msg->data.resize(data_size);
//           memcpy(image_msg->data.data(), (char*)(scan->ranges.data()), data_size);
//           
//           PipsCollisionChecker::setImage(image_msg);
        }
        else
        {
          ROS_ERROR("[EgoCircleCollisionChecker]: cam_model_ is NULL!");
        }
      }
      
      ComparisonResult isLessThan(COLUMN_TYPE col)
      {
        ComparisonResult res;
        
        int ind = scan_->ranges.size() - col.rect.tl().x;
        float actual_depth = scan_->ranges[ind];
        
        float model_depth = std::max(col.depth, decltype(col.depth)(0));
        
        if(model_depth > actual_depth)
        {
          res.addPoint(cv::Point(ind,0), actual_depth);
        }
        return res;
      }

      CCResult testCollisionImpl(geometry_msgs::Pose pose, CCOptions options)
      {
        ROS_ASSERT(scan_);
        ROS_ASSERT(cam_model_);
        
        generateHallucinatedScan(pose);
        
        ROS_INFO_STREAM("Collision testing " << ::toString(pose));
        
        //TODO: Implement optimized version of models for egocircle: evaluate depths as calculate indicies
        int img_width = scan_->ranges.size(); // cam_model_->indexer_.size;
        ROS_ASSERT(img_width > 0);
        
        CCResult result;
        auto geometry_models = robot_model_.getModel<EgoCircleGeometryConverter>(pose);
        
        std::vector<CollisionPoint> world_points;
        
        
        for(auto& model : geometry_models)
        {          
          std::vector<float> ranges;
          unsigned int start_ind;
          model->getValues(cam_model_, img_width, ranges, start_ind);
          
          for(unsigned int i = 0; i < ranges.size(); ++i)
          {
            float model_depth = ranges[i];
            
            unsigned int egocircle_ind = (start_ind + i) % img_width;
            float actual_depth = scan_->ranges[egocircle_ind];
            //if(actual_depth>model_depth)
            {
              cv::Point3d ray = cam_model_->projectPixelTo3dRay(cv::Point2d(egocircle_ind,0));
              //cv::Point3d worldPoint = ray * actual_depth;
              cv::Point3d worldPoint = ray * model_depth;
              
              result.addPoint(pips::collision_testing::toCollisionPoint(worldPoint));
              
              if(!options)
              {
                //goto done;
              }
            }
          }
        }
        done:
        
        return result;
      }
      
      sensor_msgs::LaserScan::Ptr generateHallucinatedScan(geometry_msgs::Pose pose)
      {
        ROS_ASSERT(scan_);
        ROS_ASSERT(cam_model_);
        
        ROS_INFO_STREAM("Generating hallucainted egocircle scan: " << ::toString(pose));
        
        //TODO: Implement optimized version of models for egocircle: evaluate depths as calculate indicies
        int img_width = scan_->ranges.size(); // cam_model_->indexer_.size;
        ROS_ASSERT(img_width > 0);
        
        sensor_msgs::LaserScan::Ptr generated_scan = boost::make_shared<sensor_msgs::LaserScan>();
        generated_scan->header = scan_->header;
        generated_scan->angle_min = scan_->angle_min;
        generated_scan->angle_max = scan_->angle_max;
        generated_scan->angle_increment = scan_->angle_increment;
        generated_scan->time_increment = scan_->time_increment;
        generated_scan->scan_time = scan_->scan_time;
        generated_scan->range_min = scan_->range_min;
        generated_scan->range_max = scan_->range_max;
        
        generated_scan->ranges.resize(img_width);
        
        auto geometry_models = robot_model_.getModel<EgoCircleGeometryConverter>(pose);
        
        for(auto& model : geometry_models)
        { 
          std::vector<float> ranges;
          unsigned int start_ind;
          model->getValues(cam_model_, img_width, ranges, start_ind);
          
          for(unsigned int i = 0; i < ranges.size(); ++i)
          {
            float model_depth = ranges[i];
            
            unsigned int egocircle_ind = (start_ind + i) % img_width;
            
            
            //todo: allow overlap, keep largest value
            generated_scan->ranges[egocircle_ind] = model_depth;
            
          }
        }
        
        hallucinated_pub_.publish(generated_scan);
        
        return generated_scan;
      }
      
      static constexpr const char* DEFAULT_NAME="egocircle_collision_checker";

      
    private:
      std::shared_ptr<pips::utils::AbstractCameraModel> getCameraModel()
      {
        cam_model_ = std::make_shared<pips_egocircle::EgoCircleCameraModel>();
        return cam_model_;
      }
      
    private:
      typedef pips::collision_testing::GeneralCollisionChecker GeneralCollisionChecker;
      ros::NodeHandle pnh_;
      
      std::shared_ptr<pips_egocircle::EgoCircleCameraModel> cam_model_;
      ros::Publisher hallucinated_pub_;
      sensor_msgs::LaserScan::ConstPtr scan_;
    };

  
  
}

#endif //PIPS_EGOCIRCLE_COLLISION_CHECKER_H
