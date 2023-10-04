
#include <pips_egocircle/egocircle_collision_checker_compact.h>

namespace pips_egocircle
{
      
      EgoCircleCollisionChecker::EgoCircleCollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name, const tf2_utils::TransformManager& tfm):
        GeneralCollisionChecker(nh,pnh,name,tfm),
        pnh_(pnh, name)
      {
        ROS_DEBUG_STREAM_NAMED(name_+".constructor", "Constructing collision checker");
        robot_model_ = std::make_shared<pips::collision_testing::robot_models::RobotModel>(nh, pnh, tfm);
      }
      
      EgoCircleCollisionChecker::EgoCircleCollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh, const tf2_utils::TransformManager& tfm, const std::string& name):
        GeneralCollisionChecker(nh,pnh,name,tfm),
        pnh_(pnh, name)
      {
        ROS_DEBUG_STREAM_NAMED(name_+".constructor", "Constructing collision checker");
        robot_model_ = std::make_shared<pips::collision_testing::robot_models::RobotModel>(nh, pnh, tfm);
      }

      void EgoCircleCollisionChecker::initImpl()
      {
        GeneralCollisionChecker::initImpl();
        ROS_INFO_STREAM("Initializing [" << name_ << "]");
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
      
      void EgoCircleCollisionChecker::update(const sensor_msgs::LaserScan::ConstPtr& scan)
      {
        ROS_DEBUG_STREAM("Updating egocircle scan");
        if(cam_model_)
        {
          boost::mutex::scoped_lock lock(scan_mutex_);
          
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
      
 
      sensor_msgs::LaserScan::Ptr EgoCircleCollisionChecker::generateHallucinatedScan(geometry_msgs::Pose pose)
      {
        ROS_ASSERT(scan_);
        ROS_ASSERT(cam_model_);
        
        ROS_DEBUG_STREAM("Generating hallucinated egocircle scan: " << ::toString(pose));
        
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
        
        auto geometry_models = robot_model_->getModel<EgoCircleGeometryConverter>(pose);
        
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

      std::shared_ptr<pips::utils::AbstractCameraModel> EgoCircleCollisionChecker::getCameraModel()
      {
        cam_model_ = std::make_shared<pips_egocircle::EgoCircleCameraModel>();
        return cam_model_;
      }
}

