

#include <pips_egocircle/egocircle_collision_checker_compact.h>

namespace pips_egocircle
{
      
    ComparisonResult EgoCircleCollisionChecker::isLessThan(COLUMN_TYPE col)
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

    CCResult EgoCircleCollisionChecker::testCollisionImpl(geometry_msgs::Pose pose, CCOptions options)
    {
      ROS_ASSERT(scan_);
      ROS_ASSERT(cam_model_);
      
      if(hallucinated_pub_.getNumSubscribers()>0)
      {
        generateHallucinatedScan(pose);
      }
      
      ROS_DEBUG_STREAM("Collision testing " << ::toString(pose));

      boost::mutex::scoped_lock lock(scan_mutex_);
      
      //TODO: Implement optimized version of models for egocircle: evaluate depths as calculate indicies
      int img_width = scan_->ranges.size(); // cam_model_->indexer_.size;
      ROS_ASSERT(img_width > 0);
      
      CCResult result;
      auto geometry_models = robot_model_->getModel<EgoCircleGeometryConverter>(pose);
      
      std::vector<CollisionPoint> world_points;
      
      //TODO: use egocircle_utils::Container to make accessing points a bit cleaner
      const float egocircle_radius = scan_->range_max-ego_circle::EgoCircleROS::OFFSET;
      
      
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

          if(actual_depth < model_depth && actual_depth < egocircle_radius)
          {
            cv::Point3d ray = cam_model_->projectPixelTo3dRay(cv::Point2d(egocircle_ind,0));
            cv::Point3d worldPoint = ray * actual_depth;
            //cv::Point3d worldPoint = ray * model_depth;
            
            result.addPoint(pips::collision_testing::toCollisionPoint(worldPoint));
            
            if(!options)
            {
              //goto done;
            }
          }
        }
      }
      //done:
      
      return result;
    }
      
}
