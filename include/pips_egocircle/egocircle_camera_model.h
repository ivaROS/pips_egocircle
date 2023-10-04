#ifndef PIPS_EGOCIRCLE_CAMERA_MODEL_H
#define PIPS_EGOCIRCLE_CAMERA_MODEL_H


#include <pips/utils/abstract_camera_model.h>
//#include <opencv2/core/core.hpp>
#include <sensor_msgs/LaserScan.h>
// #include <egocircle_utils/container.h>
#include <egocircle/ego_circle.h>
#include <cmath>

namespace pips_egocircle
{

//   struct EgoCircleIndexToRay
//   {
//     int size;
//     float scale;
//     
//     EgoCircleIndexToRay() {}
//     
//     EgoCircleIndexToRay(int size):
//       size(size),
//       scale(2*std::acos(-1)/size)
//     {}
//       
//     cv::Point3d projectPixelTo3dRay(int ind)
//     {
//       float angle = (ind - size/2) * scale;
//       cv::Point3d point(std::cos(angle),std::sin(angle),0);
//       return point;
//     }
//       
//   };
    
  class EgoCircleCameraModel : public pips::utils::AbstractCameraModel
  {
    private:
      //egocircle_utils::Container model_;
      //EgoCircleIndexToRay ray_model_;
      ego_circle::EgoCircleIndexer indexer_;
      //sensor_msgs::LaserScan::ConstPtr scan_msg_;
      int new_size_;
      
    public:
      
      void setScan(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
      {
        new_size_ = scan_msg->ranges.size();
      }
      
      void update()
      {
        indexer_ = ego_circle::EgoCircleIndexer(new_size_);
        //ray_model_ = EgoCircleIndexToRay(size_);
      }
      
      cv::Point2d project3dToPixel(const cv::Point3d& point) const
      {
        ego_circle::EgoCircularPoint pt(point.x, point.y);
        float ind;
        indexer_.getIndex(pt, ind);
        cv::Point2d cvpt(ind,0);
        ROS_DEBUG_STREAM_NAMED("project3dToPixel", "Point in: (" << point.x << "," << point.y << "), index: " << ind << ", point out: " << cvpt);
        return cvpt;
      }
      
      cv::Point3d projectPixelTo3dRay(const cv::Point2d& point) const
      {
        float angle = (point.x - indexer_.size/2) / indexer_.scale;
        cv::Point3d pt(std::cos(angle),std::sin(angle),0);
        ROS_DEBUG_STREAM_NAMED("projectPixelTo3dRay", "Point in: (" << point.x << "," << point.y << "), angle: " << angle << ", point out: " << pt);
        return pt;
      }
      
      std::vector<int> getColumnRange(int left, int right) const
      {          
        std::vector<int> cols;
        
        if(left >=right)
        {
          for(int i = right; i < left; ++i)
          {
            cols.push_back(i);
          }
        }
        else
        {
          for(int i = right; i < int(indexer_.size); ++i)
          {
            cols.push_back(i);
          }
          for(int i = 0; i < left; ++i)
          {
            cols.push_back(i);
          }
        }
        
        return cols;
      }
      
      float getPixelValue(const cv::Point3d& point) const
      {
        ego_circle::EgoCircularPoint pt(point.x, point.y);
        ego_circle::PolarPoint p(pt);
        return p.r;
      }
    
      cv::Size getImageSize() const
      {   
          return cv::Size(new_size_, 1);
      }
  };


}


#endif  // PIPS_EGOCIRCLE_CAMERA_MODEL_H
