#ifndef PIPS_EGOCIRCLE_GEOMETRY_MODELS_CYLINDER_H
#define PIPS_EGOCIRCLE_GEOMETRY_MODELS_CYLINDER_H

//#include <sensor_msgs/Image.h>
//#include <geometry_msgs/TransformStamped.h>

//#include <opencv/cv.h>

#include <pips_egocircle/geometry_models/geometry_model.h>
#include <pips/collision_testing/geometry_models/cylinder.h>

#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/highgui/highgui.hpp"


//#include <Eigen/Eigen>
//#include <image_transport/image_transport.h>
//#include <image_geometry/pinhole_camera_model.h>
//#include <cv_bridge/cv_bridge.h>

//#include <iomanip>      // std::setprecision

//#include "ros/ros.h" //Needed for 'ROS_BREAK and ROS_ASSERT
#include <ros/console.h> //Needed for logging
#include <ros/assert.h> //Needed for 'ROS_BREAK and ROS_ASSERT


namespace pips_egocircle
{
  namespace geometry_models
  {

    class Cylinder : public GeometryModel
    {
    public:
        double radius_=-1, height_=-1;
        
    public:
      Cylinder(const std::shared_ptr<const pips::collision_testing::geometry_models::Cylinder>& source, const geometry_msgs::Pose& pose):
        GeometryModel(source, pose),
        radius_(source->radius_),
        height_(source->height_)
      {}

      virtual void getValues(const std::shared_ptr<const pips::utils::AbstractCameraModel>& cam_model_, unsigned int img_width, std::vector<float>& ranges, unsigned int& start_ind) const
      {    
        const cv::Point3d pt(pose_.position.x, pose_.position.y, pose_.position.z);
        
        ROS_DEBUG_STREAM("Get columns for " << pt << ", with height=" << height_ << ", radius=" << radius_ <<", img_width=" << img_width);
        
        unsigned int left = img_width;
        unsigned int right = 0;
        
        double h_squared = pt.x*pt.x + pt.y*pt.y;
        double h = std::sqrt(h_squared);
        
        cv::Point3d Xc_l, Xc_r;
        
        // We only calculate the actual side borders of the robot if it is far enough away that they could be seen
        ROS_DEBUG_STREAM("Distance of cylinder origin: " << h << ", radius: " << radius_);
        if(h > radius_ && (pt.x - radius_) > 0)
        {
          double tangentDist = std::sqrt(h_squared - radius_*radius_);
          
          double theta_c = std::atan2(pt.y,pt.x);
          double theta_d = std::asin(radius_/h);
          
          
          Xc_l = cv::Point3d(tangentDist*std::cos(theta_c + theta_d), tangentDist*std::sin(theta_c + theta_d), 0);
          Xc_r = cv::Point3d(tangentDist*std::cos(theta_c - theta_d), tangentDist*std::sin(theta_c - theta_d), 0);
        }
        else
        {
          return;
        }

        
        {
          cv::Point2d p_l = cam_model_->project3dToPixel(Xc_l);
          cv::Point2d p_r = cam_model_->project3dToPixel(Xc_r);
          
          //This is slightly relaxed, but safety expansion should compensate for it
          left = std::floor(p_l.x);
          right = std::ceil(p_r.x);
        }
        
        ROS_DEBUG_STREAM("[getValues]: pt=" << pt << ", h=" << h << ", Xc_l=" << Xc_l << ", left=" << left << ", Xc_r=" << Xc_r << ", right=" << right);

        start_ind = right;
        
        for(int p_x : cam_model_->getColumnRange(left,right))
        {

          cv::Point3d ray = cam_model_->projectPixelTo3dRay(cv::Point2d(p_x,0));
          
          
          //equations for intersection of ray and circle
          double a = ray.y*ray.y + ray.x*ray.x;
          double b = -2*(ray.y*pt.y + ray.x*pt.x);
          double c = h_squared - radius_*radius_;
          
          if(b*b - 4*a*c <0)
          {
            //ROS_ERROR_STREAM("complex solution! Left=" << left << ", right=" << right << ", p_x="<< p_x << ", ray=" << ray << ", h_squared="<< h_squared );
            //ROS_BREAK();
            continue;
          }
          else
          {
            ROS_DEBUG_STREAM("Left=" << left << ", right=" << right << ", p_x="<< p_x << ", ray=" << ray << ", h_squared="<< h_squared );
          }
          
          //This likely does nothing given the above condition checks
          ROS_ASSERT_MSG(b*b - 4*a*c >=0, "Complex solution for ray-circle intersection!");

          /*
          Solve for parameter t that yields intersection
          Note that we only care about the more distant intersection (the + solution)
          */
          double t = (-b + std::sqrt(b*b-4*a*c))/(2*a);
          
          //Get world coordinates of intersection
          cv::Point3d X_h = ray*t;

          float depth = cam_model_->getPixelValue(X_h);
          ranges.push_back(depth);
        }

      }
      
    };
  } //end namespace geometry_models
} //end namespace pips_egocircle
  

#endif //PIPS_EGOCIRCLE_GEOMETRY_MODELS_CYLINDER_H

