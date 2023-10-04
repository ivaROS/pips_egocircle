

#ifndef PIPS_EGOCIRCLE_GEOMETRY_MODELS_BOX_H
#define PIPS_EGOCIRCLE_GEOMETRY_MODELS_BOX_H

#include <pips/utils/pose_conversions.h>
#include <pips_egocircle/geometry_models/geometry_model.h>
#include <pips/collision_testing/geometry_models/box.h>

#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"


#include <ros/console.h> //Needed for logging
#include <ros/assert.h> //Needed for 'ROS_BREAK and ROS_ASSERT

namespace pips_egocircle
{
  namespace geometry_models
  {
    

  class Box : public GeometryModel
  {
  public:
    double length_, width_, height_;
    
  
    static cv::Point3d quaternionToRPY(const geometry_msgs::Quaternion& quaternion)
    {
        // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
        tf::Quaternion quat;
        tf::quaternionMsgToTF(quaternion, quat);

        // the tf::Quaternion has a method to acess roll pitch and yaw
        double roll, pitch, yaw;
        cv::Point3d point;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        
        point.x = roll;
        point.y = pitch;
        point.z = yaw;

        return point;
    }
    
public:
  Box(const std::shared_ptr<const pips::collision_testing::geometry_models::Box>& source, const geometry_msgs::Pose& pose):
    GeometryModel(source, pose),
    length_(source->length_),
    width_(source->width_),
    height_(source->height_)
  {}
  
  virtual void getValues(const std::shared_ptr<const pips::utils::AbstractCameraModel>& cam_model_, unsigned int img_width, std::vector<float>& ranges, unsigned int& start_ind) const
  {
    ROS_ASSERT(length_>0 && width_ > 0 && height_>0);
    
    ROS_DEBUG_STREAM("Get columns for " << pose_);
        
    //TODO make it work without this or using class parameters
    double distance_from_rear, hrw, rd, fd;//hrl is half of the robot length, hrw is half of the robot width, and fd and rd are distance from front and rear
    
    distance_from_rear = length_/2; // half way for now.
    
    rd = distance_from_rear;
    //hrl = robot_length/2;
    hrw = width_/2;
    fd = length_ - rd;
    
    
    //unsigned int left = 0;
    //unsigned int right = img_width;
    
    cv::Point3d pt;
    convertPose(pose_, pt);
     
    double h_squared = pt.x*pt.x + pt.y*pt.y;
    
    double h = std::sqrt(h_squared);
    
   
    cv::Point3d pt2 = quaternionToRPY(pose_.orientation);
    double ang = pt2.z; //Accessing yaw value since orientation is expected to not be transformed
    
    //Reduces # of possible scenarios
//     if(ang > M_PI/2)
//     {
//       ang -= M_PI;
//     }
//     else if(ang < M_PI/2)
//     {
//       ang += M_PI;
//     }
    
    ROS_DEBUG_STREAM("roll, pitch, and yaw" << pt2);
    
    double theta1 = std::atan2(hrw,fd); 
    double theta2 = std::atan2(hrw,rd); 
    
    ROS_DEBUG_STREAM("theta1: " << theta1 << ", theta2: " << theta2);
    
    //double theta1 = std::atan2(hrw,fd); 
    //double theta2 = std::atan2(hrw,rd); 
    
    double t1, t2, t3, t4, t5, hd1, hd2;
        
    t1 = theta1 + ang;
    t2 = M_PI - theta1 + ang;
    t3 = M_PI + theta2 + ang;
    t4 = 2*M_PI - theta2 + ang;
    t5 = std::atan2(pt.y, pt.x);
    
//    corner 2.                      corner 1.
//     
//     
//     
//     
//     corner 3.                     corner 4.
    
    //hd is the hypotenuse-distance from the base frame to the corners of the car
    
    hd1  = std::sqrt(fd*fd + hrw*hrw);// from the base frame to corner 1 and 2
    hd2  = std::sqrt(rd*rd + hrw*hrw);// from the base frame to corner 3 and 4
    
    //TODO this distance is to check if the camera is not in the robot. May not be needed or may need aditional distance to be added.
    double distance = rd/std::cos(t5);   
    
    cv::Point3d p1, p2, p3, p4, p5, p6, p7, p8;
    
    // We only calculate the actual side borders of the robot if it is far enough away that they could be seen.
    if(h > distance && (pt.x - hd1) > 0)
    {
        //TODO remove the debug message
        ROS_DEBUG_STREAM("H was greater!!" << h);
        
        //corner 1
        p1.x = pt.x + hd1*std::cos(t1); 
        p1.y = pt.y + hd1*std::sin(t1);
        
        //corner 2
        p2.x = pt.x + hd1*std::cos(t2); 
        p2.y = pt.y + hd1*std::sin(t2);
        
        //corner 3
        p3.x = pt.x + hd2*std::cos(t3); 
        p3.y = pt.y + hd2*std::sin(t3);
        
        //corner 4
        p4.x = pt.x + hd2*std::cos(t4); 
        p4.y = pt.y + hd2*std::sin(t4);

    }
    else
    {
      return;
    }
        
    std::vector<cv::Point3d> robot_corners = {p3,p4,p1,p2};
    int num_points = int(robot_corners.size());
    
    std::vector<cv::Point2d> corners_2d;
    for(auto p : robot_corners)
    {
      corners_2d.push_back(cv::Point2d(p.x,p.y));
    }
    
    // With the current indexer, these could be ints
    std::vector<double> projected_points;
    for(auto p : robot_corners)
    {
      auto pp = cam_model_->project3dToPixel(p);
      projected_points.push_back(pp.x);
    }
    
    std::vector<bool> is_visible;
    std::vector<cv::Point2d> normals;
    for(int i = 0; i < num_points; ++i)
    {
      int next_i = (i+1)%num_points;
      auto v_side = corners_2d[next_i] - corners_2d[i];
      cv::Point2d v_norm(v_side.y, -v_side.x);
      auto center = corners_2d[i]+v_side/2;
      normals.push_back(v_norm);
      bool visible = (center.dot(v_norm) > 0);
      is_visible.push_back(visible);
      
      ROS_DEBUG_STREAM("i=" << i << ", p1=" << corners_2d[i] << "[" << projected_points[i] << "], p2=" << corners_2d[next_i] << "[" << projected_points[next_i] << "], visible=" << visible);
    }
    
    int start_face_ind;
    for(start_face_ind= 0; start_face_ind < num_points; ++start_face_ind)
    {
      int prev_i = (start_face_ind-1)%num_points;
      prev_i = prev_i >= 0 ? prev_i : prev_i + num_points;
      if(is_visible[start_face_ind] && !is_visible[prev_i])
      {
        break;
      }
    }
    
    if(start_face_ind == num_points)
    {
      ROS_ERROR_STREAM("Didn't find visibility condition!");
      return;
    }
    ROS_DEBUG_STREAM("start_face_ind=" << start_face_ind);
    
    int range_ind=0;
    for(int i = start_face_ind; is_visible[i % num_points]; i++)
    {
      double point_ind_start = projected_points[i % num_points];
      double point_ind_end = projected_points[(i+1) % num_points];
      
      cv::Point2d face1(corners_2d[i % num_points]);
      cv::Point2d face2(corners_2d[(i+1) % num_points]);
      
      ROS_DEBUG_STREAM("i=" << (i % num_points) << ", " ", point_ind_start=" << point_ind_start << ", point_ind_end=" << point_ind_end << ", face1=" << face1 << ", face2=" << face2);
        
      if(i==start_face_ind)
      {
        start_ind = point_ind_start;
      }
      
      for(int p_x = int(round(point_ind_start)); p_x != int(round(point_ind_end)); ++p_x, ++range_ind)
      {
        p_x = p_x % img_width;
        cv::Point2d intersection_pt;
        cv::Point3d ray = cam_model_->projectPixelTo3dRay(cv::Point2d(p_x,0));
        cv::Point2d ray2(ray.x,ray.y);
        cv::Point2d ray1;
        
        bool intersects = getIntersection(face1, face2, ray1, ray2, intersection_pt);
        ROS_DEBUG_STREAM("p_x=" << p_x << ", ray=" << ray2 << ", intersects?=" << intersects << ", intersection_pt=" << intersection_pt);
        if(intersects)  //Should (almost?) always be true
        {
          float range = cam_model_->getPixelValue(cv::Point3d(intersection_pt.x, intersection_pt.y, 0));
          ranges.push_back(range);
        }
      }
    }
    
  }
  
  bool getIntersection(cv::Point2d f1, cv::Point2d f2, cv::Point2d r1, cv::Point2d r2, cv::Point2d& intersection) const
  {
    cv::Point2d face=f2-f1;
    cv::Point2d ray=-(r2-r1);
    
    cv::Matx22d m(face.x, ray.x, face.y, ray.y);
    
/*   
    Eigen::Matrix2d m;
    m.col(0) = p4-p3;
    m.col(1) = -(p2-p1);*/
    
    double eps = .01;
    
    //Eigen::Vector2d res = m.inverse() * (p1-p3);
    cv::Point2d res = m.inv() * (r1-f1);
    
    ROS_DEBUG_STREAM("Res=" << res);
    
    //ROS_INFO_STREAM_NAMED("[intersects]","Intersection results: (" << p3.x() << "," << p3.y() << ") : (" << p4.x() << "," << p4.y() << ") = " << res.x() << ", " << res.y());
    if(res.x>=-eps && res.x < 1+eps)
    {
      intersection = r1 - res.y*ray;
      return true;
    }
    return false;
  }
  

/*  
  COLUMN_TYPE getIntersection(const std::shared_ptr<const pips::utils::AbstractCameraModel>& cam_model_, cv::Point3d p1, cv::Point3d p2, cv::Point3d p3, cv::Point2d pix, int img_width, int img_height) const
  {
        //TODO: finding the normal and passing it might be faster
        cv::Point3d normal, p12, p13;
        p12 = p1 - p2;
        p13 = p1 - p3;
        
        normal.x = p12.y*p13.z - p12.z*p13.y;
        normal.y = p12.z*p13.x - p12.x*p13.z;
        normal.z = p12.x*p13.y - p12.y*p13.x;
        
        COLUMN_TYPE col;

        cv::Point3d ray = cam_model_->projectPixelTo3dRay(pix);
        //@ Justin I think that the pt.y can be 0 without effect
        
        //equations for intersection of ray and plane
        double numera = p1.x*normal.x + p1.y*normal.y + p1.z*normal.z;
        double denom = ray.x*normal.x + ray.y*normal.y + ray.z*normal.z;
        double t;
         if (denom > 1e-6)
        {
            t = numera/denom;
        }
        else
        {
            ROS_DEBUG_STREAM("Very small denominator!"<< "p_x="<< pix.x << "ray=" << ray);
            //TODO find a better way. 
            //can I skipp this column?
            //small t means the palne and the ray are close to being parallel 
            t = 1e-6;
        }
        
        //Get world coordinates of intersection
        cv::Point3d intersection = ray*t;
        intersection.y = p1.y;
        

        float depth = cam_model_->getPixelValue(intersection);
      
        col = getColumn(p_xht,p_xhb,depth, img_width, img_height);
        
        return col;
        
  }*/



    };
    }
    
    }
    
#endif //PIPS_EGOCIRCLE_GEOMETRY_MODELS_BOX_H
