#ifndef PIPS_EGOCIRCLE_GEOMETRY_CONVERTER_H
#define PIPS_EGOCIRCLE_GEOMETRY_CONVERTER_H

#include <pips_egocircle/geometry_models/geometry_model.h>
#include <pips_egocircle/geometry_models/cylinder.h>
#include <pips_egocircle/geometry_models/box.h>
#include <memory>

namespace pips_egocircle
{
class EgoCircleGeometryConverter
{
public:
  using geometry_type = std::shared_ptr<pips_egocircle::geometry_models::GeometryModel>;
    
  static geometry_type convert(const std::shared_ptr<const pips::collision_testing::geometry_models::GenericGeometryModel>& model, const geometry_msgs::Pose& model_pose)
  {
    ROS_DEBUG_STREAM("Model name: " << model->name_ << ", type_id: " << model->type_id_);
    
    geometry_type ptr;
    
    //TODO: Move the switch to higher level class and just implement the necessary conversion functions. ex: <geometry_type, pips::collision_testing::geometry_models::Cylinder>
    switch(model->type_id_)
    {
      case 0:
        ptr = std::make_shared<pips_egocircle::geometry_models::Cylinder>(std::static_pointer_cast<const pips::collision_testing::geometry_models::Cylinder>(model), model_pose);
        break;
      case 1:
        ptr = std::make_shared<pips_egocircle::geometry_models::Box>(std::static_pointer_cast<const pips::collision_testing::geometry_models::Box>(model), model_pose);
        break;
      default:
        ROS_WARN_STREAM("WARNING! Attempted to convert a geometry model (" << model->type_id_ << ") for which no conversion is known!");
    }
    return ptr;
  }


};


} //namespace pips_egocircle

#endif //PIPS_EGOCIRCLE_GEOMETRY_CONVERTER_H
