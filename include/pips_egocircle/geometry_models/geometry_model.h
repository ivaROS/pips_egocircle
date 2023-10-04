#ifndef PIPS_EGOCIRCLE_GEOMETRY_MODELS_H
#define PIPS_EGOCIRCLE_GEOMETRY_MODELS_H

#include <pips/collision_testing/geometry_models/generic_models.h>
#include <pips/utils/abstract_camera_model.h>

//this is just included to include a bunch of other stuff until I figure out exactly what is needed
#include <pips/collision_testing/robot_models/hallucinated_robot_model.h>

namespace pips_egocircle
{
  namespace geometry_models
  {
    class GeometryModel
    {
      private:
      
      public:
        const std::shared_ptr<const pips::collision_testing::geometry_models::GenericGeometryModel> source_;
        geometry_msgs::Pose pose_;
        int type_id_;
        
        GeometryModel(const std::shared_ptr<const pips::collision_testing::geometry_models::GenericGeometryModel>& source, const geometry_msgs::Pose& pose):
          source_(source),
          pose_(pose),
          type_id_(source->type_id_)
        {}
          
        virtual void getValues(const std::shared_ptr<const pips::utils::AbstractCameraModel>& cam_model_, unsigned int img_width, std::vector<float>& ranges, unsigned int& start_ind) const=0;
    };
  }
}

#endif //PIPS_EGOCIRCLE_GEOMETRY_MODELS_H
