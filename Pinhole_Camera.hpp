//
// Created by Göksu Güvendiren on 2019-05-14.
//

#pragma once

#include <vector>
#include "Vector.hpp"
#include "Camera.hpp"

#include "global.hpp"

#include "BVH.hpp"
#include "Ray.hpp"

class PinholeCamera : public Camera
{
    Vector3f pos_;
    Vector3f dir_;

    float focal_film_width_  = 1;
    float focal_film_height_ = 1;

    float area_focal_film_ = 1;
    float area_lens_       = 1;

    float lens_radius_ = 0;
    float focal_distance_ = 1;

    struct Params
    {
        float film_aspect    = 1;
        Vector3f pos;
        Vector3f dst;
        Vector3f up;
        float fov            = 0;
//        float lens_radius    = 0;
        float focal_distance = 0;

    } params_;

    void init_from_params(const Params &params)
    {
        const float aspect = params.film_aspect;

        focal_film_height_ = 2 * params.focal_distance
                               * std::tan(params.fov / 2);
        focal_film_width_ = aspect * focal_film_height_;

        area_focal_film_ = focal_film_width_ * focal_film_height_;

//        camera_to_world_ = FTransform3(
//            FTrans4::look_at(
//                params.pos, params.dst, params.up)).inv();

        pos_ = params.pos;
        dir_ = (params.dst - params.pos).normalized();

        focal_distance_ = params.focal_distance;
    }

    public:

        PinholeCamera(
               float film_aspect,
               const Vector3f &pos, const Vector3f &dst, const Vector3f &up,
               float fov,   float focal_distance)
           {
               params_.film_aspect    = film_aspect;
               params_.pos            = pos;
               params_.dst            = dst;
               params_.up             = up;
               params_.fov            = fov;
//               params_.lens_radius    = lens_radius;
               params_.focal_distance = focal_distance;
               init_from_params(params_);
           }
      /* CameraSampleWeResult sample_we(
             const Vector2f &film_coord ) const noexcept override
        {
            return  ;
        }*/
       CameraEvalWeResult eval_we(
        const Vector3f &pos_on_cam,  const Vector3f &pos_to_out)  const noexcept override
        {
           Vector3f  _dir = pos_to_out ;
         if(_dir.z <= 0)
            return CAMERA_EVAL_WE_RESULT_ZERO;
           Vector3f focal_film_pos = pos_ + (focal_distance_ / _dir.z) * _dir;
           Vector2f film_coord(
                            float(0.5) - focal_film_pos.x / focal_film_width_,
                            float(0.5) + focal_film_pos.y / focal_film_height_
                        );
         const float cos_theta  = _dir.z;
         const float cos2_theta = cos_theta * cos_theta;
         const float we         = focal_distance_ * focal_distance_
                               / (area_focal_film_  * cos2_theta * cos2_theta);
         return { Vector3f(we), film_coord, dir_ };
       }

      CameraSampleWiResult sample_wi(
        const Vector3f &ref ) const noexcept override
      {
          Vector3f  _dir = (ref - pos_).normalized();
        if(_dir.z <= 0)
                    return CAMERA_SAMPLE_WI_RESULT_INVALID;
          Vector3f focal_film_pos = pos_
                                          + (focal_distance_ / _dir.z) * _dir;
          Vector2f film_coord(
                   float(0.5) - focal_film_pos.x / focal_film_width_,
                   float(0.5) + focal_film_pos.y / focal_film_height_
               );
          Vector3f ref_to_pos = pos_ - ref;
          Vector3f we = eval_we(pos_, -ref_to_pos.normalized()).we;
        const float pdf = (pos_ - ref).norm() / (_dir.z );

        return CameraSampleWiResult(
            pos_, dir_, ref_to_pos, we, pdf, film_coord);
      }

      void apply_image_filter(
          const Vector2f &pixel_range, float filter_radius,
          const Vector2f &sample,std::vector<Vector3f> *image,Vector3f& texel )
      {
          const int x_min = (std::max)(pixel_range.x,
               (std::ceil(sample.x - filter_radius - float(0.5))));
          const int y_min = (std::max)(pixel_range.y,
               (std::ceil(sample.y - filter_radius - float(0.5))));
          const int x_max = (std::min)(pixel_range.x,
               (std::floor(sample.x + filter_radius - float(0.5))));
          const int y_max = (std::min)(pixel_range.y,
               (std::floor(sample.y + filter_radius - float(0.5))));

          float y_rel = std::abs(y_min + float(0.5) - sample.y);
          for(int y = y_min; y <= y_max; ++y)
          {
               y_rel += 1;  ;
              if(y_rel > filter_radius)
                  continue;

              float x_rel = std::abs(x_min + float(0.5) - sample.x);
              for(int x = x_min; x <= x_max; ++x)
              {
                   x_rel += 1;  ;
                  if(x_rel > filter_radius)
                      continue;
                   int index = x*y;
                   (*image)[index]  =(*image)[index]+   texel;

              }
          }
      }
};