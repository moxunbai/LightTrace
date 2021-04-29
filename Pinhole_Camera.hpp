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


    struct Params
    {
        real film_aspect    = 1;
        Vector3f pos;
        Vector3f dst;
        Vector3f up;
        real fov            = 0;
        real lens_radius    = 0;
        real focal_distance = 0;

    } params_;

    void init_from_params(const Params &params)
    {
        const real aspect = params.film_aspect;

        focal_film_height_ = 2 * params.focal_distance
                               * std::tan(params.fov / 2);
        focal_film_width_ = aspect * focal_film_height_;

        area_focal_film_ = focal_film_width_ * focal_film_height_;

//        camera_to_world_ = FTransform3(
//            FTrans4::look_at(
//                params.pos, params.dst, params.up)).inv();

        pos_ = params.pos;
        dir_ = (params.dst - params.pos).normalized();

        lens_radius_ = params.lens_radius;
        focal_distance_ = params.focal_distance;
    }

    public:

        PinholeCamera(
               real film_aspect,
               const Vector3f &pos, const Vector3f &dst, const Vector3f &up,
               real fov,   real focal_distance)
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
       CameraSampleWeResult sample_we(
             const Vector2f &film_coord ) const noexcept override
        {

        }
       CameraEvalWeResult eval_we(
        const Vector3f &pos_on_cam, const Vector3f &pos_to_out)  const noexcept override
        {
         const Vector3f  _dir = pos_to_out.normalized();
         if(_dir.z <= 0)
            return CAMERA_SAMPLE_WI_RESULT_INVALID;
         const Vector3f focal_film_pos = pos_ + (focal_distance_ / _dir.z) * _dir;
         const Vector2f film_coord(
                            real(0.5) - focal_film_pos.x / focal_film_width_,
                            real(0.5) + focal_film_pos.y / focal_film_height_
                        );
         const real cos_theta  = _dir.z;
         const real cos2_theta = cos_theta * cos_theta;
         const real we         = focal_distance_ * focal_distance_
                               / (area_focal_film_  * cos2_theta * cos2_theta);
         return { Vector3f(we), film_coord, dir_ };
       }

      CameraSampleWiResult sample_wi(
        const Vector3f &ref ) const noexcept override
      {
        const Vector3f  _dir = (ref - pos_).normalized();
        if(_dir.z <= 0)
                    return CAMERA_SAMPLE_WI_RESULT_INVALID;
        const Vector3f focal_film_pos = pos_
                                          + (focal_distance_ / _dir.z) * _dir;
        const Vector2f film_coord(
                   real(0.5) - focal_film_pos.x / focal_film_width_,
                   real(0.5) + focal_film_pos.y / focal_film_height_
               );
        const Vector3f ref_to_pos = pos_ - ref;
        const Vector3f we = eval_we(pos_, -ref_to_pos).we;
        const real pdf = (pos_ - ref).norm() / (_dir.z );

        return CameraSampleWiResult(
            pos_, dir_, ref_to_pos, we, pdf, film_coord);
      }

      void apply_image_filter(
          const Vector2f &pixel_range, real filter_radius,
          const Vector2f &sample,std::vector<Vector3f> *image,Vector3f& texel )
      {
          const int x_min = (std::max)(pixel_range.x,
              static_cast<int>(std::ceil(sample.x - filter_radius - real(0.5))));
          const int y_min = (std::max)(pixel_range.y,
              static_cast<int>(std::ceil(sample.y - filter_radius - real(0.5))));
          const int x_max = (std::min)(pixel_range.x,
              static_cast<int>(std::floor(sample.x + filter_radius - real(0.5))));
          const int y_max = (std::min)(pixel_range.y,
              static_cast<int>(std::floor(sample.y + filter_radius - real(0.5))));

          real y_rel = std::abs(y_min + real(0.5) - sample.y);
          for(int y = y_min; y <= y_max; ++y)
          {
               y_rel += 1;  ;
              if(y_rel > filter_radius)
                  continue;

              real x_rel = std::abs(x_min + real(0.5) - sample.x);
              for(int x = x_min; x <= x_max; ++x)
              {
                   { x_rel += 1;  ;
                  if(x_rel > filter_radius)
                      continue;
                   int index = x*y;
                   &image[index] +=   texel;

              }
          }
      }
};