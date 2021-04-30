//
// Created by   on 2019-05-14.
//

#pragma once

#include <vector>
#include "Vector.hpp"
#include "global.hpp"


/**
 * @brief result of sampling we
 */
struct CameraSampleWeResult
{
    CameraSampleWeResult(
        const Vector3f &pos_on_cam,
        const Vector3f &pos_to_out,
        const Vector3f &nor_on_cam,
        const Vector3f &throughput) noexcept
        : pos_on_cam(pos_on_cam),
          pos_to_out(pos_to_out),
          nor_on_cam(nor_on_cam),
          throughput(throughput)
    {

    }

    Vector3f pos_on_cam;
    Vector3f pos_to_out;
    Vector3f nor_on_cam;
    Vector3f throughput;
};

/**
 * @brief pdf of sampling we
 */
struct CameraWePDFResult
{
    CameraWePDFResult(float pdf_pos, float pdf_dir) noexcept
        : pdf_pos(pdf_pos), pdf_dir(pdf_dir)
    {

    }

    float pdf_pos;
    float pdf_dir;
};

/**
 * @brief result of eval we
 */
struct CameraEvalWeResult
{
    CameraEvalWeResult(
        const Vector3f &we,
        const Vector2f &film_coord,
        const Vector3f &nor_on_cam) noexcept
        : we(we), film_coord(film_coord), nor_on_cam(nor_on_cam)
    {

    }

    Vector3f we;
    Vector2f film_coord;
    Vector3f nor_on_cam;
};

inline const CameraEvalWeResult CAMERA_EVAL_WE_RESULT_ZERO =
    CameraEvalWeResult({}, {}, {});

/**
 * @brief result of sampling camera wi
 */
struct CameraSampleWiResult
{
    CameraSampleWiResult(
        const Vector3f &pos_on_cam,
        const Vector3f &nor_at_pos,
        const Vector3f &ref_to_pos,
        const Vector3f &we,
        float pdf,
        const Vector2f &film_coord) noexcept
        : pos_on_cam(pos_on_cam),
          nor_at_pos(nor_at_pos),
          ref_to_pos(ref_to_pos),
          we(we),
          pdf(pdf),
          film_coord(film_coord)
    {

    }

    Vector3f pos_on_cam; // position on camera lens
    Vector3f nor_at_pos; // lens normal
    Vector3f ref_to_pos; // from reference point to position on lens
    Vector3f we;     // initial importance function
    float pdf = 0;     // pdf w.r.t. solid angle at ref
    Vector2f film_coord;  // where on the film does this sample correspond to
};

inline const CameraSampleWiResult CAMERA_SAMPLE_WI_RESULT_INVALID =
    CameraSampleWiResult({}, {}, {}, {}, 0, {});
class Camera
{



public:

    virtual ~Camera() = default;

    /**
     * @brief generate a ray
     *
     * @param film_coord film coordinate. origin is at the left-bottom corner
     *  and the coordinate range is [0, 1]^2
     * @param aperture_sam used to sample the aperture
     */
//    virtual CameraSampleWeResult sample_we(
//        const Vector2f &film_coord ) const noexcept = 0;

    /**
     * @brief eval we(pos_on_cam -> pos_to_out)
     */
    virtual CameraEvalWeResult eval_we(
        const Vector3f &pos_on_cam, const Vector3f &pos_to_out) const noexcept = 0;

    /**
     * @brief pdf of sample_we
     */
//    virtual CameraWePDFResult pdf_we(
//        const Vector3f &pos_on_cam, const Vector3f &pos_to_out) const noexcept = 0;

    /**
     * @brief sample camera wi
     */
    virtual CameraSampleWiResult sample_wi(
        const Vector3f &ref ) const noexcept = 0;

    void apply_image_filter(
              const Vector2f &pixel_range, float filter_radius,
              const Vector2f &sample,std::vector<Vector3f> *image,Vector3f& texel );
};


/*

void look_at(const Vector3f &eye, const Vector3f &dst, const Vector3f &up){
  {
      auto D = (dst - eye).normalized();
      auto R = crossProduct(up, D).normalized();
      auto U = crossProduct(D, R);
      return self_t(R.x, U.x, D.x, eye.x,
                    R.y, U.y, D.y, eye.y,
                    R.z, U.z, D.z, eye.z,
                    0, 0, 0, 1).inverse();
  }
}*/
