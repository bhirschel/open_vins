/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2022 Patrick Geneva
 * Copyright (C) 2018-2022 Guoquan Huang
 * Copyright (C) 2018-2022 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef OV_CORE_CAM_BASE_OMNI_H
#define OV_CORE_CAM_BASE_OMNI_H

#include "CamBase.h"
#include <Eigen/Eigen>
#include <unordered_map>

#include <opencv2/opencv.hpp>
#include <opencv2/ccalib/omnidir.hpp>

namespace ov_core {

/**
 * @brief Base omnidirectional camera model class
 *
 * This is the base class for all our camera models.
 * All these models are omnidirectional cameras, thus just have standard reprojection logic.
 *
 * See each base class for detailed examples on each model:
 *  - @ref ov_core::CamOmniRadtan
 */
class CamBaseOmni : public CamBase {

public:
  /**
   * @brief Default constructor
   * @param width Width of the camera (raw pixels)
   * @param height Height of the camera (raw pixels)
   */
  CamBaseOmni(int width, int height) : CamBase(width, height) {}

  /**
   * @brief This will set and update the camera calibration values.
   * This should be called on startup for each camera and after update!
   * @param calib Camera calibration information (xi & f_u & f_v & p_u & p_v & k_1 & k_2 & r_1 & r_2)
   */
  virtual void set_value(const Eigen::MatrixXd &calib) {

    // Assert we are of size eight
    assert(calib.rows() == 9);
    camera_values = calib;

    // Camera matrix
    cv::Matx33d tempK;
    tempK(0, 0) = calib(1);
    tempK(0, 1) = 0; // skew coefficient s -> often 0?
    tempK(0, 2) = calib(3);
    tempK(1, 0) = 0;
    tempK(1, 1) = calib(2);
    tempK(1, 2) = calib(4);
    tempK(2, 0) = 0;
    tempK(2, 1) = 0;
    tempK(2, 2) = 1;
    camera_k_OPENCV = tempK;

    // Distortion parameters
    cv::Vec4d tempD;
    tempD(0) = calib(5);
    tempD(1) = calib(6);
    tempD(2) = calib(7);
    tempD(3) = calib(8);
    camera_d_OPENCV = tempD;

    // Xi parameter for CMei model
    camera_xi_OPENCV = calib(0);
  }

protected:
  // Cannot construct the base camera class, needs a distortion model
  CamBaseOmni() = default;

  /// parameter xi for CMei's model
  double camera_xi_OPENCV;
};

} // namespace ov_core

#endif /* OV_CORE_CAM_BASE_OMNI_H */