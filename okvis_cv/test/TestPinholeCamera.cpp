/**
 * OKVIS2-X - Open Keyframe-based Visual-Inertial SLAM Configurable with Dense 
 * Depth or LiDAR, and GNSS
 *
 * Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 * Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 * Copyright (c) 2025, Mobile Robotics Lab / Technical University of Munich 
 * and ETH Zurich
 *
 * SPDX-License-Identifier: BSD-3-Clause, see LICENESE file for details
 */

/**
 * @file TestPinholeCamera.cpp
 * @brief Runs PinholeCamera tests.
 * @author Stefan Leutenegger
 */

#include <iostream>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "okvis/cameras/PinholeCamera.hpp"
#include "okvis/cameras/NoDistortion.hpp"
#include "okvis/cameras/RadialTangentialDistortion.hpp"
#include "okvis/cameras/RadialTangentialDistortion8.hpp"
#include "okvis/cameras/EquidistantDistortion.hpp"
#include <okvis/assert_macros.hpp>

template<class Distortion>
class PinholeCameraIntrinsics : public ::testing::Test {};

typedef ::testing::Types<okvis::cameras::NoDistortion,
                        okvis::cameras::RadialTangentialDistortion,
                        okvis::cameras::EquidistantDistortion,
                        okvis::cameras::RadialTangentialDistortion8> DistortionTypes;
TYPED_TEST_SUITE(PinholeCameraIntrinsics, DistortionTypes);

TYPED_TEST(PinholeCameraIntrinsics, acceptsValidLength)
{
  typedef okvis::cameras::PinholeCamera<TypeParam> Camera;
  Camera camera = Camera::testObject();
  Eigen::VectorXd intrinsics;
  camera.getIntrinsics(intrinsics);
  intrinsics.head<4>() << 420.0, 430.0, 310.0, 230.0;
  intrinsics.tail(TypeParam::NumDistortionIntrinsics) *= 0.5;

  ASSERT_TRUE(camera.setIntrinsics(intrinsics));
  Eigen::VectorXd actual;
  camera.getIntrinsics(actual);
  EXPECT_TRUE(actual.isApprox(intrinsics));
  EXPECT_DOUBLE_EQ(camera.focalLengthU(), intrinsics[0]);
  EXPECT_DOUBLE_EQ(camera.focalLengthV(), intrinsics[1]);
  EXPECT_DOUBLE_EQ(camera.imageCenterU(), intrinsics[2]);
  EXPECT_DOUBLE_EQ(camera.imageCenterV(), intrinsics[3]);

  TypeParam distortion = TypeParam::testObject();
  ASSERT_TRUE(distortion.setParameters(
      intrinsics.tail(TypeParam::NumDistortionIntrinsics)));
  const Camera expected(camera.imageWidth(), camera.imageHeight(),
                        intrinsics[0], intrinsics[1], intrinsics[2], intrinsics[3],
                        distortion);
  const Eigen::Vector3d point(0.1, -0.08, 1.0);
  Eigen::Vector2d projected, expectedProjection;
  ASSERT_EQ(camera.project(point, &projected),
            okvis::cameras::ProjectionStatus::Successful);
  ASSERT_EQ(expected.project(point, &expectedProjection),
            okvis::cameras::ProjectionStatus::Successful);
  EXPECT_TRUE(projected.isApprox(expectedProjection));

  Eigen::Vector3d ray, expectedRay;
  ASSERT_TRUE(camera.backProject(expectedProjection, &ray));
  ASSERT_TRUE(expected.backProject(expectedProjection, &expectedRay));
  EXPECT_TRUE(ray.isApprox(expectedRay));
}

TYPED_TEST(PinholeCameraIntrinsics, rejectsInvalidLengthsWithoutUpdating)
{
  typedef okvis::cameras::PinholeCamera<TypeParam> Camera;
  Camera camera = Camera::testObject();
  Eigen::VectorXd before;
  camera.getIntrinsics(before);
  const Eigen::Vector3d point(0.1, -0.08, 1.0);
  Eigen::Vector2d projectionBefore;
  ASSERT_EQ(camera.project(point, &projectionBefore),
            okvis::cameras::ProjectionStatus::Successful);
  Eigen::Vector3d rayBefore;
  ASSERT_TRUE(camera.backProject(projectionBefore, &rayBefore));

  for (const int length : {0, 1, Camera::NumIntrinsics - 1, Camera::NumIntrinsics + 1}) {
    SCOPED_TRACE(length);
    EXPECT_FALSE(camera.setIntrinsics(Eigen::VectorXd::Constant(length, 123.0)));
    Eigen::VectorXd after;
    camera.getIntrinsics(after);
    EXPECT_TRUE(after.isApprox(before));
    Eigen::Vector2d projectionAfter;
    ASSERT_EQ(camera.project(point, &projectionAfter),
              okvis::cameras::ProjectionStatus::Successful);
    EXPECT_TRUE(projectionAfter.isApprox(projectionBefore));
    Eigen::Vector3d rayAfter;
    ASSERT_TRUE(camera.backProject(projectionBefore, &rayAfter));
    EXPECT_TRUE(rayAfter.isApprox(rayBefore));
  }
}

TEST(PinholeCamera, project)
{
  const size_t NUM_POINTS = 100;

  // instantiate all possible versions of test cameras
  std::vector<std::shared_ptr<okvis::cameras::CameraBase> > cameras;
  cameras.push_back(
      okvis::cameras::PinholeCamera<okvis::cameras::NoDistortion>::createTestObject());
  cameras.push_back(
      okvis::cameras::PinholeCamera<
          okvis::cameras::RadialTangentialDistortion>::createTestObject());
  cameras.push_back(
      okvis::cameras::PinholeCamera<okvis::cameras::EquidistantDistortion>::createTestObject());
  cameras.push_back(
      okvis::cameras::PinholeCamera<okvis::cameras::RadialTangentialDistortion8>
        ::createTestObject());

  for (size_t c = 0; c < cameras.size(); ++c) {
    //std::cout << "Testing " << cameras.at(c)->type() << std::endl;
    // try quite a lot of points:
    for (size_t i = 0; i < NUM_POINTS; ++i) {
      // create a random point in the field of view:
      Eigen::Vector2d imagePoint = cameras.at(c)->createRandomImagePoint();

      // backProject
      Eigen::Vector3d ray;
      OKVIS_ASSERT_TRUE(std::runtime_error, cameras.at(c)->backProject(imagePoint, &ray),
                        "unsuccessful back projection")

      // randomise distance
      ray.normalize();
      ray *= (0.2 + 8 * (Eigen::Vector2d::Random()[0] + 1.0));

      // project
      Eigen::Vector2d imagePoint2;
      Eigen::Matrix<double, 2, 3> J;
      Eigen::Matrix2Xd J_intrinsics;
      OKVIS_ASSERT_TRUE(std::runtime_error,
          cameras.at(c)->project(ray, &imagePoint2, &J, &J_intrinsics)
              == okvis::cameras::ProjectionStatus::Successful,
                        "unsuccessful projection")

      // check they are the same
      OKVIS_ASSERT_TRUE(std::runtime_error, (imagePoint2 - imagePoint).norm() < 0.01,
                        "project/unproject failure")

      // check point Jacobian vs. NumDiff
      const double dp = 1.0e-7;
      Eigen::Matrix<double, 2, 3> J_numDiff;
      for (size_t d = 0; d < 3; ++d) {
        Eigen::Vector3d point_p = ray
            + Eigen::Vector3d(d == 0 ? dp : 0, d == 1 ? dp : 0,
                              d == 2 ? dp : 0);
        Eigen::Vector3d point_m = ray
            - Eigen::Vector3d(d == 0 ? dp : 0, d == 1 ? dp : 0,
                              d == 2 ? dp : 0);
        Eigen::Vector2d imagePoint_p;
        Eigen::Vector2d imagePoint_m;
        cameras.at(c)->project(point_p, &imagePoint_p);
        cameras.at(c)->project(point_m, &imagePoint_m);
        J_numDiff.col(int(d)) = (imagePoint_p - imagePoint_m) / (2 * dp);
      }
      OKVIS_ASSERT_TRUE(std::runtime_error, (J_numDiff - J).norm() < 0.0001,
                        "Jacobian Verification failed")

      // check intrinsics Jacobian
      const int numIntrinsics = cameras.at(c)->noIntrinsicsParameters();
      Eigen::VectorXd intrinsics;
      cameras.at(c)->getIntrinsics(intrinsics);
      Eigen::Matrix2Xd J_numDiff_intrinsics;
      J_numDiff_intrinsics.resize(2,numIntrinsics);
      for (int d = 0; d < numIntrinsics; ++d) {
        Eigen::VectorXd di;
        di.resize(numIntrinsics);
        di.setZero();
        di[d] = dp;
        Eigen::Vector2d imagePoint_p;
        Eigen::Vector2d imagePoint_m;
        Eigen::VectorXd intrinsics_p = intrinsics+di;
        Eigen::VectorXd intrinsics_m = intrinsics-di;
        cameras.at(c)->projectWithExternalParameters(ray, intrinsics_p, &imagePoint_p);
        cameras.at(c)->projectWithExternalParameters(ray, intrinsics_m, &imagePoint_m);
        J_numDiff_intrinsics.col(d) = (imagePoint_p - imagePoint_m) / (2 * dp);
      }
      OKVIS_ASSERT_TRUE(std::runtime_error, (J_numDiff_intrinsics - J_intrinsics).norm() < 0.0001,
          "Jacobian verification failed")

    }
  }
}

TEST(PinholeCamera, projectWithExternalParameters)
{
  const size_t NUM_POINTS = 100;

  // instantiate all possible versions of test cameras
  std::vector<std::shared_ptr<okvis::cameras::CameraBase> > cameras;
  cameras.push_back(
      okvis::cameras::PinholeCamera<okvis::cameras::NoDistortion>::createTestObject());
  cameras.push_back(
      okvis::cameras::PinholeCamera<
          okvis::cameras::RadialTangentialDistortion>::createTestObject());
  cameras.push_back(
      okvis::cameras::PinholeCamera<okvis::cameras::EquidistantDistortion>::createTestObject());
  cameras.push_back(
      okvis::cameras::PinholeCamera<okvis::cameras::RadialTangentialDistortion8>
        ::createTestObject());

  for (size_t c = 0; c < cameras.size(); ++c) {
    std::cout << "Testing " << cameras.at(c)->type() << std::endl;
    // try quite a lot of points:
    for (size_t i = 0; i < NUM_POINTS; ++i) {
      // create a random point in the field of view:
      Eigen::Vector2d imagePoint = cameras.at(c)->createRandomImagePoint();

      // backProject
      Eigen::Vector3d ray;
      OKVIS_ASSERT_TRUE(std::runtime_error, cameras.at(c)->backProject(imagePoint, &ray),
                        "unsuccessful back projection")

      // randomise distance
      ray.normalize();
      ray *= (0.2 + 8 * (Eigen::Vector2d::Random()[0] + 1.0));

      // get camera params
      Eigen::VectorXd cam_params;
      cameras.at(c)->getIntrinsics(cam_params);

      // project
      Eigen::Vector2d imagePoint2;
      Eigen::Matrix<double, 2, 3> J;
      Eigen::Matrix2Xd J_intrinsics;
      OKVIS_ASSERT_TRUE(std::runtime_error,
          cameras.at(c)->projectWithExternalParameters(
                          ray, cam_params, &imagePoint2, &J, &J_intrinsics)
              == okvis::cameras::ProjectionStatus::Successful,
                        "unsuccessful projection")
      // ^ NOTE: Instead of calling project() we are explicitly calling
      // projectWithExternalParameters()
      // The point jacobian (`J` in this instance) returned is incorrect, therefore the following
      // test will fail.

      // check they are the same
      OKVIS_ASSERT_TRUE(std::runtime_error, (imagePoint2 - imagePoint).norm() < 0.01,
                        "project/unproject failure")

      // check point Jacobian vs. NumDiff
      const double dp = 1.0e-7;
      Eigen::Matrix<double, 2, 3> J_numDiff;
      for (size_t d = 0; d < 3; ++d) {
        Eigen::Vector3d point_p = ray
            + Eigen::Vector3d(d == 0 ? dp : 0, d == 1 ? dp : 0,
                              d == 2 ? dp : 0);
        Eigen::Vector3d point_m = ray
            - Eigen::Vector3d(d == 0 ? dp : 0, d == 1 ? dp : 0,
                              d == 2 ? dp : 0);
        Eigen::Vector2d imagePoint_p;
        Eigen::Vector2d imagePoint_m;
        cameras.at(c)->projectWithExternalParameters(point_p, cam_params, &imagePoint_p);
        cameras.at(c)->projectWithExternalParameters(point_m, cam_params, &imagePoint_m);
        J_numDiff.col(int(d)) = (imagePoint_p - imagePoint_m) / (2 * dp);
      }
      OKVIS_ASSERT_TRUE(std::runtime_error, J_numDiff.isApprox(J, 0.0001),
                        "Jacobian Verification failed")
      for (size_t d = 0; d < 3; ++d) {
        Eigen::Vector3d point_p = ray
            + Eigen::Vector3d(d == 0 ? dp : 0, d == 1 ? dp : 0,
                              d == 2 ? dp : 0);
        Eigen::Vector3d point_m = ray
            - Eigen::Vector3d(d == 0 ? dp : 0, d == 1 ? dp : 0,
                              d == 2 ? dp : 0);
        Eigen::Vector2d imagePoint_p;
        Eigen::Vector2d imagePoint_m;
        cameras.at(c)->project(point_p, &imagePoint_p);
        cameras.at(c)->project(point_m, &imagePoint_m);
        J_numDiff.col(int(d)) = (imagePoint_p - imagePoint_m) / (2 * dp);
      }
      OKVIS_ASSERT_TRUE(std::runtime_error, J_numDiff.isApprox(J, 0.0001),
                        "Jacobian Verification failed")

      // check intrinsics Jacobian
      const int numIntrinsics = cameras.at(c)->noIntrinsicsParameters();
      Eigen::VectorXd intrinsics;
      cameras.at(c)->getIntrinsics(intrinsics);
      Eigen::Matrix2Xd J_numDiff_intrinsics;
      J_numDiff_intrinsics.resize(2,numIntrinsics);
      for (int d = 0; d < numIntrinsics; ++d) {
        Eigen::VectorXd di;
        di.resize(numIntrinsics);
        di.setZero();
        di[d] = dp;
        Eigen::Vector2d imagePoint_p;
        Eigen::Vector2d imagePoint_m;
        Eigen::VectorXd intrinsics_p = intrinsics+di;
        Eigen::VectorXd intrinsics_m = intrinsics-di;
        cameras.at(c)->projectWithExternalParameters(ray, intrinsics_p, &imagePoint_p);
        cameras.at(c)->projectWithExternalParameters(ray, intrinsics_m, &imagePoint_m);
        J_numDiff_intrinsics.col(d) = (imagePoint_p - imagePoint_m) / (2 * dp);
      }
      OKVIS_ASSERT_TRUE(std::runtime_error, J_numDiff_intrinsics.isApprox(J_intrinsics, 0.0001),
          "Jacobian verification failed")

    }
  }
}
