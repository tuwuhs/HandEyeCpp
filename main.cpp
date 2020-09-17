/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    CameraResectioning.cpp
 * @brief   An example of gtsam for solving the camera resectioning problem
 * @author  Duy-Nguyen Ta
 * @date    Aug 23, 2011
 */

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <boost/make_shared.hpp>

#include <vector>

#include "PoseSimulation.h"

using namespace gtsam;
using namespace gtsam::noiseModel;
using symbol_shorthand::X;

/**
 * Unary factor on the unknown pose, resulting from meauring the projection of
 * a known 3D point in the image
 */
class ResectioningFactor : public NoiseModelFactor1<Pose3>
{
    typedef NoiseModelFactor1<Pose3> Base;

    Cal3_S2::shared_ptr K_; ///< camera's intrinsic parameters
    Point3 P_;              ///< 3D point on the calibration rig
    Point2 p_;              ///< 2D measurement of the 3D point

public:
    /// Construct factor given known point P and its projection p
    ResectioningFactor(const SharedNoiseModel &model, const Key &key,
                       const Cal3_S2::shared_ptr &calib, const Point2 &p, const Point3 &P) : Base(model, key), K_(calib), P_(P), p_(p)
    {
    }

    /// evaluate the error
    Vector evaluateError(const Pose3 &pose, boost::optional<Matrix &> H =
                                                boost::none) const override
    {
        PinholeCamera<Cal3_S2> camera(pose, *K_);
        const auto error = camera.project(P_, H, boost::none, boost::none) - p_;
        // std::cout << error << std::endl;
        return error;
    }
};

/*******************************************************************************
 * Camera: f = 1, Image: 100x100, center: 50, 50.0
 * Pose (ground truth): (Xw, -Yw, -Zw, [0,0,2.0]')
 * Known landmarks:
 *    3D Points: (10,10,0) (-10,10,0) (-10,-10,0) (10,-10,0)
 * Perfect measurements:
 *    2D Point:  (55,45)   (45,45)    (45,55)     (55,55)
 *******************************************************************************/
int main(int argc, char *argv[])
{
    // /* read camera intrinsic parameters */
    // Cal3_S2::shared_ptr calib(new Cal3_S2(1, 1, 0, 50, 50));

    // /* 1. create graph */
    // NonlinearFactorGraph graph;

    // /* 2. add factors to the graph */
    // // add measurement factors
    // SharedDiagonal measurementNoise = Diagonal::Sigmas(Vector2(0.5, 0.5));
    // boost::shared_ptr<ResectioningFactor> factor;
    // graph.emplace_shared<ResectioningFactor>(measurementNoise, X(1), calib,
    //                                          Point2(55, 45), Point3(10, 10, 0));
    // graph.emplace_shared<ResectioningFactor>(measurementNoise, X(1), calib,
    //                                          Point2(45, 45), Point3(-10, 10, 0));
    // graph.emplace_shared<ResectioningFactor>(measurementNoise, X(1), calib,
    //                                          Point2(45, 55), Point3(-10, -10, 0));
    // graph.emplace_shared<ResectioningFactor>(measurementNoise, X(1), calib,
    //                                          Point2(55, 55), Point3(10, -10, 0));

    // /* 3. Create an initial estimate for the camera pose */
    // Values initial;
    // initial.insert(X(1),
    //                Pose3(Rot3(1, 0, 0, 0, -1, 0, 0, 0, -1), Point3(0, 0, 0.9)));

    // /* 4. Optimize the graph using Levenberg-Marquardt*/
    // Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
    // result.print("Final result:\n");

    auto simulatedPose = simulatePoseKoide();
    auto wThList = std::get<0>(simulatedPose);
    auto eToList = std::get<1>(simulatedPose);
    auto hTe = std::get<2>(simulatedPose);
    auto wTo = std::get<3>(simulatedPose);

    // for (auto wTh: wThList) {
    //     std::cout << wTh << std::endl;
    // }
    // for (auto eTo: eToList) {
    //     std::cout << eTo << std::endl;
    // }
    // std::cout << wTo << std::endl;
    // std::cout << hTe << std::endl;

    // Create target object
    const int rows = 7;
    const int cols = 5;
    const double dimension = 0.1;
    std::vector<Point3> objectPoints;
    for (int row = 0; row < rows; row++) {
        for (int col = 0; col < cols; col++) {
            objectPoints.push_back(dimension * Point3(
                0.5 * (row - rows + 1),
                0.5 * (col - cols + 1),
                0.0
            ));
        }
    }

    // Project points
    const auto cameraCalibration = boost::make_shared<Cal3_S2>(Cal3_S2(300.0, 300.0, 0.0, 320.0, 240.0));
    std::vector<std::vector<Point2>> imagePointsList;
    for (auto wTh: wThList) {
        const auto oTe = wTo.inverse() * wTh * hTe;
        const auto cam = PinholeCamera<Cal3_S2>(oTe, *cameraCalibration);
        std::vector<Point2> imagePoints;
        for (auto objectPoint: objectPoints) {
            imagePoints.push_back(cam.project(objectPoint));
        }
        imagePointsList.push_back(imagePoints);
    }

    // for (auto imagePoints: imagePointsList) {
    //     for (auto imagePoint: imagePoints) {
    //         std::cout << "(" << imagePoint.x() << ", " << imagePoint.y() << ")  ";
    //     }
    //     std::cout << std::endl;
    // }

    // Try camera resectioning
    for (int i = 0; i < wThList.size(); i++) {
        const auto imagePoints = imagePointsList[i];
        const auto wTh = wThList[i];
        const auto oTe = wTo.inverse() * wTh * hTe;
        cout << "Actual: " << oTe << std::endl;

        NonlinearFactorGraph graph;
        auto measurementNoise = Diagonal::Sigmas(Point2(1.0, 1.0));

        for (int j = 0; j < imagePoints.size(); j++) {
            graph.emplace_shared<ResectioningFactor>(
                measurementNoise, X(1), cameraCalibration, imagePoints[j], objectPoints[j]);
        }

        Values initial;
        initial.insert(X(1), Pose3(
            Rot3(
                -1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, -1.0), 
            Point3(-0.1, -0.1, 0.1)
        ));
        
        Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
        result.print("Result: ");

        // break;
    }

    return 0;
}
