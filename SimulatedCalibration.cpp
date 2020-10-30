
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/PinholePose.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <boost/make_shared.hpp>
#include <boost/optional/optional_io.hpp>

#include <vector>

#include "PoseSimulation.h"

using namespace gtsam;
using namespace gtsam::noiseModel;
using symbol_shorthand::X;

class FixedHandEyePoseFactor : public NoiseModelFactor2<Pose3, Pose3>
{
    typedef NoiseModelFactor2<Pose3, Pose3> Base;

    Pose3 eTo_;
    Pose3 wTh_;

public:
    FixedHandEyePoseFactor(const SharedNoiseModel &model, const Key &hTe, const Key &wTo,
                           const Pose3 eTo, const Pose3 wTh)
    : Base(model, hTe, wTo), eTo_(eTo), wTh_(wTh) 
    {
    }

    Vector evaluateError(const Pose3 &hTe, const Pose3 &wTo,
                         boost::optional<Matrix&> HhTe = boost::none,
                         boost::optional<Matrix&> HwTo = boost::none) const override
    {
        // Inspired from BetweenFactor
        Matrix6 H1;
        Matrix6 H2;
        auto eTo = hTe.between(wTh_.inverse() * wTo, H1, H2);

        Matrix6 Hlocal;
        auto error = eTo_.localCoordinates(eTo, boost::none, Hlocal);

        if (HhTe)
            *HhTe = Hlocal * H1;

        if (HwTo)
            *HwTo = Hlocal * H2;
        
        // std::cout << error << std::endl << std::flush;
        return error;
    }
};

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
                       const Cal3_S2::shared_ptr &calib, const Point2 &p, const Point3 &P) 
                       : Base(model, key), K_(calib), P_(P), p_(p)
    {
    }

    /// evaluate the error
    Vector evaluateError(const Pose3 &pose, 
                         boost::optional<Matrix &> H = boost::none) const override
    {
        Matrix6 Dinverse;
        PinholePose<Cal3_S2> camera(pose.inverse(H ? &Dinverse : 0), K_);

        Matrix26 Dproject;
        const auto error = camera.project(P_, H ? &Dproject : 0, boost::none, boost::none) - p_;

        // Chain the Jacobians
        if (H) {
            *H = Dproject * Dinverse;
        }
        
        // std::cout << error << std::endl << std::flush;
        return error;
    }
};

int main(int argc, char *argv[])
{
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
    std::cout << hTe << std::endl;
    std::cout << wTo << std::endl;

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
        const auto cam = PinholePose<Cal3_S2>(oTe, cameraCalibration);
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

/*
    // Try camera resectioning
    for (int i = 0; i < wThList.size(); i++) {
        const auto imagePoints = imagePointsList[i];
        const auto wTh = wThList[i];
        const auto oTe = wTo.inverse() * wTh * hTe;
        // cout << "Actual: " << oTe << std::endl;
        cout << "Actual: " << oTe.inverse() << std::endl;

        NonlinearFactorGraph graph;
        auto measurementNoise = nullptr; //Diagonal::Sigmas(Point2(1.0, 1.0));

        for (int j = 0; j < imagePoints.size(); j++) {
            graph.emplace_shared<ResectioningFactor>(
                measurementNoise, X(1), cameraCalibration, imagePoints[j], objectPoints[j]);
        }

        Values initial;
        // initial.insert(X(1), Pose3(
        //     Rot3(
        //         -1.0, 0.0, 0.0,
        //         0.0, 1.0, 0.0,
        //         0.0, 0.0, -1.0), 
        //     Vector3(-0.1, -0.1, 0.1)
        // ));
        initial.insert(X(1), Pose3(
            Rot3(
                -1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, -1.0), 
            Vector3(-0.1, -0.1, 0.1)
        ).inverse());
        // initial.insert(X(1), Pose3(
        //     Rot3(
        //         -0.788675, -0.211325, 0.57735,
        //         0.211325, 0.788675, 0.57735,
        //         -0.57735, 0.57735, -0.57735),
        //     Vector3(-0, -0.5, 0.5)
        // ));
        // initial.insert(X(1), Pose3(
        //     Rot3(
        //         -0.788675, 0.211325, -0.57735,
        //         -0.211325, 0.788675, 0.57735,
        //         0.57735, 0.57735, -0.57735),
        //     Vector3(0, 0, 0.866025)
        // ));

        LevenbergMarquardtParams params; // = LevenbergMarquardtParams::CeresDefaults();
        // params.maxIterations = 30;
        // params.absoluteErrorTol = 0;
        // params.relativeErrorTol = 1e-6;
        // params.useFixedLambdaFactor = false;
        // params.minModelFidelity = 0.1;
        Values result = LevenbergMarquardtOptimizer(graph, initial, params).optimize();
        // Values result = NonlinearConjugateGradientOptimizer(graph, initial).optimize();
        result.print("Result: ");

        break;
    }
*/

    // Add pose noise
    wThList = applyNoise(wThList, 0.05, 0.5);

    // Solve Hand-Eye using poses
    NonlinearFactorGraph graph;
    auto measurementNoise = nullptr;

    for (int i = 0; i < wThList.size(); i++) {
        auto eTo = eToList[i];
        auto wTh = wThList[i];
        graph.emplace_shared<FixedHandEyePoseFactor>(
            measurementNoise, X(1), X(2), eTo, wTh);
    }

    Values initial;
    initial.insert(X(1), Pose3());
    initial.insert(X(2), Pose3());

    LevenbergMarquardtParams params;
    Values result = LevenbergMarquardtOptimizer(graph, initial, params).optimize();
    result.print("Result: ");

    return 0;
}
