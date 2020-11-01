
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

#include "HandEyeCalibration.h"
#include "PoseSimulation.h"
#include "ResectioningFactor.h"

using namespace gtsam;
using namespace gtsam::noiseModel;
using symbol_shorthand::X;
using symbol_shorthand::A;
using symbol_shorthand::B;

int main(int argc, char* argv[])
{
    auto simulatedPose = simulatePoseKoide(Pose3(
        Rot3(
            1, 0, 0,
            0, -1, 0,
            0, 0, -1), 
        Vector3(0.1, 0.1, 0.1)).inverse());
    
    auto wThList = std::get<0>(simulatedPose);
    auto eToList = std::get<1>(simulatedPose);
    auto hTe = std::get<2>(simulatedPose);
    auto wTo = std::get<3>(simulatedPose);

    std::cout << hTe << std::endl;
    std::cout << wTo << std::endl;
    // for (auto eTo: eToList) {
    //     std::cout << eTo << std::endl;
    // }
    // for (auto wTh: wThList) {
    //     std::cout << wTh << std::endl;
    // }

    // Create target object
    const auto objectPoints = createTargetObject(3, 2, 0.15);

    // Project points
    const auto cameraCalibration = boost::make_shared<Cal3_S2>(Cal3_S2(300.0, 300.0, 0.0, 320.0, 240.0));
    const auto imagePointsList = projectPoints(eToList, objectPoints, cameraCalibration);

    auto measurementNoise = nullptr;

    for (int i = 0; i < imagePointsList.size(); i++) {
        auto imagePoints = imagePointsList[i];
        auto eTo = eToList[i];

        for (int j = 0; j < imagePoints.size(); j++) {
            auto crf = ResectioningFactor(measurementNoise, X(1), cameraCalibration, imagePoints[j], objectPoints[j]);

            // std::cout << imagePoints[j] << std::endl;
            // std::cout << objectPoints[j] << std::endl;

            auto currPose = Pose3(
                Rot3(),
                Vector3(0.2, 0.4, 0.0)) * eTo;
            
            Matrix jac;
            auto error = crf.evaluateError(currPose, jac);
            
            std::cout << error << std::endl;
            std::cout << jac << std::endl;

            std::cout << std::endl;
        }

        break;
    }

    std::cout << std::endl;

    for (int i = 0; i < eToList.size(); i++) {
        auto eTo = eToList[i];
        auto wTh = wThList[i];

        auto hepf = HandEyePoseFactor(measurementNoise, X(1), X(2), X(3), wTh);
        // auto fhepf = FixedHandEyePoseFactor(measurementNoise, X(1), X(2), eTo, wTh);

        auto hTe_curr = Pose3( //) * hTe;
            Rot3(
                -1, 0, 0,
                0, 1, 0,
                0, 0, -1),
            Vector3(0.1, 0.1, 0.1)) * hTe;
        
        auto eTo_curr = Pose3( //) * eTo;
            Rot3(),
            Vector3(0.2, 0.4, 0.0)) * eTo;

        auto wTo_curr = Pose3( //) * wTo;
            Rot3(),
            Vector3(0.2, 0.4, 0.0)) * wTo;

        Matrix HhTe;
        Matrix HwTo;
        Matrix HeTo;
        auto error = hepf.evaluateError(hTe_curr, wTo_curr, eTo_curr, HhTe, HwTo, HeTo);
        // auto error = fhepf.evaluateError(hTe_curr, wTo_curr, HhTe, HwTo);

        std::cout << error << std::endl;
        std::cout << HhTe << std::endl;
        std::cout << HwTo << std::endl;
        std::cout << HeTo << std::endl;
        
        std::cout << std::endl;

        break;
    }

    return 0;
}
