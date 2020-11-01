
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
#include "ResectioningFactor.h"

using namespace gtsam;
using namespace gtsam::noiseModel;
using symbol_shorthand::X;
using symbol_shorthand::A;
using symbol_shorthand::B;

int main(int argc, char* argv[])
{
    auto simulatedPose = simulatePoseKoide();
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
    const auto objectPoints = createTargetObject(7, 5, 0.15);

    // Project points
    const auto cameraCalibration = boost::make_shared<Cal3_S2>(Cal3_S2(300.0, 300.0, 0.0, 320.0, 240.0));
    const auto imagePointsList = projectPoints(eToList, objectPoints, cameraCalibration);

    return 0;
}
