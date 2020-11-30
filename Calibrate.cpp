
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/PinholePose.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <boost/make_shared.hpp>
#include <boost/optional/optional_io.hpp>
#include <yaml-cpp/yaml.h>

#include <vector>

#include "HandEyeCalibration.h"
#include "PoseSimulation.h"
#include "ResectioningFactor.h"
#include "YamlUtilities.h"

using namespace gtsam;
using namespace gtsam::noiseModel;
using symbol_shorthand::A;
using symbol_shorthand::B;
using symbol_shorthand::X;

int main(int argc, char *argv[])
{
  auto dataset = readDataset(argv[1]);
  auto objectPoints = std::get<0>(dataset);
  auto imagePointsList = std::get<1>(dataset);
  auto wThList = std::get<2>(dataset);
  auto eToList = std::get<3>(dataset);
  auto cameraCalibration = boost::make_shared<Cal3DS2>(std::get<4>(dataset));
  std::cout << cameraCalibration->k() << std::endl;

  // std::cout << hTe << std::endl;
  // std::cout << wTo << std::endl;
  // for (auto eTo: eToList) {
  //   std::cout << eTo << std::endl;
  // }
  // for (auto wTh: wThList) {
  //   std::cout << wTh << std::endl;
  // }

  // for (auto imagePoints: imagePointsList) {
  //   for (auto imagePoint: imagePoints) {
  //     std::cout << "(" << imagePoint.x() << ", " << imagePoint.y() << ")  ";
  //   }
  //   std::cout << std::endl;
  // }
  
  // Try camera resectioning
  for (int i = 0; i < wThList.size(); i++) {
    const auto imagePoints = imagePointsList[i];
    const auto wTh = wThList[i];
    if (eToList.size() > 0) {
      const auto eTo = eToList[i];
      cout << "Actual: " << eTo << std::endl;
    }

    NonlinearFactorGraph graph;
    auto measurementNoise = nullptr; //Diagonal::Sigmas(Point2(1.0, 1.0));

    for (int j = 0; j < imagePoints.size(); j++) {
      graph.emplace_shared<ResectioningFactor<Cal3DS2>>(
        measurementNoise, X(1), cameraCalibration, imagePoints[j], objectPoints[j]);
    }

    Values initial;
    initial.insert(X(1), Pose3(
      Rot3(
        1.0, 0.0, 0.0,
        0.0, -1.0, 0.0,
        0.0, 0.0, -1.0), 
      Vector3(-0.1, -0.1, 0.1)));

    LevenbergMarquardtParams params; // = LevenbergMarquardtParams::CeresDefaults();
    // params.maxIterations = 30;
    // params.absoluteErrorTol = 0;
    // params.relativeErrorTol = 1e-6;
    // params.useFixedLambdaFactor = false;
    // params.minModelFidelity = 0.1;
    Values result = LevenbergMarquardtOptimizer(graph, initial, params).optimize();
    // Values result = NonlinearConjugateGradientOptimizer(graph, initial).optimize();
    result.print("Result: ");
  }

  // Add pose noise
  // wThList = applyNoise(wThList, 0.01, 0.1);

  /*
    // Solve Hand-Eye using poses
    NonlinearFactorGraph graph;
    auto measurementNoise = nullptr;

    for (int i = 0; i < wThList.size(); i++) {
      auto eTo = eToList[i];
      auto wTh = wThList[i];
      graph.emplace_shared<FixedHandEyePoseFactor>(
        measurementNoise, A(1), B(1), eTo, wTh);
    }

    Values initial;
    initial.insert(A(1), Pose3());
    initial.insert(B(1), Pose3());

    LevenbergMarquardtParams params;
    Values result = LevenbergMarquardtOptimizer(graph, initial, params).optimize();
    result.print("Result: ");
*/

  // Solve Hand-Eye using image points
  NonlinearFactorGraph graph;
  auto measurementNoise = nullptr;

  Values initial;
  for (int i = 0; i < wThList.size(); i++) {
    auto imagePoints = imagePointsList[i];
    auto wTh = wThList[i];

    graph.emplace_shared<HandEyePoseFactor>(
      measurementNoise, A(1), B(1), X(i), wTh);

    for (int j = 0; j < imagePoints.size(); j++) {
      graph.emplace_shared<ResectioningFactor<Cal3DS2>>(
        measurementNoise, X(i), cameraCalibration, imagePoints[j], objectPoints[j]);
    }

    // initial.insert(X(i), eToList[i].inverse());
    initial.insert(X(i), Pose3(
      Rot3(
        1.0, 0.0, 0.0,
        0.0, -1.0, 0.0,
        0.0, 0.0, -1.0),
      Vector3(-0.1, -0.1, 0.1)));
  }

  initial.insert(A(1), Pose3());
  initial.insert(B(1), Pose3());

  LevenbergMarquardtParams params; // = LevenbergMarquardtParams::CeresDefaults();
  Values result = LevenbergMarquardtOptimizer(graph, initial, params).optimize();
  // result.print("Result: ");
  result.at(A(1)).print("hTe");
  result.at(B(1)).print("wTo");
  result.at(B(1)).cast<Pose3>().inverse().print("oTw");

  return 0;
}
