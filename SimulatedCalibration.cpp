
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/PinholePose.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <boost/make_shared.hpp>
#include <boost/optional/optional_io.hpp>

#include <vector>

#include "HandEyeCalibration.h"
#include "PoseSimulation.h"
#include "ResectioningFactor.h"

using namespace gtsam;
using namespace gtsam::noiseModel;
using symbol_shorthand::A;
using symbol_shorthand::B;
using symbol_shorthand::L;
using symbol_shorthand::X;

int main(int argc, char *argv[])
{
  auto simulatedPose = simulatePoseKoide(Pose3(
    Rot3(),
    Vector3(0.1, -0.1, 0.05)));
  auto wThList = std::get<0>(simulatedPose);
  auto eToList = std::get<1>(simulatedPose);
  auto hTe = std::get<2>(simulatedPose);
  auto wTo = std::get<3>(simulatedPose);

  std::cout << hTe << std::endl;
  std::cout << wTo << std::endl;
  // for (auto eTo: eToList) {
  //   std::cout << eTo << std::endl;
  // }
  // for (auto wTh: wThList) {
  //   std::cout << wTh << std::endl;
  // }

  // Project points
  const auto objectPoints = createTargetObject(3, 3, 0.25);
  const auto cameraCalibration = boost::make_shared<Cal3_S2>(Cal3_S2(300.0, 300.0, 0.0, 320.0, 240.0));
  const auto imagePointsList = projectPoints(eToList, objectPoints, cameraCalibration);

  // for (auto imagePoints: imagePointsList) {
  //   for (auto imagePoint: imagePoints) {
  //     std::cout << "(" << imagePoint.x() << ", " << imagePoint.y() << ")  ";
  //   }
  //   std::cout << std::endl;
  // }

  cout << "Actual:" << endl;
  for (int i = 0; i < wThList.size(); i++) {
    const auto imagePoints = imagePointsList[i];
    const auto wTh = wThList[i];
    const auto oTe = eToList[i].inverse();
    cout << "X[" << i << "]: " << oTe << endl;
  }

  auto measurementNoise = nullptr; //Diagonal::Sigmas(Point2(1.0, 1.0));
  NonlinearFactorGraph graph;
  Values initial;
  graph.addPrior(X(0), eToList[0].inverse(), measurementNoise);
  graph.addPrior(L(0), objectPoints[0], measurementNoise);
  
  cout << "X(0): " << eToList[0].inverse() << endl;
  cout << "L(0): " << objectPoints[0] << endl;

  for (int i = 0; i < wThList.size(); i++) {
    const auto imagePoints = imagePointsList[i];
    const auto wTh = wThList[i];
    const auto oTe = eToList[i].inverse();
    // cout << "Actual: " << eTo << std::endl;

    auto oTe_corrupted = oTe.compose(Pose3(
      Rot3::Rodrigues((-1 * (i % 2)) * 0.3, 0.2, 0.25), Point3(0.5, -0.40, 0.60)));
    initial.insert(X(i), oTe_corrupted);

    for (int j = 0; j < imagePoints.size(); j++) {
      // graph.emplace_shared<ResectioningFactor>(
      //   measurementNoise, X(1), cameraCalibration, imagePoints[j], objectPoints[j]);
      graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2>>(
        imagePoints[j], measurementNoise, X(i), L(j), cameraCalibration);
    }
  }

  for (int j = 0; j < objectPoints.size(); j++) {
    auto corruptedObjectPoints = objectPoints[j] + Vector3((-1 * (j % 2)) * 0.25, 0.20, 0.15);
    initial.insert<Vector3>(L(j), corruptedObjectPoints);
  }

  initial.print("Initial estimate:\n");

  LevenbergMarquardtParams params; // = LevenbergMarquardtParams::CeresDefaults();
  // params.maxIterations = 30;
  // params.absoluteErrorTol = 0;
  // params.relativeErrorTol = 1e-6;
  // params.useFixedLambdaFactor = false;
  // params.minModelFidelity = 0.1;
  Values result = LevenbergMarquardtOptimizer(graph, initial, params).optimize();
  // Values result = NonlinearConjugateGradientOptimizer(graph, initial).optimize();
  result.print("Result: ");

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
 
  /*
  // Solve Hand-Eye using image points
  NonlinearFactorGraph graph;
  auto measurementNoise = nullptr;

  Values initial;
  auto poseIndex = 1;
  for (int i = 0; i < wThList.size(); i++) {
    auto imagePoints = imagePointsList[i];
    auto wTh = wThList[i];

    graph.emplace_shared<HandEyePoseFactor>(
      measurementNoise, A(1), B(1), X(poseIndex), wTh);

    for (int j = 0; j < imagePoints.size(); j++) {
      graph.emplace_shared<ResectioningFactor<Cal3_S2>>(
        measurementNoise, X(poseIndex), cameraCalibration, imagePoints[j], objectPoints[j]);
    }

    initial.insert(X(poseIndex), Pose3(
      Rot3(
        -1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, -1.0),
      Vector3(-0.11, -0.11, 0.1)).inverse());

    poseIndex++;
  }

  initial.insert(A(1), Pose3());
  initial.insert(B(1), Pose3());

  LevenbergMarquardtParams params; // = LevenbergMarquardtParams::CeresDefaults();
  Values result = LevenbergMarquardtOptimizer(graph, initial, params).optimize();
  // result.print("Result: ");
  result.at(A(1)).print("hTe");
  result.at(B(1)).print("wTo");
  */
  return 0;
}
