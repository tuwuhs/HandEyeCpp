
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

std::tuple<
  std::vector<Vector3>, 
  std::vector<std::vector<Vector2>>, 
  std::vector<Pose3>, 
  std::vector<Pose3>, 
  Cal3DS2> readDataset(std::string filename)
{
  YAML::Node root = YAML::LoadFile(filename);
  
  std::vector<Pose3> wThList;
  std::vector<Pose3> eToList;
  std::vector<std::vector<Vector2>> imagePointsList;
  for (auto view: root["views"]) {
    wThList.push_back(view["wTh"].as<Pose3>());
    if (view["eTo"])
      eToList.push_back(view["eTo"].as<Pose3>());
    imagePointsList.push_back(view["image_points"].as<std::vector<Vector2>>());
  }
  auto objectPoints = root["object_points"].as<std::vector<Vector3>>();
  auto cameraCalibration = root["camera_calibration"].as<Cal3DS2>();

  return std::tie(objectPoints, imagePointsList, wThList, eToList, cameraCalibration);
}

