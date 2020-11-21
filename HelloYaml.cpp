
#include <yaml-cpp/yaml.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>

#include <iostream>
#include <vector>

#include "YamlUtilities.h"

using namespace gtsam;

int main(int argc, char* argv[])
{
  YAML::Emitter out;

  out << YAML::BeginMap;

  out << YAML::Key << "yeah";
  out << YAML::Value << YAML::Flow << YAML::BeginSeq;
  out << "eggs";
  out << "bread";
  out << "milk";
  out << YAML::EndSeq;

  out << YAML::Key << "oh";
  out << YAML::Value << 23.45;

  out << YAML::EndMap;

  std::cout << out.c_str() << std::endl;
  std::cout << std::endl;

  YAML::Node root = YAML::LoadFile("yeah.yaml");
  
  std::vector<Pose3> wThList;
  std::vector<std::vector<Vector2>> imagePointsList;
  for (auto view: root["views"]) {
    wThList.push_back(view["wTh"].as<Pose3>());
    imagePointsList.push_back(view["image_points"].as<std::vector<Vector2>>());
  }
  auto objectPoints = root["object_points"].as<std::vector<Vector3>>();
  auto cameraCalibration = root["camera_calibration"].as<Cal3_S2>();

  for (auto imagePoints: imagePointsList) {
    for (auto p: imagePoints) {
      std::cout << p << std::endl << std::endl;
    }
    break;
  }

  for (auto wTh: wThList) {
    std::cout << wTh << std::endl << std::endl;
    break;
  }

  return 0;
}
