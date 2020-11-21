
#include "PoseSimulation.h"

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/PinholePose.h>
#include <gtsam/geometry/Pose3.h>

#include <random>
#include <vector>

using namespace gtsam;

std::tuple<std::vector<Pose3>, std::vector<Pose3>, Pose3, Pose3> simulatePoseKoide(
  boost::optional<Pose3> eTh_,
  boost::optional<Pose3> wTo_)
{
  double xRange = 1.0;
  double yRange = 1.0;
  double zRange = 1.0;

  double xStep = 0.5;
  double yStep = 0.5;
  double zStep = 0.25;

  double xOffset = 1.0;
  double yOffset = 0.0;
  double zOffset = 0.5;

  double handToEyeTrans = 0.3;
  double handToEyeRotDeg = 90.0;

  const auto wTo = wTo_.value_or(
    Pose3(Rot3(), Vector3(0.3, 0.3, 0.0)));

  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<double> rotDistribution(0.0, handToEyeRotDeg * M_PI / 180.0);
  std::normal_distribution<double> transDistribution(0.0, handToEyeTrans);
  std::uniform_real_distribution<double> uniform(-1.0, std::nextafter(1.0, DBL_MAX));

  const auto eTh = eTh_.value_or(Pose3(Rot3::AxisAngle(
    Unit3(uniform(gen), uniform(gen), uniform(gen)), rotDistribution(gen)),
    transDistribution(gen) * Unit3(uniform(gen), uniform(gen), uniform(gen))));
  const auto hTe = eTh.inverse();

  // const auto hTe = Pose3(
  //   Rot3(
  //     1, 0, 0,
  //     0, -1, 0,
  //     0, 0, -1
  //   ), Vector3(0.1, 0.1, 0.1));
  // const auto eTh = hTe.inverse();

  std::vector<Pose3> wThList;
  std::vector<Pose3> eToList;
  for (double z = 0.0; z <= zRange; z += 1.0) {
    for (double y = -yRange; y <= yRange; y += 1.0) {
      for (double x = -xRange; x <= xRange; x += 1.0) {
        const auto wTe_t = Vector3(
          xOffset + xStep * x,
          yOffset + yStep * y,
          zOffset + zStep * z);

        const auto rotInit = Rot3::AxisAngle(Unit3(0.0, 1.0, 0.0), M_PI);
        const auto zfrom = rotInit * Vector3(0.0, 0.0, 1.0);
        const auto zto = (wTo.translation() - wTe_t).normalized();

        const auto angle = acos(zfrom.dot(zto));
        const auto axis = Unit3(zto.cross(zfrom));

        const auto wTe = Pose3(
            Rot3::AxisAngle(axis, -angle) * rotInit, wTe_t);
        const auto wTh = wTe * eTh;
        wThList.push_back(wTh);

        const auto eTo = wTe.inverse() * wTo;
        eToList.push_back(eTo);

        // std::cout << wTh << std::endl;
        // std::cout << eTo << std::endl;
      }
    }
  }

  return std::tie(wThList, eToList, hTe, wTo);
}

std::vector<Pose3> applyNoise(
  const std::vector<Pose3> &poses, double stdevTrans, double stdevRotDeg)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<double> rotDistribution(0.0, stdevRotDeg * M_PI / 180.0);
  std::normal_distribution<double> transDistribution(0.0, stdevTrans);
  std::uniform_real_distribution<double> uniform(-1.0, std::nextafter(1.0, DBL_MAX));

  std::vector<Pose3> result;
  for (auto pose : poses) {
    auto noise = Pose3(Rot3::AxisAngle(
      Unit3(uniform(gen), uniform(gen), uniform(gen)), rotDistribution(gen)),
      transDistribution(gen) * Unit3(uniform(gen), uniform(gen), uniform(gen)));

    result.push_back(noise * pose);
  }

  return result;
}

std::vector<Rot3> applyNoise(const std::vector<Rot3> &poses, double stdevRotDeg)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<double> rotDistribution(0.0, stdevRotDeg * M_PI / 180.0);
  std::uniform_real_distribution<double> uniform(-1.0, std::nextafter(1.0, DBL_MAX));

  std::vector<Rot3> result;
  for (auto pose : poses)
  {
    auto noise = Rot3::AxisAngle(
      Unit3(uniform(gen), uniform(gen), uniform(gen)), rotDistribution(gen));

    result.push_back(noise * pose);
  }

  return result;
}

std::vector<std::vector<Vector2>> projectPoints(
  const std::vector<Pose3> &eToList,
  const std::vector<Vector3> &objectPoints,
  const Cal3_S2::shared_ptr calibration)
{
  std::vector<std::vector<Vector2>> imagePointsList;
  for (auto eTo : eToList) {
    const auto oTe = eTo.inverse();
    const auto cam = PinholePose<Cal3_S2>(oTe, calibration);
    std::vector<Vector2> imagePoints;
    for (auto objectPoint : objectPoints) {
      imagePoints.push_back(cam.project(objectPoint));
    }
    imagePointsList.push_back(imagePoints);
  }

  return imagePointsList;
}

std::vector<Vector3> createTargetObject(int rows, int cols, double dimension)
{
  std::vector<Vector3> objectPoints;
  for (int row = 0; row < rows; row++) {
    for (int col = 0; col < cols; col++) {
      objectPoints.push_back(dimension * Vector3(
        row - 0.5 * (rows - 1),
        col - 0.5 * (cols - 1),
        0.0));
    }
  }

  return objectPoints;
}
