
#include "PoseSimulation.h"

#include <gtsam/geometry/Pose3.h>

#include <random>
#include <vector>

std::tuple<
    std::vector<gtsam::Pose3>,
    std::vector<gtsam::Pose3>,
    gtsam::Pose3,
    gtsam::Pose3
> simulatePoseKoide()
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

    const auto wTo = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0));

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> rotDistribution(0.0, handToEyeRotDeg * M_PI / 180.0);
    std::normal_distribution<double> transDistribution(0.0, handToEyeTrans);
    std::uniform_real_distribution<double> uniform(-1.0, std::nextafter(1.0, DBL_MAX));

    const auto eTh = gtsam::Pose3(gtsam::Rot3::AxisAngle(
        gtsam::Unit3(uniform(gen), uniform(gen), uniform(gen)), rotDistribution(gen)
    ), transDistribution(gen) * gtsam::Unit3(uniform(gen), uniform(gen), uniform(gen)));

    const auto hTe = eTh.inverse();

    std::vector<gtsam::Pose3> wThList;
    std::vector<gtsam::Pose3> eToList;
    for (double z = 0.0; z <= zRange; z += 1.0) {
        for (double y = -yRange; y <= yRange; y += 1.0) {
            for (double x = -xRange; x <= xRange; x += 1.0) {
                const auto wTe_t = gtsam::Point3(
                    xOffset + xStep * x,
                    yOffset + yStep * y,
                    zOffset + zStep * z);
                
                const auto rotInit = gtsam::Rot3::AxisAngle(gtsam::Unit3(0.0, 1.0, 0.0), M_PI);
                const auto zfrom = rotInit * gtsam::Point3(0.0, 0.0, 1.0);
                const auto zto = (wTo.translation() - wTe_t).normalized();

                const auto angle = acos(zfrom.dot(zto));
                const auto axis = gtsam::Unit3(zto.cross(zfrom));

                const auto wTe = gtsam::Pose3(
                    gtsam::Rot3::AxisAngle(axis, -angle) * rotInit, wTe_t);
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

std::vector<gtsam::Pose3> applyNoise(
    std::vector<gtsam::Pose3> poses, double stdevTrans, double stdevRotDeg)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> rotDistribution(0.0, stdevRotDeg * M_PI / 180.0);
    std::normal_distribution<double> transDistribution(0.0, stdevTrans);
    std::uniform_real_distribution<double> uniform(-1.0, std::nextafter(1.0, DBL_MAX));

    std::vector<gtsam::Pose3> result;
    for (auto pose: poses) {
        auto noise = gtsam::Pose3(gtsam::Rot3::AxisAngle(
            gtsam::Unit3(uniform(gen), uniform(gen), uniform(gen)), rotDistribution(gen)
        ), transDistribution(gen) * gtsam::Unit3(uniform(gen), uniform(gen), uniform(gen)));

        result.push_back(noise * pose);
    }

    return result;
}

std::vector<gtsam::Rot3> applyNoise(
    std::vector<gtsam::Rot3> poses, double stdevRotDeg)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> rotDistribution(0.0, stdevRotDeg * M_PI / 180.0);
    std::uniform_real_distribution<double> uniform(-1.0, std::nextafter(1.0, DBL_MAX));

    std::vector<gtsam::Rot3> result;
    for (auto pose: poses) {
        auto noise = gtsam::Rot3::AxisAngle(
            gtsam::Unit3(uniform(gen), uniform(gen), uniform(gen)), rotDistribution(gen));

        result.push_back(noise * pose);
    }

    return result;
}
