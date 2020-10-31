
#pragma once

#include <gtsam/geometry/Pose3.h>

#include <vector>

std::tuple<
    std::vector<gtsam::Pose3>,
    std::vector<gtsam::Pose3>,
    gtsam::Pose3,
    gtsam::Pose3
> simulatePoseKoide(
    boost::optional<gtsam::Pose3> eTh_ = boost::none,
    boost::optional<gtsam::Pose3> wTo_ = boost::none);

std::vector<gtsam::Pose3> applyNoise(
    std::vector<gtsam::Pose3> poses, double stdevTrans, double stdevRotDeg);

std::vector<gtsam::Rot3> applyNoise(
    std::vector<gtsam::Rot3> poses, double stdevRotDeg);
