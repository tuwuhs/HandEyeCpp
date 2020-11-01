
#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2.h>

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
    const std::vector<gtsam::Pose3>& poses, double stdevTrans, double stdevRotDeg);

std::vector<gtsam::Rot3> applyNoise(
    const std::vector<gtsam::Rot3>& poses, double stdevRotDeg);

std::vector<std::vector<gtsam::Vector2>> projectPoints(
    const std::vector<gtsam::Pose3>& eToList,
    const std::vector<gtsam::Vector3>& objectPoints,
    const boost::shared_ptr<gtsam::Cal3_S2> calibration);

std::vector<gtsam::Vector3> createTargetObject(int rows, int cols, double dimension);
