
#pragma once

#include <gtsam/geometry/Pose3.h>

#include <vector>

void simulatePoseKoide(
    std::vector<gtsam::Pose3>& wThList,
    std::vector<gtsam::Pose3>& eToList,
    gtsam::Pose3& hTe,
    gtsam::Pose3& wTo);

