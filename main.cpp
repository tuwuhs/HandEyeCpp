
#include <iostream>

#include <gtsam/config.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

using namespace gtsam;

int main(int, char**) {
    std::cout << "Hello, world! GTSAM version: " << GTSAM_VERSION_STRING << std::endl;

    Pose3 pose(Rot3::Rodrigues(0.1, 0.2, 0.3), Point3(1.0, 2.0, 3.0));
    std::cout << pose << std::endl;
}
