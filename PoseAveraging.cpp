
#include "PoseSimulation.h"

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/geometry/Rot3.h>

#include <iostream>
#include <vector>

using namespace gtsam;
using namespace gtsam::noiseModel;
using symbol_shorthand::X;

int main() 
{
    NonlinearFactorGraph graph;

    auto noise = nullptr; //Diagonal::Sigmas(Vector3(1, 1, 1));

    std::vector<Rot3> cleanPoses;
    for (int i = 0; i < 12; i++) {
        cleanPoses.push_back(Rot3::Rodrigues(1.0, 0.0, 0.0));
    }

    auto poses = applyNoise(cleanPoses, 1);
    for (auto pose: poses) {
       graph.emplace_shared<PriorFactor<Rot3>>(X(1), pose, noise); 
    }

    // graph.emplace_shared<PriorFactor<Rot3>>(X(1), Rot3::Rodrigues(1.0, 0.0, 0.0), noise);

    Values initial;
    initial.insert(X(1), Rot3::Rodrigues(0.0, 0.1, 0.1));

    Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
    result.print("Result: ");

    auto axisAngle = result.at(X(1)).cast<Rot3>().axisAngle();
    auto axis = axisAngle.first;
    auto angle = axisAngle.second;
    std::cout << angle * axis << std::endl;

    return 0;
}