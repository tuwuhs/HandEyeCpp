
#include "PoseSimulation.h"

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/geometry/Rot3.h>

#include <iostream>
#include <vector>

#include <boost/optional/optional_io.hpp>

using namespace gtsam;
using namespace gtsam::noiseModel;
using symbol_shorthand::X;

class MyPriorFactor: public NoiseModelFactor1<Rot3> {
private:
    typedef NoiseModelFactor1<Rot3> Base;

    Rot3 prior_;

public:
    MyPriorFactor(Key key, const Rot3& prior, const SharedNoiseModel& model = nullptr)
    : Base(model, key), prior_(prior) {
    }

    Vector evaluateError(const Rot3& x, boost::optional<Matrix&> H = boost::none) const override {
        Matrix3 Dinverse1;
        Matrix3 Dinverse2;
        auto error = prior_.localCoordinates(x.inverse(H ? &Dinverse1 : 0).inverse(H ? &Dinverse2 : 0));

        // Chain the Jacobians
        if (H) {
            *H = Dinverse2 * Dinverse1;
        }        
        // std::cout << H << std::endl << std::flush;

        return error;
    }
};

int main() 
{
    NonlinearFactorGraph graph;

    auto noise = nullptr; //Diagonal::Sigmas(Vector3(1, 1, 1));

    std::vector<Rot3> cleanPoses;
    for (int i = 0; i < 12; i++) {
        cleanPoses.push_back(Rot3::Rodrigues(1.0, 0.0, 0.0));
    }

    auto poses = applyNoise(cleanPoses, 0);
    for (auto pose: poses) {
       graph.emplace_shared<MyPriorFactor>(X(1), pose, noise); 
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