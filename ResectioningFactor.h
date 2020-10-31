
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/PinholePose.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <boost/make_shared.hpp>
#include <boost/optional/optional_io.hpp>

using namespace gtsam;
using namespace gtsam::noiseModel;

class ResectioningFactor : public NoiseModelFactor1<Pose3>
{
    typedef NoiseModelFactor1<Pose3> Base;

    Cal3_S2::shared_ptr K_; ///< camera's intrinsic parameters
    Point3 P_;              ///< 3D point on the calibration rig
    Point2 p_;              ///< 2D measurement of the 3D point

public:
    /// Construct factor given known point P and its projection p
    ResectioningFactor(const SharedNoiseModel &model, const Key &key,
                       const Cal3_S2::shared_ptr &calib, const Point2 &p, const Point3 &P) 
                       : Base(model, key), K_(calib), P_(P), p_(p)
    {
    }

    /// evaluate the error
    Vector evaluateError(const Pose3 &pose, 
                         boost::optional<Matrix &> H = boost::none) const override
    {
        Matrix6 Dinverse;
        PinholePose<Cal3_S2> camera(pose.inverse(H ? &Dinverse : 0), K_);

        Matrix26 Dproject;
        const auto error = camera.project(P_, H ? &Dproject : 0, boost::none, boost::none) - p_;

        // Chain the Jacobians
        if (H) {
            *H = Dproject * Dinverse;
        }
        
        // std::cout << error << std::endl << std::flush;
        return error;
    }
};
