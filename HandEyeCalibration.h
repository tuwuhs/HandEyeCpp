
#pragma once

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/PinholePose.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <boost/make_shared.hpp>
#include <boost/optional/optional_io.hpp>

#include <vector>

using namespace gtsam;
using namespace gtsam::noiseModel;

class FixedHandEyePoseFactor : public NoiseModelFactor2<Pose3, Pose3>
{
  typedef NoiseModelFactor2<Pose3, Pose3> Base;

  Pose3 eTo_;
  Pose3 wTh_;

public:
  FixedHandEyePoseFactor(const SharedNoiseModel &model, const Key &hTe, const Key &wTo,
    const Pose3 eTo, const Pose3 wTh)
    : Base(model, hTe, wTo), eTo_(eTo), wTh_(wTh)
  {
  }

  // Inspired from BetweenFactor
  Vector evaluateError(const Pose3 &hTe, const Pose3 &wTo,
    boost::optional<Matrix &> HhTe = boost::none,
    boost::optional<Matrix &> HwTo = boost::none) const override
  {
    Matrix6 H1;
    Matrix6 H2;
    Matrix6 H3;
    Matrix6 H4;
    Matrix6 Hlocal;

    // 1st variant
    // // auto eTo = hTe.between(wTh_.inverse() * wTo, H1, H2);
    // auto eTo = hTe.inverse(H1).compose(wTh_.inverse() * wTo, H2, H3);

    // auto error = eTo_.localCoordinates(eTo, boost::none, Hlocal);

    // if (HhTe)
    //   *HhTe = Hlocal * H2 * H1;

    // if (HwTo)
    //   *HwTo = Hlocal * H3;

    // 2nd variant
    auto wTe = wTo.compose(eTo_.inverse(), H1, boost::none);
    auto wTh = wTe.compose(hTe.inverse(H2), H3, H4);

    auto error = wTh_.localCoordinates(wTh, boost::none, Hlocal);

    if (HhTe)
      *HhTe = H4 * H2;

    if (HwTo)
      *HwTo = H3 * H1;

    // std::cout << H1 << std::endl;
    // std::cout << H2 << std::endl;
    // std::cout << H3 << std::endl;
    // std::cout << H4 << std::endl;
    // std::cout << Hlocal << std::endl;
    // std::cout << error << std::endl << std::flush;

    return error;
  }
};

class HandEyePoseFactor : public NoiseModelFactor3<Pose3, Pose3, Pose3>
{
  typedef NoiseModelFactor3<Pose3, Pose3, Pose3> Base;

  Pose3 wTh_;

public:
  HandEyePoseFactor(const SharedNoiseModel &model, const Key &hTe, const Key &wTo,
    const Key &eTo, const Pose3 wTh)
    : Base(model, hTe, wTo, eTo), wTh_(wTh)
  {
  }

  Vector evaluateError(const Pose3 &hTe, const Pose3 &wTo, const Pose3 &eTo,
    boost::optional<Matrix &> HhTe = boost::none,
    boost::optional<Matrix &> HwTo = boost::none,
    boost::optional<Matrix &> HeTo = boost::none) const override
  {
    Matrix6 H1;
    Matrix6 H2;
    Matrix6 H3;
    Matrix6 H4;
    Matrix6 H5;
    Matrix6 H6;
    Matrix6 Hlocal;
    Matrix6 Hlocal1;

    auto wTe = wTo.compose(eTo.inverse(H1), H2, H3);
    auto wTh = wTe.compose(hTe.inverse(H4), H5, H6);
    auto error = wTh_.localCoordinates(wTh, Hlocal1, Hlocal);

    if (HhTe)
      *HhTe = H6 * H4;

    if (HwTo)
      *HwTo = H5 * H2;

    if (HeTo)
      *HeTo = H5 * H3 * H1;

    // std::cout << H1 << std::endl;
    // std::cout << H2 << std::endl;
    // std::cout << H3 << std::endl;
    // std::cout << H4 << std::endl;
    // std::cout << H5 << std::endl;
    // std::cout << H6 << std::endl;
    // std::cout << Hlocal1 << std::endl;
    // std::cout << Hlocal << std::endl;
    // std::cout << error << std::endl << std::flush;

    return error;
  }
};
