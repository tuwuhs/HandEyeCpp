
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

  Pose3 oTe_;
  Pose3 wTh_;

public:
  FixedHandEyePoseFactor(const SharedNoiseModel &model, const Key &hTe, const Key &wTo,
    const Pose3 oTe, const Pose3 wTh)
    : Base(model, hTe, wTo), oTe_(oTe), wTh_(wTh)
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

    // // 1st variant
    // auto oTe = wTo.inverse(H1).compose(wTh_ * hTe, H2, H3);

    // auto error = oTe_.localCoordinates(oTe, boost::none, Hlocal);

    // if (HhTe)
    //   *HhTe = Hlocal * H3;

    // if (HwTo)
    //   *HwTo = Hlocal * H2 * H1;

    // 2nd variant
    auto wTe = wTo.compose(oTe_, H1, boost::none);
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
    const Key &oTe, const Pose3 wTh)
    : Base(model, hTe, wTo, oTe), wTh_(wTh)
  {
  }

  Vector evaluateError(const Pose3 &hTe, const Pose3 &wTo, const Pose3 &oTe,
    boost::optional<Matrix &> HhTe = boost::none,
    boost::optional<Matrix &> HwTo = boost::none,
    boost::optional<Matrix &> HoTe = boost::none) const override
  {
    Matrix6 H1;
    Matrix6 H2;
    Matrix6 H3;
    Matrix6 H4;
    Matrix6 H5;
    Matrix6 Hlocal;
    Matrix6 Hlocal1;

    auto wTe = wTo.compose(oTe, H1, H2);
    auto wTh = wTe.compose(hTe.inverse(H3), H4, H5);
    auto error = wTh_.localCoordinates(wTh, Hlocal1, Hlocal);

    if (HhTe)
      *HhTe = H5 * H3;

    if (HwTo)
      *HwTo = H4 * H1;

    if (HoTe)
      *HoTe = H4 * H2;

    // std::cout << H1 << std::endl;
    // std::cout << H2 << std::endl;
    // std::cout << H3 << std::endl;
    // std::cout << H4 << std::endl;
    // std::cout << H5 << std::endl;
    // std::cout << Hlocal1 << std::endl;
    // std::cout << Hlocal << std::endl;
    // std::cout << error << std::endl << std::flush;

    return error;
  }
};
