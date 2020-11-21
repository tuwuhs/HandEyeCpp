
#pragma once

#include <yaml-cpp/yaml.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Pose3.h>

namespace YAML
{
  using namespace gtsam;

  template<>
  struct convert<Vector3> 
  {
    static YAML::Node encode(const Vector3& rhs)
    {
      YAML::Node node;
      node.push_back(rhs[0]);
      node.push_back(rhs[1]);
      node.push_back(rhs[2]);
      return node;
    }

    static bool decode(const YAML::Node& node, Vector3& rhs) 
    {
      if (!node.IsSequence() || node.size() != 3) {
        return false;
      }

      rhs = Vector3(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
      return true;
    }
  };

  template<>
  struct convert<Vector2> 
  {
    static YAML::Node encode(const Vector2& rhs) 
    {
      YAML::Node node;
      node.push_back(rhs[0]);
      node.push_back(rhs[1]);
      return node;
    }

    static bool decode(const YAML::Node& node, Vector2& rhs) 
    {
      if (!node.IsSequence() || node.size() != 2) {
        return false;
      }

      rhs = Vector2(node[0].as<double>(), node[1].as<double>());
      return true;
    }
  };

  template<>
  struct convert<Cal3_S2>
  {
    static YAML::Node encode(const Cal3_S2& rhs) 
    {
      YAML::Node node;
      node["fx"] = rhs.fx();
      node["fy"] = rhs.fy();
      node["s"] = rhs.skew();
      node["u0"] = rhs.px();
      node["v0"] = rhs.py();
      return node;
    }

    static bool decode(const YAML::Node& node, Cal3_S2& rhs) 
    {
      auto fx = node["fx"];
      auto fy = node["fy"];
      auto s = node["s"];
      auto u0 = node["u0"];
      auto v0 = node["v0"];
      if (!node.IsMap() || !fx || !fy || !s || !u0 || !v0) {
        return false;
      }

      rhs = Cal3_S2(
        fx.as<double>(),
        fy.as<double>(),
        s.as<double>(),
        u0.as<double>(),
        v0.as<double>()
      );
      return true;
    }
  };

  template<>
  struct convert<Cal3DS2>
  {
    static YAML::Node encode(const Cal3DS2& rhs) 
    {
      YAML::Node node;
      node["fx"] = rhs.fx();
      node["fy"] = rhs.fy();
      node["s"] = rhs.skew();
      node["u0"] = rhs.px();
      node["v0"] = rhs.py();
      auto distortionCoefficients = std::vector<double>();
      distortionCoefficients.push_back(rhs.k1());
      distortionCoefficients.push_back(rhs.k2());
      distortionCoefficients.push_back(rhs.p1());
      distortionCoefficients.push_back(rhs.p2());
      distortionCoefficients.push_back(0.0);
      node["distortion_coefficients"] = distortionCoefficients;
      return node;
    }

    static bool decode(const YAML::Node& node, Cal3DS2& rhs) 
    {
      auto fx = node["fx"];
      auto fy = node["fy"];
      auto s = node["s"];
      auto u0 = node["u0"];
      auto v0 = node["v0"];
      auto distortionCoefficients = node["distortion_coefficients"];
      if (!node.IsMap() || !fx || !fy || !s || !u0 || !v0 || !distortionCoefficients) {
        return false;
      }

      rhs = Cal3DS2(
        fx.as<double>(),
        fy.as<double>(),
        s.as<double>(),
        u0.as<double>(),
        v0.as<double>(),
        distortionCoefficients[0].as<double>(),
        distortionCoefficients[1].as<double>(),
        distortionCoefficients[2].as<double>(),
        distortionCoefficients[3].as<double>()
      );
      return true;
    }
  };

  template<>
  struct convert<Pose3>
  {
    static YAML::Node encode(const Pose3& rhs) 
    {
      YAML::Node node;
      node["rvec"] = Rot3::Logmap(rhs.rotation());
      node["tvec"] = rhs.translation();
      return node;
    }

    static bool decode(const YAML::Node& node, Pose3& rhs) 
    {
      auto rvec = node["rvec"];
      auto tvec = node["tvec"];
      if (!node.IsMap() || !rvec || !tvec) {
        return false;
      }

      rhs = Pose3(
        Rot3::Rodrigues(rvec.as<Vector3>()),
        tvec.as<Vector3>()
      );
      return true;
    }
  };
}
