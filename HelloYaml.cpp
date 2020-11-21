
#include "yaml-cpp/yaml.h"

#include <iostream>

int main(int argc, char* argv[])
{
  YAML::Emitter out;

  out << YAML::BeginMap;

  out << YAML::Key << "yeah";
  out << YAML::Value << YAML::BeginSeq;
  out << "eggs";
  out << "bread";
  out << "milk";
  out << YAML::EndSeq;

  out << YAML::Key << "oh";
  out << YAML::Value << 23.45;

  out << YAML::EndMap;

  std::cout << out.c_str() << std::endl;

  return 0;
}
