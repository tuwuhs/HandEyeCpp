
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Pose3.h>

#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <sstream>
#include <vector>

using namespace gtsam;

struct Point2D 
{
  Vector2 p;
  int point3D_ID;
};

struct View
{
  Pose3 pose;
  std::vector<Point2D> points2D;
  int camera_ID;
  std::string name;
};

std::map<int, Vector3> loadPoints3D(std::string filename)
{
  std::map<int, Vector3> result;
  std::ifstream file(filename);

  if (!file.is_open() || !file.good()) 
    return result;

  std::string line;
  while (std::getline(file, line)) {
    std::size_t firstCharIdx = line.find_first_not_of(" ");
    
    // Skip empty lines and comments (a line starting with #)
    if (firstCharIdx == std::string::npos || line[firstCharIdx] == '#') 
      continue;

    // #   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)
    std::stringstream ss(line);
    int point3D_ID = -1;
    double x, y, z;
    if (!(ss >> point3D_ID >> x >> y >> z))
      continue;

    result[point3D_ID] = Vector3(x, y, z);
  }

  return result;
}

std::map<int, View> loadImages(std::string filename)
{
  std::map<int, View> result;
  std::ifstream file(filename);

  if (!file.is_open() || !file.good()) 
    return result;

  std::string line;
  while (std::getline(file, line)) {
    std::size_t firstCharIdx = line.find_first_not_of(" ");
    
    // Skip empty lines and comments (a line starting with #)
    if (firstCharIdx == std::string::npos || line[firstCharIdx] == '#') 
      continue;

    View view;

    // #   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
    std::stringstream ss1(line);
    int image_ID = -1;
    double qw, qx, qy, qz, tx, ty, tz;
    int camera_ID = -1;
    std::string name;
    if (!(ss1 >> image_ID >> qw >> qx >> qy >> qz >> tx >> ty >> tz >> camera_ID >> name))
      continue;
  
    view.pose = Pose3(
      Rot3::Quaternion(qw, qx, qy, qz),
      Vector3(tx, ty, tz)
    );
    view.name = name;
    view.camera_ID = camera_ID;

    // #   POINTS2D[] as (X, Y, POINT3D_ID)
    if (!std::getline(file, line))
      break;
    std::stringstream ss2(line);
    double x, y;
    int point3D_ID;
    while (ss2 >> x >> y >> point3D_ID) {
      if (point3D_ID < 0)
        continue;
      view.points2D.push_back(Point2D { Vector2(x, y), point3D_ID });
    }

    result[image_ID] = view;
  }

  return result;
}

std::map<int, Cal3DS2> loadCameras(std::string filename)
{
  std::map<int, Cal3DS2> result;
  std::ifstream file(filename);

  if (!file.is_open() || !file.good()) 
    return result;

  std::string line;
  while (std::getline(file, line)) {
    std::size_t firstCharIdx = line.find_first_not_of(" ");
    
    // Skip empty lines and comments (a line starting with #)
    if (firstCharIdx == std::string::npos || line[firstCharIdx] == '#') 
      continue;

    // #   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]
    std::stringstream ss(line);
    int camera_ID = -1;
    std::string model;
    double width, height, f, u0, v0, k1;
    if (!(ss >> camera_ID >> model >> width >> height >> f >> u0 >> v0 >> k1))
      continue;

    if (model.compare("SIMPLE_RADIAL") != 0)
      continue;

    result[camera_ID] = Cal3DS2(f, f, 0, u0, v0, k1, 0, 0);
  }

  return result;
}

std::vector<View> sortViews(std::map<int, View>& views)
{
  std::vector<std::pair<int, View>> pairs;
  std::copy(views.begin(), views.end(), std::back_inserter<std::vector<std::pair<int, View>>>(pairs));
  std::sort(pairs.begin(), pairs.end(),
    [](const std::pair<int, View>& l, const std::pair<int, View>& r) {
      auto val = l.second.name.compare(r.second.name);
      if (val != 0)
        return (val < 0);
      return l.first < r.first;
    });
  
  std::vector<View> result;  
  for (auto p: pairs) {
    // std::cout << p.first << " " << p.second.name << std::endl;
    result.push_back(p.second);
  }

  return result;
}

int main(int argc, char* argv[])
{
  auto cameras = loadCameras("cameras.txt");
  auto views = loadImages("images.txt");
  auto points = loadPoints3D("points3D.txt");
  auto viewsList = sortViews(views);

  for (auto camera: cameras) {
    std::cout << camera.first << std::endl;
    camera.second.print();
  }
  std::cout << std::endl;

  for (auto view: views) {
    auto v = view.second;
    std::cout << view.first << " " << v.name << " " << v.camera_ID << std::endl;
    v.pose.print();
    
    for (auto point2D: v.points2D) {
      std::cout << "- " << point2D.point3D_ID << " " << point2D.p.x() << " " << point2D.p.y() << std::endl;
    }
  }
  std::cout << std::endl;

  for (auto point: points) {
    auto p = point.second;
    std::cout << point.first << " " << p.x() << " " << p.y() << " " << p.z() << std::endl;
  }
  std::cout << std::endl;

  return 0;
}
