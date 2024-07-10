#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <random>

#include <Eigen/Eigen>

void BuildCorridor(
    const double origin_x, const double origin_y, 
    const double radius, 
    const std::vector<Eigen::Vector2d>& data, 
    std::vector<cv::Point2d>* const final_vertex,
    std::vector<Eigen::Vector3d>* const constrains) {

  constrains->clear();
  final_vertex->clear();
  double interior_x = 0.0;
  double interior_y = 0.0;
  double safe_radius = radius;
  std::vector<cv::Point2d> flipData(data.size()+1,cv::Point2f(0,0));
  for (size_t i=0; i<data.size(); i++) {
    double dx = data[i](0) - origin_x;
    double dy = data[i](1) - origin_y;
    double norm2 = std::sqrt(dx*dx + dy*dy);
    if (norm2 < safe_radius) safe_radius = norm2;
    if (norm2 == 0) continue;
    flipData[i].x = dx + 2*(radius-norm2)*dx/norm2;
    flipData[i].y = dy + 2*(radius-norm2)*dy/norm2;
  }
    
  std::vector<int> vertexIndice;
  cv::convexHull(flipData,vertexIndice,false,false);
    
  bool isOriginAVertex = false;
  int OriginIndex = -1;
  std::vector<cv::Point2f> vertexData;
  for (size_t i=0; i<vertexIndice.size(); i++) {
    int v = vertexIndice[i];
    if (v == data.size()) {
      isOriginAVertex = true;
      OriginIndex = i;
      vertexData.push_back(cv::Point2f(origin_x, origin_y));
    }else {
      vertexData.push_back(cv::Point2f(data[v](0), data[v](1)));
    }
  }

  if (isOriginAVertex) {
    int last_index = (OriginIndex - 1)%vertexIndice.size();
    int next_index = (OriginIndex + 1)%vertexIndice.size();
    double dx = (data[vertexIndice[last_index]](0) + origin_x + data[vertexIndice[next_index]](0))/3 - origin_x;
    double dy = (data[vertexIndice[last_index]](1) + origin_y + data[vertexIndice[next_index]](1))/3 - origin_y;
    double d = std::sqrt(dx*dx + dy*dy);
    interior_x = 0.99*safe_radius*dx/d + origin_x;
    interior_y = 0.99*safe_radius*dy/d + origin_y;
  }else {
    interior_x = origin_x;
    interior_y = origin_y;
  }

  std::vector<int> vIndex2;
  cv::convexHull(vertexData,vIndex2,false,false); // counterclockwise right-hand

  std::vector<Eigen::Vector3f> constraints; // (a,b,c) a x + b y <= c
  for (size_t j=0; j<vIndex2.size(); j++) {
    int jplus1 = (j+1)%vIndex2.size();
    cv::Point2f rayV = vertexData[vIndex2[jplus1]] - vertexData[vIndex2[j]];
    Eigen::Vector2f normalJ(rayV.y, -rayV.x);  // point to outside
    normalJ.normalize();
    int indexJ = vIndex2[j];
    while (indexJ != vIndex2[jplus1]) {
      double c = (vertexData[indexJ].x-interior_x) * normalJ(0) + (vertexData[indexJ].y-interior_y) * normalJ(1);
      constraints.push_back(Eigen::Vector3f(normalJ(0), normalJ(1), c));
      indexJ = (indexJ+1)%vertexData.size();
    }
  }    

  std::vector<cv::Point2f> dualPoints(constraints.size(), cv::Point2f(0,0));
  for (size_t i=0; i<constraints.size(); i++) {
    dualPoints[i].x = constraints[i](0)/constraints[i](2);
    dualPoints[i].y = constraints[i](1)/constraints[i](2);
  }
    
  std::vector<cv::Point2f> dualVertex;
  cv::convexHull(dualPoints,dualVertex,true,false);

  // std::vector<cv::Point2f> final_vertex;
  for (size_t i=0; i<dualVertex.size(); i++) {
    int iplus1 = (i+1)%dualVertex.size();
    cv::Point2f rayi = dualVertex[iplus1] - dualVertex[i];
    double c = rayi.y*dualVertex[i].x - rayi.x*dualVertex[i].y;
    final_vertex->push_back(cv::Point2f(interior_x+rayi.y/c, interior_y-rayi.x/c));
  }

  for (size_t i=0; i<final_vertex->size(); i++) {
    int iplus1 = (i+1)%final_vertex->size();
    cv::Point2f rayi = final_vertex[iplus1] - final_vertex[i];
    double c = rayi.y*final_vertex[i].x - rayi.x*final_vertex[i].y;
    constrains->push_back(Eigen::Vector3f(rayi.y, -rayi.x, c));
  }
}