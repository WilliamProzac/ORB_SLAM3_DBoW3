#pragma once
#include <algorithm>
#include <cmath>
namespace ORB_SLAM3 {
// All callers use undistorted matching bounds. Closed right/bottom edges
// belong to the final cell; invalid/out-of-bounds coordinates return -1.
inline int LocalMapGridCell(float x,float y,float min_x,float max_x,
    float min_y,float max_y,int columns,int rows) {
  const float width=max_x-min_x,height=max_y-min_y;
  if(columns<=0 || rows<=0 || !(width>0 && height>0) ||
     !std::isfinite(width) || !std::isfinite(height) ||
     !std::isfinite(x) || !std::isfinite(y) ||
     x<min_x || x>max_x || y<min_y || y>max_y) return -1;
  const int cx=std::min(columns-1,int(columns*((x-min_x)/width)));
  const int cy=std::min(rows-1,int(rows*((y-min_y)/height)));
  return cy*columns+cx;
}
} // namespace ORB_SLAM3
