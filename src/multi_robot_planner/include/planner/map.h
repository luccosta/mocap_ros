#pragma once
#include "planner/types.h"
#include <vector>
namespace planner {

struct OccGrid {
  bool has_grid = false;
  int width = 0, height = 0; // cells
  double res = 0.2;          // [m/cell]
  double origin_x = 0.0, origin_y = 0.0; // world coords of cell (0,0)
  std::vector<int8_t> data; // row-major, length width*height, occupancy 0..100, -1 unknown
  inline bool inBounds(int ix, int iy) const { return ix>=0 && ix<width && iy>=0 && iy<height; }
  inline bool cellOccupied(int ix, int iy, int thresh = 50) const {
    if (!inBounds(ix,iy)) return true; // treat out of bounds as occupied
    int8_t v = data[iy*width + ix];
    if (v < 0) return false; // unknown -> free (conservative tuning later)
    return v >= thresh;
  }
};

struct MapRect { // axis-aligned obstacle
  double x_min, y_min, x_max, y_max;
};

struct GridMap {
  // Either use explicit rectangles OR an occupancy grid
  double res = 0.2; // default grid resolution if occ not set
  double x_min = -5.0, x_max = 5.0, y_min = -5.0, y_max = 5.0;
  std::vector<MapRect> obstacles; // in world coords

  OccGrid occ; // optional; if has_grid=true this takes precedence

  bool isOccupied(const Vec2& p) const;
};

class World {
public:
  GridMap map;
  bool segmentFree(const Vec2& a, const Vec2& b, double step = 0.05) const;
};

} // namespace planner
