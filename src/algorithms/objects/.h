#ifndef IMAGE_H
#define IMAGE_H

#include "point_xy_greyscale.h"

#include <vector>

class PointCloudXYGreyscale
{
private:
    std::vector<point_xy_greyscale> _points;

public:
    PointCloudXYGreyscale();

    std::vector<point_xy_greyscale> points() { return _points; }
    std::vector<point_xy_greyscale>::iterator begin() { return _points.begin(); }
    std::vector<point_xy_greyscale>::iterator end() { return _points.end(); }

    void push_back(point_xy_greyscale pt_gs) { _points.push_back(pt_gs); }
    int size() { return _points.size(); }
};

#endif // IMAGE_H
