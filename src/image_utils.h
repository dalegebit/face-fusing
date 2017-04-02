#ifndef __IMAGE_UTILS_H__
#define __IMAGE_UTILS_H__

#include "CImg.h"
#include <vector>

using namespace cimg_library;
using namespace std;

#define SQRT2 1.4142135623731
#define PI 3.14159265358979323846
#define EPSILON 0.000000001

typedef CImg<float> Img;
typedef CImg<double> Mat;
typedef vector<Mat> MatList;

struct Point {
    double x, y;
    Point(double x_in, double y_in):x(x_in), y(y_in) {}
};

typedef vector<Point> PointList;

struct PointWithNeighbor {
    double x, y;
    PointList neighbor;
    PointWithNeighbor(Point & p):x(p.x), y(p.y) {}
    PointWithNeighbor(Point & p, PointList & pl):x(p.x), y(p.y), neighbor(pl) {}
};

typedef vector<PointWithNeighbor> PointWithNeighborList;


#endif
