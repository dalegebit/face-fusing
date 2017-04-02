#ifndef __FACEMORPH_H__
#define __FACEMORPH_H__

#include "image_utils.h"
#include "Delaunay.h"
#include <vector>
#include <string>

using namespace std;

typedef Img ColorSpace;
typedef vector<Img> ImgList;


class FaceMorph {
private:
    int step;
    Img src;
    Img tgt;
    PointList src_cps;
    PointList tgt_cps;
    ImgList middles;
    vector<double> middle_weights;
	DelaunayTriangulation src_dt;
	DelaunayTriangulation tgt_dt;


    void loadControlPoints(string, PointList &);
    void delaunayTriangulate(PointList &, DelaunayTriangulation &);
    void colorSpace(DelaunayTriangulation &, ColorSpace &);
	void calcTranMats(DelaunayTriangulation &, DelaunayTriangulation &, MatList &);
	void morphMiddles(Img &, Img &, DelaunayTriangulation &, DelaunayTriangulation &, PointList &, PointList &, ImgList &);

public:
    FaceMorph(Img &, string, Img &, string, int=11);
    ImgList getMorphMiddles();
    void plotControlPoints(Img &, PointList &);
    void plotTriangles(Img &, DelaunayTriangulation &);
	void plot();
    void saveMiddles(string);
	void test();
};


#endif /* end of include guard: __FACEMORPH_H__ */
