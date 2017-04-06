#ifndef __FACEMORPH_H__
#define __FACEMORPH_H__

#include "image_utils.h"
#include "Delaunay.h"
#include <vector>
#include <string>

using namespace std;

/*
	Key Modules:

	*class FaceMorph:
        Fuse two images.
*/


typedef Img ColorSpace;
typedef vector<Img> ImgList;


class FaceMorph {
private:
    int step;   // the number of frames (middle images)
    Img src;    // source image
    Img tgt;    // target image
    PointList src_cps;  // source image's control points
    PointList tgt_cps;  // target image's control points
    ImgList middles;    // list of middle images fusing src and tgt

    //  pre-computed middle weights
    //  for example:
    //      if step is 3, middle_weights is [0.25, 0.5, 0.75]
    vector<double> middle_weights;

	DelaunayTriangulation src_dt;  // delaunay triangulation of src
	DelaunayTriangulation tgt_dt;  // delaunay triangulation of tgt


    void loadControlPoints(string, PointList &);
    void delaunayTriangulate(PointList &, DelaunayTriangulation &);
    // Paint the triangle in the colorspace that has the same size as the fusing image
    // with the color corresponding to the triangle's index in the dt's triangle lists
    // so that we can know which triangle a point is included in based on its color in
    // its corresponding position in the colorspace.
    void colorSpace(DelaunayTriangulation &, ColorSpace &);
    // Calculate an affine translation matrix for each corresponding triangles pair.
	void calcTranMats(DelaunayTriangulation &, DelaunayTriangulation &, MatList &);
    // Generate N fusing images, N=step 
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
