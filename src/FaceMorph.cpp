#include "FaceMorph.h"
#include "image_utils.h"
#include <fstream>
#include <string>
#include <stack>

using namespace std;

FaceMorph::FaceMorph(Img & src_img, string src_cps_path, Img & tgt_img, string tgt_cps_path, int n_step) {
    src = Img(src_img, false);
    tgt = Img(tgt_img, false);
	src_dt = DelaunayTriangulation(src._width, src._height);
	tgt_dt = DelaunayTriangulation(tgt._width, tgt._height);
    src_cps = PointList();
    tgt_cps = PointList();
    step = n_step;
    middle_weights = vector<double>();

    loadControlPoints(src_cps_path, src_cps);
    loadControlPoints(tgt_cps_path, tgt_cps);

    for (int i = 0; i < step; ++i)
        middle_weights.push_back(double(i+1)/(step+1));
}

void FaceMorph::loadControlPoints(string cps_path, PointList & pts) {
    ifstream in(cps_path, ios::in);

    double x, y;
    while (in >> x >> y)
        pts.push_back(Point(x, y));
    in.close();
}

void FaceMorph::delaunayTriangulate(PointList & pts, DelaunayTriangulation & dt) {
    for (auto p : pts)
        dt.AddPoint(DPoint(p.x, p.y));
}

void FaceMorph::colorSpace(DelaunayTriangulation & dt, ColorSpace & cs) {

    for (int i = 0; i < dt.triangles.size(); ++i) {
		Img texture(cs._width, cs._height, 1, 1, i);
        auto t = dt.triangles[i];
		cs.draw_triangle(dt.points[t->v[0]].x, dt.points[t->v[0]].y, dt.points[t->v[1]].x, dt.points[t->v[1]].y, dt.points[t->v[2]].x, dt.points[t->v[2]].y, texture,
			dt.points[t->v[0]].x, dt.points[t->v[0]].y, dt.points[t->v[1]].x, dt.points[t->v[1]].y, dt.points[t->v[2]].x, dt.points[t->v[2]].y);

    }
}

void FaceMorph::calcTranMats(DelaunayTriangulation & src_dt, DelaunayTriangulation & tgt_dt, MatList & mts) {
	Mat m0 = Mat(3, 3);
	Mat m1 = Mat(3, 3);
	Mat a = Mat(3, 3);
	int cnt = 0;
	for (auto t : src_dt.triangles) {
        m0(0, 0) = src_dt.points[t->v[0]].x;
		m0(1, 0) = src_dt.points[t->v[1]].x;
		m0(2, 0) = src_dt.points[t->v[2]].x;
		m0(0, 1) = src_dt.points[t->v[0]].y;
		m0(1, 1) = src_dt.points[t->v[1]].y;
		m0(2, 1) = src_dt.points[t->v[2]].y;
		m0(0, 2) = 1;
		m0(1, 2) = 1;
		m0(2, 2) = 1;
		int idx = 0;
		for (int i = 0; i < tgt_dt.triangles.size(); ++i) {
			if (*t == *tgt_dt.triangles[i]) {
				idx = i;
				// cout << idx << " " << cnt << endl;
				break;
			}
		}

		m1(0, 0) = tgt_dt.points[t->v[0]].x;
		m1(1, 0) = tgt_dt.points[t->v[1]].x;
		m1(2, 0) = tgt_dt.points[t->v[2]].x;
		m1(0, 1) = tgt_dt.points[t->v[0]].y;
		m1(1, 1) = tgt_dt.points[t->v[1]].y;
		m1(2, 1) = tgt_dt.points[t->v[2]].y;
		m1(0, 2) = 1;
		m1(1, 2) = 1;
		m1(2, 2) = 1;
        a = m1 / m0;
		mts.push_back(Mat(a, false));
		cnt += 1;
	}
}


void FaceMorph::morphMiddles(Img & src, Img & tgt, DelaunayTriangulation & src_dt, DelaunayTriangulation & tgt_dt, PointList & src_cps, PointList & tgt_cps, ImgList & mids) {
	for (int i = 0; i < step; ++i) {
		Img mid(src, false);
		PointList mid_pts;
		ColorSpace mid_color(mid._width, mid._height, 1, 1, 0);
		MatList src_tran_mats;
		MatList tgt_tran_mats;
		for (int j = 0; j < src_cps.size(); ++j) {
			double x = middle_weights[i] * (tgt_cps[j].x - src_cps[j].x) + src_cps[j].x;
			double y = middle_weights[i] * (tgt_cps[j].y - src_cps[j].y) + src_cps[j].y;
            mid_pts.push_back(Point(x,y));
		}
		DelaunayTriangulation mid_dt(mid._width, mid._height);
		delaunayTriangulate(mid_pts, mid_dt);
		colorSpace(mid_dt, mid_color);
		calcTranMats(mid_dt, src_dt, src_tran_mats);
		calcTranMats(mid_dt, tgt_dt, tgt_tran_mats);
		Mat src_pos(1, 3), tgt_pos(1, 3), mid_pos(1, 3);
		cimg_forXY(mid, x, y) {
			int color = mid_color(x, y);
			mid_pos(0, 0) = x, mid_pos(0, 1) = y, mid_pos(0, 2) = 1;
			src_pos = src_tran_mats[color] * mid_pos;
			tgt_pos = tgt_tran_mats[color] * mid_pos;
			mid(x, y, 0) = (1 - middle_weights[i])* src.linear_atXY(src_pos(0, 0), src_pos(0, 1), 0, 0) +
							middle_weights[i] * tgt.linear_atXY(tgt_pos(0, 0), tgt_pos(0, 1), 0, 0);
			mid(x, y, 1) = (1 - middle_weights[i])* src.linear_atXY(src_pos(0, 0), src_pos(0, 1), 0, 1) +
							middle_weights[i] * tgt.linear_atXY(tgt_pos(0, 0), tgt_pos(0, 1), 0, 1);
			mid(x, y, 2) = (1 - middle_weights[i])* src.linear_atXY(src_pos(0, 0), src_pos(0, 1), 0, 2) +
							middle_weights[i] * tgt.linear_atXY(tgt_pos(0, 0), tgt_pos(0, 1), 0, 2);
		}
		mids.push_back(Img(mid, false));
	}
}

ImgList FaceMorph::getMorphMiddles() {
    delaunayTriangulate(src_cps, src_dt);
    delaunayTriangulate(tgt_cps, tgt_dt);
    morphMiddles(src, tgt, src_dt, tgt_dt, src_cps, tgt_cps, middles);
	return middles;
}

void FaceMorph::plotControlPoints(Img & img, PointList & pts) {
    Img draw_img(img, false);
    const unsigned char color_point[] = {0, 255, 255};
    for (auto p : pts)
        draw_img.draw_circle(p.x, p.y, 2, color_point);
    draw_img.display();
}

void FaceMorph::plotTriangles(Img & img, DelaunayTriangulation & dt) {
    Img draw_img(img, false);
    const unsigned char color_point[] = {0, 255, 255};
    const unsigned char color_line[] = {0, 0, 255};
    for (auto p : dt.points)
        draw_img.draw_circle(p.x, p.y, 2, color_point);
    for (auto t : dt.triangles)
        draw_img.draw_triangle(dt.points[t->v[0]].x, dt.points[t->v[0]].y, dt.points[t->v[1]].x,
								dt.points[t->v[1]].y, dt.points[t->v[2]].x, dt.points[t->v[2]].y, color_line, 1, ~0U);
    draw_img.display();
}

void FaceMorph::plot() {
	plotControlPoints(src, src_cps);
	plotControlPoints(tgt, tgt_cps);
	plotTriangles(src, src_dt);
	plotTriangles(tgt, tgt_dt);
	for (auto mid : middles)
		mid.display();
}

void FaceMorph::test() {

	plotControlPoints(src, src_cps);
	plotControlPoints(tgt, tgt_cps);
	delaunayTriangulate(src_cps, src_dt);
	delaunayTriangulate(tgt_cps, tgt_dt);
	plotTriangles(src, src_dt);
	plotTriangles(tgt, tgt_dt);
	// ColorSpace src_color(src._width, src._height, 1, 1, 0);
	// colorSpace(src_dt, src_color);
	// src_color.display();
	// ColorSpace tgt_color(tgt._width, tgt._height, 1, 1, 0);
	// colorSpace(tgt_dt, tgt_color);
	// tgt_color.display();
	morphMiddles(src, tgt, src_dt, tgt_dt, src_cps, tgt_cps, middles);
	for (auto mid : middles)
		mid.display();
	saveMiddles("example/1to2");
}

void FaceMorph::saveMiddles(string path_prefix) {
    int i = 0;
	char path[100];
	for (auto img : middles) {
		sprintf(path, "%s.%d.bmp", path_prefix.c_str(), i);
		img.save_bmp(path);
        i ++;
    }
}
