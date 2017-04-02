#ifndef __DELAUNAY_H__
#define __DELAUNAY_H__

#include <algorithm>
#include <memory>
#include <iostream>
#include <vector>
#include <array>

class DPoint;
class DTriangle;

typedef std::shared_ptr<DTriangle>  Tptr;
typedef std::tuple<int, int, Tptr> Edge;

class DPoint
{
public:

	double x, y, z;

	DPoint(double x, double y) : x(x), y(y), z(0) {}
	DPoint(double x, double y, double z) : x(x), y(y), z(z) {}
	bool operator== (const DPoint &b)
	{
		return x == b.x && y == b.y && z == b.z;
	}
};

class DTriangle
{
public:

	std::array<int, 3> v;     // Holds the verticies
	std::array<Tptr, 3> n;   // Holds the neighbours

	DTriangle(int a, int b, int c)
		: v{ { a, b, c } }
	{
		n.fill(nullptr);
	}
	bool operator== (const DTriangle &b)
	{
		return v[0]==b.v[0] && v[1]==b.v[1] && v[2]==b.v[2];
	}
	void SetEdge(const Edge edge, const Tptr T);
};

class DelaunayTriangulation
{
public:

	std::vector< DPoint > points;
	std::vector< Tptr >  triangles;

	DelaunayTriangulation() {}

	DelaunayTriangulation(int width, int height);


	void print();

	void AddPoint(DPoint p);


private:

	// Check whether the circumcirlce of T contains p
	bool CircumcircleContains(Tptr T, DPoint p);

	std::vector<Edge> GetBoundary(std::vector<Tptr> bad_triangles);
};


#endif
