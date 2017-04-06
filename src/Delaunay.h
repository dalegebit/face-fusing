#ifndef __DELAUNAY_H__
#define __DELAUNAY_H__

#include <algorithm>
#include <memory>
#include <iostream>
#include <vector>
#include <array>

/*
	Key Modules:

	*class DelaunayTriangulation:
		Applying Delaunay Triangulation to a given list of points.
		An incremental algorithm, Bowyer-Watson algorithm specifically, is implemented.
		Refs: https://en.wikipedia.org/wiki/Bowyer%E2%80%93Watson_algorithm

	*class DPoint:
		Point type used in DelaunayTriangulation

	*class DTriangle:
		Triangle type used in DelaunayTriangulation
*/


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

	//	Given the width and height of the map that includes all points.
	//	4 new points are created:
	//		(0, 0), (width, 0) (width, height) (0, height)
	//	2 new triangles are created:
	//		{(0, 0), (width, 0), (width, height)}
	//		{(0, 0), (width, height), (0, height)}
	DelaunayTriangulation(int width, int height);

	// print details
	void print();

	void AddPoint(DPoint p);


private:

	// Check whether the circumcirlce of T contains p
	bool CircumcircleContains(Tptr T, DPoint p);

	// Get the edges of the convex hull of all triangles whose
	// circumcircles contains the new point
	std::vector<Edge> GetBoundary(std::vector<Tptr> bad_triangles);
};


#endif
