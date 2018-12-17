#pragma once
#ifndef PLANEEDGE_H
#define PLANEEDGE_H
#include <iostream>
#include <vector>
#include <stdexcept>
#include <math.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/common/geometry.h>
class Plane;

using namespace std;
typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointXYZRGBNormal PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

enum EdgeType {
	EdgeLeft,
	EdgeRight,
	EdgeDown,
	EdgeUp,
	EdgeNone,
};

class PlaneEdge {
private:
	PointT _p;
	PointT _q;
public:
	Plane* plane;
	PlaneEdge* connectedEdge;
	EdgeType edgeType;
	
	PlaneEdge(PointT p, PointT q);
	PlaneEdge(PointT p, PointT q, EdgeType type);
	float length();
};

#endif