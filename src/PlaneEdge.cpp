#include <iostream>
#include <vector>
#include "PlaneEdge.h"
using namespace std;


PlaneEdge::PlaneEdge(PointT p, PointT q, EdgeType type) {
	this->_p = p;
	this->_q = q;
	this->edgeType = type;
}

PlaneEdge::PlaneEdge(PointT p, PointT q) {
	this->_p = p;
	this->_q = q;
}

float PlaneEdge::length() {
	return pcl::geometry::distance(this->_p, this->_q);
}