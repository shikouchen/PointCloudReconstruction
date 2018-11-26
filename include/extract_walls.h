//
// Created by czh on 11/4/18.
//

#ifndef TEST_PCL_EXTRACTWALL_H
#define TEST_PCL_EXTRACTWALL_H

#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h> //RANSAC‚Ì‚½‚ß
#include <math.h>
#include <pcl/filters/project_inliers.h> //•½–Ê‚É“Š‰e‚·‚é‚½‚ß
#include <pcl/filters/passthrough.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h> //ƒ_ƒEƒ“ƒTƒ“ƒvƒŠƒ“ƒO‚Ì‚½‚ß
#include <pcl/common/geometry.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include "Plane.h"
using namespace std;

typedef pcl::PointXYZRGBNormal PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void generateLinePointCloud(PointT pt1, PointT pt2, int pointPitch, int color, PointCloudT::Ptr output);

// mark: for debug reason
void extendSmallPlaneToBigPlane(Plane& sourceP, Plane& targetP, int color, int pointPitch, PointCloudT::Ptr output);
bool onSegment(PointT p, PointT q, PointT r);
float orientation(PointT p, PointT q, PointT r);
bool isIntersect(PointT p1, PointT q1, PointT p2, PointT q2);
#endif //TEST_PCL_EXTRACTWALL_H
