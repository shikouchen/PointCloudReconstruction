//
// Created by czh on 1/21/19.
//

#ifndef RECONSTRUCTION_DXFEXPORTER_H
#define RECONSTRUCTION_DXFEXPORTER_H

#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <Plane.h>
using namespace std;
typedef pcl::PointXYZ Point;

struct DxfFace{
    DxfFace(Point a, Point b, Point c, Point d){
        this->a = a;
        this->b = b;
        this->c = c;
        this->d = d;
    };
    DxfFace(Plane plane){
        Point _a,_b,_c,_d;
        _a.x = plane.leftDown().x; _a.y = plane.leftDown().y; _a.z = plane.leftDown().z;
        _b.x = plane.leftUp().x;   _b.y = plane.leftUp().y;   _b.z = plane.leftUp().z;
        _c.x = plane.rightUp().x;  _c.y = plane.rightUp().y;  _c.z = plane.rightUp().z;
        _d.x = plane.rightDown().x;_d.y = plane.rightDown().y; d.z = plane.rightDown().z;
        this->a = _a;
        this->b = _b;
        this->c = _c;
        this->d = _d;
    }
    vector<DxfFace> vacants;
    Point a,b,c,d;
};

namespace KKRecons{
    class DxfExporter{
    public:
        DxfExporter(string fileName);
        void insert(DxfFace plane);
        void exportDXF(string path);
    private:
        string fileName;
        vector<DxfFace> planes;
        string getHeaderPart();
        string getEndPart();
        string getPlanesPart();
    };
}

#endif //RECONSTRUCTION_DXFEXPORTER_H
