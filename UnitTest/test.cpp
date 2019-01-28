//
// Created by czh on 1/21/19.
//

#include <gtest/gtest.h>
#include <DxfExporter.h>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
//#include <pcl/point_types.h>
using namespace std;
TEST(DXF,Init){
    KKRecons::DxfExporter exporter("testDxfFile");
    Point point1,point2,point3,point4;
    point1.x = (float)(rand()%1000) / 10; point1.y = (float)(rand()%1000) / 10; point1.z = (float)(rand()%1000) / 10;
    point2.x = (float)(rand()%1000) / 10; point2.y = (float)(rand()%1000) / 10; point2.z = (float)(rand()%1000) / 10;
    point3.x = (float)(rand()%1000) / 10; point3.y = (float)(rand()%1000) / 10; point3.z = (float)(rand()%1000) / 10;
    point4.x = (float)(rand()%1000) / 10; point4.y = (float)(rand()%1000) / 10; point4.z = (float)(rand()%1000) / 10;
    DxfFace face(point1,point2,point3,point4);
    ASSERT_EQ(face.a.x, point1.x); ASSERT_EQ(face.a.y, point1.y); ASSERT_EQ(face.a.z, point1.z);
    ASSERT_EQ(face.b.x, point2.x); ASSERT_EQ(face.b.y, point2.y); ASSERT_EQ(face.b.z, point2.z);
}

TEST(DXF, Insert) {
    KKRecons::DxfExporter exporter("testDxfFile");
    ASSERT_EQ(exporter.size(), 0);
    Point point1,point2,point3,point4;
    point1.x = (float)(rand()%1000) / 10; point1.y = (float)(rand()%1000) / 10; point1.z = (float)(rand()%1000) / 10;
    point2.x = (float)(rand()%1000) / 10; point2.y = (float)(rand()%1000) / 10; point2.z = (float)(rand()%1000) / 10;
    point3.x = (float)(rand()%1000) / 10; point3.y = (float)(rand()%1000) / 10; point3.z = (float)(rand()%1000) / 10;
    point4.x = (float)(rand()%1000) / 10; point4.y = (float)(rand()%1000) / 10; point4.z = (float)(rand()%1000) / 10;
    DxfFace face(point1,point2,point3,point4);
    exporter.insert(face);
    ASSERT_EQ(exporter.size(), 1);
}

TEST(DXF, Export) {
    KKRecons::DxfExporter exporter("testDxfFile");
    Point a,b,c,d;
    a.x = -10; a.y = -10; a.z = -10;
    b.x = -10; b.y = 10;  b.z = -10;
    c.x = 10;  c.y = 10;  c.z = -10;
    d.x = 10;  d.y = -10; d.z = -10;
    DxfFace face0(a,b,c,d);

    a.x = -10; a.y = -10; a.z = 0;
    b.x = -10; b.y = 10;  b.z = 0;
    c.x = 10;  c.y = 10;  c.z = 0;
    d.x = 10;  d.y = -10; d.z = 0;
    DxfFace face(a,b,c,d);

    a.x = -5; a.y = -5; a.z = 0;
    b.x = -5; b.y = 5;  b.z = 0;
    c.x = 5;  c.y = 5;  c.z = 0;
    d.x = 5;  d.y = -5; d.z = 0;
    DxfFace subFace(a,b,c,d);
    face.vacants.push_back(subFace);

    a.x = -10; a.y = -10; a.z = 10;
    b.x = -10; b.y = 10;  b.z = 10;
    c.x = 10;  c.y = 10;  c.z = 10;
    d.x = 10;  d.y = -10; d.z = 10;
    DxfFace face2(a,b,c,d);

    exporter.insert(face0);
    exporter.insert(face);
    exporter.insert(face2);
    ASSERT_EQ(exporter.size(),3);
    ASSERT_NO_THROW(exporter.exportDXF("./"););
    FILE *file = fopen("./testDxfFile.dxf", "r");
    ASSERT_NE(file, nullptr);
}

TEST(YAML, BASIC) {
    YAML::Node config = YAML::LoadFile("config.yaml");
    YAML::Node RANSAC = config["RANSAC"];
    YAML::Node ParaRANSAC = RANSAC["value1"];
    cout << RANSAC["value1"] << endl;


    //YAML::Node config2;
}