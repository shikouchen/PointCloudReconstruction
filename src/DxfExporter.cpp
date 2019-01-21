//
// Created by czh on 1/21/19.
//
#include <iostream>
#include <vector>
#include <DxfExporter.h>
#include <fstream>
#include <sstream>

using namespace std;


KKRecons::DxfExporter::DxfExporter(string fileName) {
    this->fileName = fileName;
}

void KKRecons::DxfExporter::insert(DxfFace face) {
    this->planes.push_back(face);
}

void KKRecons::DxfExporter::exportDXF(string path) {
    std::ofstream output(path + this->fileName + ".dxf");
    output << this->getHeaderPart();
    int numCompleteFace = 0;
    for (int i = 0; i < this->planes.size(); ++i) {
        DxfFace& face = this->planes[i];
        if (face.vacants.size() == 0){
            numCompleteFace++;
            output << "0\n";
            output << "VERTEX\n";
            output << "100\n";
            output << "AcDbEntity\n";
            output << "100\n";
            output << "AcDbVertex\n";
            output << "100\n";
            output << "AcDbPolyFaceMeshVertex\n";
            output << "10\n";
            output << face.a.x << "\n";
            output << "20\n";
            output << face.a.y << "\n";
            output << "30\n";
            output << face.a.z << "\n";
            output << "70\n";
            output << "192\n";
            output << "0\n";
            output << "VERTEX\n";
            output << "100\n";
            output << "AcDbEntity\n";
            output << "100\n";
            output << "AcDbVertex\n";
            output << "100\n";
            output << "AcDbPolyFaceMeshVertex\n";
            output << "10\n";
            output << face.b.x << "\n";
            output << "20\n";
            output << face.b.y << "\n";
            output << "30\n";
            output << face.b.z << "\n";
            output << "70\n";
            output << "192\n";
            output << "0\n";
            output << "VERTEX\n";
            output << "100\n";
            output << "AcDbEntity\n";
            output << "100\n";
            output << "AcDbVertex\n";
            output << "100\n";
            output << "AcDbPolyFaceMeshVertex\n";
            output << "10\n";
            output << face.c.x << "\n";
            output << "20\n";
            output << face.c.y << "\n";
            output << "30\n";
            output << face.c.z << "\n";
            output << "70\n";
            output << "192\n";
            output << "0\n";
            output << "VERTEX\n";
            output << "100\n";
            output << "AcDbEntity\n";
            output << "100\n";
            output << "AcDbVertex\n";
            output << "100\n";
            output << "AcDbPolyFaceMeshVertex\n";
            output << "10\n";
            output << face.d.x << "\n";
            output << "20\n";
            output << face.d.y << "\n";
            output << "30\n";
            output << face.d.z << "\n";
            output << "70\n";
            output << "192\n";
        }
    }

    for (int j = 0; j < numCompleteFace; ++j) {
        output << "0" << "\n";
        output << "VERTEX" << "\n";
        output << "5" << "\n";
        output << "AcDbEntity" << "\n";
        output << "100" << "\n";
        output << "AcDbFaceRecord" << "\n";
        output << "70" << "\n"; output << "128" << "\n";
        output << "71" << "\n"; output << 4*j+1 << "\n";
        output << "72" << "\n"; output << 4*j+2 << "\n";
        output << "73" << "\n"; output << 4*j+3 << "\n";
        output << "74" << "\n"; output << 4*j+4 << "\n";
    }
    output << this->getEndPart();
    output.close();
}

string KKRecons::DxfExporter::getHeaderPart() {
    string header;
    header.append("0\n");
    header.append("SECTION\n");
    header.append("2\n");
    header.append("HEADER\n");
    header.append("9\n");
    header.append("$INSUNITS\n");
    header.append("70\n");
    header.append("6\n");
    header.append("0\n");
    header.append("SECTION\n");
    header.append("2\n");
    header.append("ENTITIES\n");
    header.append("0\n");
    header.append("POLYLINE\n");
    header.append("100\n");
    header.append("AcDbEntity\n");
    header.append("100\n");
    header.append("AcDbPolyFaceMesh\n");
    header.append("66\n");
    header.append("1\n");
    header.append("10\n");
    header.append("0.0\n");
    header.append("20\n");
    header.append("0.0\n");
    header.append("30\n");
    header.append("0.0\n");
    header.append("70\n");
    header.append("64\n");
    header.append("71\n");
    header.append("8\n");
    header.append("72\n");
    header.append("8\n");
}

string KKRecons::DxfExporter::getEndPart() {
    string endStr;
    endStr.append("0\n");
    endStr.append("SEQEND\n");
    endStr.append("0\n");
    endStr.append("ENDSEC\n");
    endStr.append("0\n");
    endStr.append("EOF\n");
}

string KKRecons::DxfExporter::getPlanesPart() {

}