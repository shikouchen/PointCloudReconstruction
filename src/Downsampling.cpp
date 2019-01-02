//
// Created by czh on 1/2/19.
//
#include <iostream>
#include <vector>
#include <cmath>
#include <regex.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <pcl/console/print.h>
#include <regex>

using namespace std;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
std::vector<std::string> split(const std::string& s, char delimiter)
{
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter))
    {
        tokens.push_back(token);
    }
    return tokens;
}

int main(int argc, char** argv) {

//    PointCloudT::Ptr all(new PointCloudT);
//    for (int i = 0; i <= 15; ++i) {
//        PointCloudT::Ptr tmp(new PointCloudT);
//        pcl::io::loadPLYFile(argv[1]+to_string(i)+".ply",*tmp);
//        for(auto &p:tmp->points) all->push_back(p);
//    }
//    pcl::io::savePLYFile("all.ply",*all);
//    return 0;

    if(argc != 5){
        cerr << "Input is not enough. Example: Downsampling [InputPath/*.txt] [leafSize] [interval] [outputName]" << endl;
        return 0;
    }
    PointCloudT::Ptr tmpPointcloud(new PointCloudT);
    PointCloudT::Ptr allPointcloud(new PointCloudT);
    float leafSize = stod(argv[2]) ;
    unsigned int interval = atoi(argv[3]);
    string outputName(argv[4]);
    cout << "==================================" << "\n";
    cout << "Leaf Size: " << leafSize << " | " << "Interval: " << interval << "\n";
    cout << "==================================" << "\n";
    int snaps = 0;
    unsigned int total = snaps*interval;
    while(1) {
        tmpPointcloud->resize(0);
        int times = 0;
        std::ifstream file(argv[1]);
        std::string str;
        unsigned int tmp = 0;
        while(++tmp <= total) std::getline(file, str);
        while (std::getline(file, str))
        {
            vector<string> tokens;
            tokens = split(str, ' ');
            if(tokens.size() != 7) continue;
            if(times++ == interval) break;
            PointT p;
            try {
                p.x = stof(tokens[0]); p.y = stof(tokens[1]); p.z = stof(tokens[2]);
                p.r = stof(tokens[4]); p.g = stof(tokens[5]); p.b = stof(tokens[6]);
                p.a = stof(tokens[3])/2;
                tmpPointcloud->push_back(p);
            }catch(exception& e) {
                for (auto t:tokens) cout << t << " ";
                cout << "\n";
            }
        }

        cout << "snap:" << snaps << "  before downsampling " << tmpPointcloud->size();
        pcl::VoxelGrid<PointT> sor;
        sor.setInputCloud(tmpPointcloud);

        sor.setLeafSize(leafSize, leafSize, leafSize); //5mm
        sor.filter(*tmpPointcloud);

        cout << " after downsampling " << tmpPointcloud->size() << endl;
        if (tmpPointcloud->size()!=0) {
            pcl::io::savePLYFile("Downsampled_"+outputName+'_'+to_string(snaps)+".ply",*tmpPointcloud);
        }
        for(auto &p:tmpPointcloud->points) allPointcloud->push_back(p);
        if (!std::getline(file, str)) break;
        file.close();
        snaps++;
        total += interval;
    }
    pcl::io::savePLYFile("Downsampled_"+outputName+"_all"+".ply",*allPointcloud);
};