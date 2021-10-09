#pragma once
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <Eigen/Eigen>
namespace egl
{
    template <class Point>
    inline void readLine(const std::string fname, std::vector<Point> &line_pts)
    {
        using namespace std;
        std::ifstream is(fname);
        if (!is.is_open())
        {
            std::cout << "cannot open " << fname << endl;
            return;
        }
        string line;
        getline(is, line);
        int pts_size;
        sscanf(line.c_str(), "%d", &pts_size);
        for (int i = 0; i < pts_size; i++)
        {
            getline(is, line);
            double x, y, z;
            sscanf(line.c_str(), "%lf %lf %lf", &x, &y, &z);
            line_pts.push_back(Point(x, y));
        }
        is.close();
    }

    template <>
    inline void readLine<Eigen::Vector3d>(const std::string fname, std::vector<Eigen::Vector3d> &line_pts)
    {
        using namespace std;
        std::ifstream is(fname);
        if (!is.is_open())
        {
            std::cout << "cannot open " << fname << endl;
            return;
        }
        string line;
        getline(is, line);
        int pts_size;
        sscanf(line.c_str(), "%d", &pts_size);
        for (int i = 0; i < pts_size; i++)
        {
            getline(is, line);
            double x, y, z;
            sscanf(line.c_str(), "%lf %lf %lf", &x, &y, &z);
            line_pts.push_back(Eigen::Vector3d(x, y, z));
        }
        is.close();
    }

};