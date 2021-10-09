#pragma once
#include <vector>
#include <string>
namespace egl
{
    template <class Point>
    inline void writeLine(const std::string fname, const std::vector<Point> &line_pts)
    {
        FILE *fptr = fopen(fname.c_str(), "w");
        if (fptr == nullptr)
        {
            printf("cannot write to file %s\n", fname.c_str());
            return;
        }
        fprintf(fptr, "%d\n", (int)line_pts.size());
        for (const Point &p : line_pts)
        {
            if (p.size() == 2)
            {
                fprintf(fptr, "%g %g\n", p.x(), p.y());
            }
            else if (p.size() == 3)
            {
                fprintf(fptr, "%g %g %g\n", p.x(), p.y(), p.z());
            }
        }
        fclose(fptr);
    }
};