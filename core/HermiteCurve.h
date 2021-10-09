#pragma once
#include "common.h"

class HermiteCurve
{
public:
    HermiteCurve(const Pnt3 &start_pt, const Pnt3 &end_pt,
                 const Vec3 &start_tan, const Pnt3 &end_tan,
                 int numsamples)
        : p_start(start_pt),
          p_end(end_pt),
          t_start(start_tan),
          t_end(end_tan),
          num_samples(numsamples),
          curv_len(0)
    {
        double step = 1.0 / num_samples;
        Pnt3 prev_p;
        for (int i = 0; i <= num_samples; ++i)
        {
            double t = i * step;
            Pnt3 p = (2 * t * t * t - 3 * t * t + 1) * p_start +
                     (t * t * t - 2 * t * t + t) * t_start +
                     (-2 * t * t * t + 3 * t * t) * p_end +
                     (t * t * t - t * t) * t_end;

            if (i == 0)
            {
                prev_p = p;
            }
            else
            {
                curv_len += (p - prev_p).norm();
                prev_p = p;
            }
        }
    }

    double curv_length() { return curv_len; }

    double sample_pts(double delta, std::vector<Pnt3> &pts, std::vector<Vec3> &normals)
    {
        int n_samples = std::ceil(curv_len / delta);
        for (int i = 0; i <= n_samples; ++i)
        {
            double t = i * step;
            Pnt3 p = (2 * t * t * t - 3 * t * t + 1) * p_start +
                     (t * t * t - 2 * t * t + t) * t_start +
                     (-2 * t * t * t + 3 * t * t) * p_end +
                     (t * t * t - t * t) * t_end;
            pts.push_back(p);

            // ToDO: check this, may implement wrong
            Vec3 n = (6 * t * t - 6 * t) * p_start + (-6 * t * t + 6 * t) * p_end + (3 * t * t - 4 * t + 1) * t_start;
            normals.push_back(n.normalized());
        }
    }

private:
    Pnt3 p_start, p_end;
    Vec3 t_start, t_end; // tangential
    int num_samples;
    double curv_len;
};
