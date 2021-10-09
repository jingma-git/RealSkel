#include "skeletor.h"
#include <opencv2/opencv.hpp>

void Skeletor::get_branches()
{
    get_img_points(inter_points, end_points);
    cout << __FILE__ << " " << __LINE__ << " get_branches successfully!" << endl;
}

void Skeletor::get_img_points(std::vector<cv::Point> &inter_points,
                              std::vector<cv::Point> &end_points)
{
    int H = H_;
    int W = W_;
    unsigned char *im = im_;

    for (int i = 1; i < H - 1; i++)
    {
        for (int j = 1; j < W - 1; j++)
        {
            int sum = 0;
            for (int r = -1; r <= 1; r++)
            {
                for (int c = -1; c <= 1; c++)
                {
                    int p = im[(i + r) * W + (j + c)];
                    sum += p;
                }
            }
            if ((int)im[i * W + j])
            {
                if (sum == 2)
                {
                    end_points.push_back(cv::Point(j, i));
                }
                else if (sum > 3)
                {
                    inter_points.push_back(cv::Point(j, i));
                }
                cout << i << ", " << j << " sum=" << sum << " pixel=" << (int)im[i * W + j] << endl;
            }
        }
    }

    cout << "end_points=" << end_points.size() << " inter_points=" << inter_points.size() << endl;
}