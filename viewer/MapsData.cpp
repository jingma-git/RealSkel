#include "MapsData.h"
#include <opencv2/opencv.hpp>

using namespace std;

bool MapsData::visualizeMask(string fileName, const MatI &mask)
{
    int height = mask.size();
    int width = mask[0].size();
    cv::Mat mat = cv::Mat::zeros(height, width, CV_8UC1);
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            if (mask[i][j])
                mat.at<uchar>(i, j) = 255;
        }
    }

    return cv::imwrite(fileName, mat);
}

bool MapsData::visualizeDepth(string fileName, const MatD &depth)
{
    int height = depth.size();
    int width = depth[0].size();
    cv::Mat mat = cv::Mat::zeros(height, width, CV_8UC1);
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            double d = depth[i][j];
            mat.at<uchar>(i, j) = (d * 0.5 + 0.5) * 255;
        }
    }

    return cv::imwrite(fileName, mat);
}