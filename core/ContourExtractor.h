#pragma once
#include "common.h"
#include <opencv2/opencv.hpp>
class ContourExtractor
{
public:
    typedef std::vector<cv::Point> Contour;
    static void detect_contour(const cv::Mat &img, std::vector<Contour> &contours);
};