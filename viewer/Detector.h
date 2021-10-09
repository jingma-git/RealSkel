#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

class Detector
{
    typedef std::vector<cv::Point> Contour;

public:
    static void detect_contour(const cv::Mat &img, std::vector<Contour> &contours);
    static void draw_contour(cv::Mat &out_img,
                             const cv::Mat &img,
                             const Contour &contour,
                             cv::Scalar color);
};