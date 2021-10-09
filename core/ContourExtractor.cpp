#include "ContourExtractor.h"

void ContourExtractor::detect_contour(const cv::Mat &img, std::vector<Contour> &contours)
{
    if (img.channels() == 3)
    {
        cv::Mat img_gray;
        cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
        // https://stackoverflow.com/questions/45323590/do-contours-returned-by-cvfindcontours-have-a-consistent-orientation
        cv::findContours(img_gray, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    }
    else
    {
        cv::findContours(img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    }

    // find the maximum area contour
    if (contours.size() > 1)
        std::sort(contours.begin(), contours.end(), [](Contour a, Contour b) {
            return cv::contourArea(a) > cv::contourArea(b);
        });
}