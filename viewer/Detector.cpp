#include "Detector.h"
#include <core/common.h>
using namespace std;

void Detector::detect_contour(const cv::Mat &img, std::vector<Contour> &contours)
{
    cv::Mat img_gray, canny_img, dilate_img, erode_img;
    cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
    cv::Canny(img_gray, canny_img, 100, 200);

    uint kernel_arr[5][5] = {{0, 0, 1, 0, 0},
                             {0, 1, 1, 1, 0},
                             {1, 1, 1, 1, 1},
                             {0, 1, 1, 1, 0},
                             {0, 0, 1, 0, 0}};
    cv::Mat kernel(5, 5, CV_8U);
    for (int i = 0; i < 5; i++)
    {
        for (int j = 0; j < 5; j++)
        {
            kernel.at<uchar>(i, j) = kernel_arr[i][j];
        }
    }

    cv::dilate(canny_img, dilate_img, kernel);
    cv::erode(dilate_img, erode_img, kernel);

    cv::findContours(erode_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // find the maximum area contour
    std::sort(contours.begin(), contours.end(), [](Contour a, Contour b)
              { return cv::contourArea(a) > cv::contourArea(b); });

    M_DEBUG << cv::contourArea(contours.front()) << ", " << cv::contourArea(contours.back()) << endl;
}

void Detector::draw_contour(cv::Mat &out_img,
                            const cv::Mat &img, const Contour &contour, cv::Scalar color)
{
    out_img = img.clone();
    vector<Contour> contours;
    contours.push_back(contour);

    cv::drawContours(out_img, contours, 0, color, 2);
}