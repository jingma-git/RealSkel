#include "silhoutte_draw.h"

#include <opencv2/opencv.hpp>

void draw_silhoutte(const std::vector<Pnt3> &sil,
                    std::string fname,
                    int w, int h, int padding)
{
    cv::Mat img(h + padding * 2, w + padding * 2, CV_8UC3, cv::Scalar(255, 255, 255));

    int n = static_cast<int>(sil.size());
    std::vector<cv::Point> pts;
    pts.reserve(n);

    for (int i = 0; i < n; ++i)
    {
        pts.push_back(cv::Point(sil[i].x() + 20, sil[i].y() + 20));
        cv::circle(img, pts[i], 8, cv::Scalar(0, 0, 255), -1);
    }

    for (int i = 0; i < n; ++i)
    {
        cv::line(img, pts[i], pts[(i + 1) % n], cv::Scalar(0, 0, 0), 2);
        cv::putText(img, std::to_string(i), (pts[i] + pts[(i + 1) % n]) * 0.5,
                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255));
    }

    cv::imwrite(fname, img);
}