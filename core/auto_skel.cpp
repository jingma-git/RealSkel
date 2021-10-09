#include "auto_skel.h"
#include "XFunctions.h"
#include <opencv2/opencv.hpp>
using namespace std;

static int W = 0;
static int H = 0;
static int xmin = 0;
static int ymin = 0;
static int padding_x = 0;
static int padding_y = 0;
static cv::Scalar colors[9] = {
    cv::Scalar(255, 255, 255), //white
    cv::Scalar(0, 0, 255),     //r
    cv::Scalar(0, 255, 0),     //g
    cv::Scalar(255, 0, 0),     //b
    cv::Scalar(0, 255, 255),   //yellow
    cv::Scalar(255, 255, 0),   // cyan
    cv::Scalar(255, 0, 255),   //pink
    cv::Scalar(125, 0, 255),   //purple
    cv::Scalar(0, 125, 255)    //orange
};

void cal_medial_axis_from_polygon(const std::vector<Pnt3> &pts, std::vector<std::vector<Pnt3>> &medial_axis)
{
    //----------------------------Convert Polygon to Binary Image----------------------
    BBox2 bbox = GetBBox(pts);
    xmin = bbox.xmin + 0.5;
    ymin = bbox.ymin + 0.5;
    int xmax = bbox.xmax + 0.5;
    int ymax = bbox.ymax + 0.5;
    padding_x = (xmax - xmin) / 4.0; // 0.25
    padding_y = (ymax - ymin) / 4.0;

    W = (xmax - xmin) + 2 * padding_x;
    H = (ymax - ymin) + 2 * padding_y;

    vector<cv::Point> cont;
    for (const Pnt3 &p : pts)
    {
        int x = int(p.x() + 0.5);
        int y = int(p.y() + 0.5);
        cont.emplace_back(x - xmin + padding_x, y - ymin + padding_y);
        // cout << cont.back() << endl;
    }
    cv::Mat img = cv::Mat::zeros(H, W, CV_8UC1);
    std::vector<std::vector<cv::Point>> contours;
    contours.push_back(cont);
    cv::drawContours(img, contours, -1, cv::Scalar(1), -1);
    // cv::imwrite("output/skel_contour.jpg", img * 255);

    //----------------------------Thinning and Divide-Conqure to Cal MedialAxis Points----------------------
    skeleton_tracer_t *T = new skeleton_tracer_t();
    T->W = W; // width of image
    T->H = H; // height of image
    // allocate the input image
    T->im = img.data;
    T->thinning_zs(); // perform raster thinning

    // run the algorithm
    skeleton_tracer_t::polyline_t *p = (skeleton_tracer_t::polyline_t *)T->trace_skeleton(0, 0, T->W, T->H, 0);

    // print out points in every polyline
    skeleton_tracer_t::polyline_t *it = p; //iterator
    while (it)
    {
        skeleton_tracer_t::point_t *jt = it->head;
        std::vector<Pnt3> key_points;
        while (jt)
        {
            // printf("%d,%d ", jt->x, jt->y);
            key_points.emplace_back(jt->x + xmin - padding_x, jt->y + ymin - padding_y, 0); //From Image Space to World Space
            jt = jt->next;
        }
        // printf("\n");
        medial_axis.push_back(key_points);
        it = it->next;
    }

    // clean up
    T->destroy_polylines(p);
    T->destroy_rects();

    delete T;
    cout << __FILE__ << " " << __LINE__ << " cal_medial_axis_from_polygon successfully!" << endl;
}

void cal_skel_from_polygon(const std::vector<Pnt3> &pts, std::vector<std::vector<Pnt3>> &skel, float tol)
{
    std::vector<std::vector<Pnt3>> medial_axis;
    cal_medial_axis_from_polygon(pts, medial_axis);
    std::vector<std::vector<Pnt3>> skel_tmp;
    skel_tmp.resize(medial_axis.size());
    skel.resize(medial_axis.size());

    for (int i = 0; i < medial_axis.size(); i++)
    {
        Float len = CalLineLength(medial_axis[i]);
        SimplifyPolyLineDP(medial_axis[i], skel_tmp[i], tol); //ToDO: use different Simplification Strategy to Different Parts
        // merge very close two points into one
        int num_bones = skel_tmp[i].size() - 1;
        Float avg_len = num_bones <= 1 ? len : (len / num_bones);
        Float thresh = avg_len / 2;
        std::vector<Pnt3> &pts = skel_tmp[i];
        // cout << __FILE__ << " " << __LINE__ << " skel" << i << " avg_len=" << avg_len << " tol=" << tol << " thresh=" << thresh << endl;
        int j = 0;
        for (; j < pts.size() - 1;)
        {
            double dis = (pts[j] - pts[j + 1]).norm();
            if (dis < thresh)
            {
                // cout << "point" << j << " dis=" << dis << endl;
                skel[i].push_back((pts[j] + pts[j + 1]) / 2.0);
                j += 2;
            }
            else
            {
                skel[i].push_back(pts[j]);
                j += 1;
            }
        }
        if (j == pts.size() - 1)
        {
            skel[i].push_back(pts[j]);
        }
        // cout << "-------------------------------------\n";
    }

    // jt->x + xmin - padding_x
    // cv::Mat skel_img = cv::Mat::zeros(W, H, CV_8UC3);
    // cout << __FILE__ << " " << __LINE__ << " W=" << W << " H=" << H << endl;
    // for (int i = 0; i < skel.size(); i++)
    // {
    //     for (int j = 0; j < skel[i].size(); j++)
    //     {
    //         Pnt3 &p = skel[i][j];
    //         cv::Point cp(p.x() - xmin + padding_x, (p.y() - ymin + padding_y));
    //         // cout << "line" << i << " p" << j << ": " << p.transpose() << " cp=" << cp.x << ", " << cp.y << endl;
    //         cv::circle(skel_img, cp, 1, colors[i], -1);
    //         cv::putText(skel_img, to_string(j), cv::Point(cp.x, cp.y), cv::FONT_HERSHEY_SIMPLEX,
    //                     0.5, colors[i]);
    //     }
    // }
    // cv::imwrite("output/skel.jpg", skel_img);
}