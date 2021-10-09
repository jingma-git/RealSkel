#pragma once

#include "common.h"
#include <opencv2/opencv.hpp>

// image skeletonization algo based on thinning
class Skeletor
{
public:
    Skeletor() : im_(nullptr), W_(-1), H_(-1) {}
    Skeletor(unsigned char *im, int W, int H) : im_(im), W_(W), H_(H) {}

    inline void print()
    {
        unsigned char *im = im_;
        int H = H_;
        int W = W_;

        for (int i = 0; i < H; i++)
        {
            for (int j = 0; j < W; j++)
            {
                int idx = i * W + j;
                // printf("%c", im[idx] == 2 ? '.' : (im[idx] == 0 ? ' ' : '@'));
                if (im[idx] == 0)
                {
                    printf("%c", ' ');
                }
                else
                {
                    printf("%d", im[idx]);
                }
            }
            printf("\n");
        }
    }

    void get_branches();

    // Binary image thinning (skeletonization) in-place.
    // Implements Zhang-Suen algorithm.
    // http://agcggs680.pbworks.com/f/Zhan-Suen_algorithm.pdf
    bool thinning_zs_iteration(unsigned char *im, int W, int H, int iter)
    {
        bool diff = false;
        for (int i = 1; i < H - 1; i++)
        {
            for (int j = 1; j < W - 1; j++)
            {
                int p2 = im[(i - 1) * W + j] & 1;
                int p3 = im[(i - 1) * W + j + 1] & 1;
                int p4 = im[(i)*W + j + 1] & 1;
                int p5 = im[(i + 1) * W + j + 1] & 1;
                int p6 = im[(i + 1) * W + j] & 1;
                int p7 = im[(i + 1) * W + j - 1] & 1;
                int p8 = im[(i)*W + j - 1] & 1;
                int p9 = im[(i - 1) * W + j - 1] & 1;

                int A = (p2 == 0 && p3 == 1) + (p3 == 0 && p4 == 1) +
                        (p4 == 0 && p5 == 1) + (p5 == 0 && p6 == 1) +
                        (p6 == 0 && p7 == 1) + (p7 == 0 && p8 == 1) +
                        (p8 == 0 && p9 == 1) + (p9 == 0 && p2 == 1);
                int B = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
                int m1 = iter == 0 ? (p2 * p4 * p6) : (p2 * p4 * p8);
                int m2 = iter == 0 ? (p4 * p6 * p8) : (p2 * p6 * p8);
                if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
                    im[i * W + j] |= 2;
            }
        }
        for (int i = 0; i < H * W; i++)
        {
            int marker = im[i] >> 1;
            int old = im[i] & 1;
            im[i] = old & (!marker);
            if ((!diff) && (im[i] != old))
            {
                diff = true;
            }
        }
        return diff;
    };

    void thinning_zs()
    {
        bool diff = true;
        do
        {
            diff &= thinning_zs_iteration(im_, W_, H_, 0);
            diff &= thinning_zs_iteration(im_, W_, H_, 1);
        } while (diff);
    }

    void get_img_points(std::vector<cv::Point> &inter_points,
                        std::vector<cv::Point> &end_points);

    void fill_H()
    {
        int pad = 2;
        int hl = 6;  // horizontal middle line width
        int ht = 4;  // horizontal middle line thickness
        int leg = 6; // H's four legs

        W_ = (hl + 2) * 2 + hl + 2 * pad;
        H_ = leg * 2 + ht + 2 * pad;
        int W = W_;
        int H = H_;
        im_ = new unsigned char[W * H]; // allocate memory and call the constructor if it is a class
                                        // uchar *im = (uchar *)malloc(sizeof(uchar) * W * H);
                                        // for primitive type, new==malloc
        unsigned char *im = im_;
        memset(im, 0, sizeof(im));
        for (int i = 0; i < H; i++)
        {
            if (i < pad || i >= (H - pad))
                continue;

            for (int j = 0; j < W; j++)
            {
                int idx = i * W + j;
                if (i < (leg + pad) || i >= (leg + ht + pad))
                {
                    if (j >= pad && j < (hl + pad + 2))
                        im[idx] = 1;
                    if (j >= (hl + pad + 2 + hl) && j < (W - pad))
                        im[idx] = 1;
                }
                else
                {
                    if (j >= pad && j < W - pad)
                    {
                        im[idx] = 1;
                    }
                }
            }
        }
    }

private:
    unsigned char *im_; // the image
    int W_;             // width
    int H_;             // height

    std::vector<cv::Point> inter_points, end_points;
};