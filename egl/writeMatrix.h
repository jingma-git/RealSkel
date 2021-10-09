#pragma once

#include <string>
#include <cstdio>

namespace egl
{
    // Write a matrix using ascii dmat file type
    //
    // Template:
    //   Mat  matrix type that supports .rows(), .cols(), operator(i,j)
    // Inputs:
    //   file_name  path to .dmat file
    //   W  eigen matrix containing to-be-written coefficients
    //   ascii  write ascii file {true}
    // Returns true on success, false on error
    //
    template <typename DerivedW>
    inline bool writeMatrix(
        const std::string file_name,
        const Eigen::MatrixBase<DerivedW> &W,
        const bool ascii = true)
    {
        FILE *fp = fopen(file_name.c_str(), "wb");
        if (fp == NULL)
        {
            fprintf(stderr, "IOError: writeMatrix() could not open %s...", file_name.c_str());
            return false;
        }
        if (ascii)
        {
            // first line contains number of rows and number of columns
            fprintf(fp, "%d %d\n", (int)W.cols(), (int)W.rows());

            for (int i = 0; i < W.rows(); i++)
            {
                for (int j = 0; j < W.cols(); j++)
                {
                    fprintf(fp, "%0.17lg ", (double)W(i, j));
                }
                fprintf(fp, "\n");
            }
        }
        else
        {
            // write header for ascii
            fprintf(fp, "0 0\n");
            // first line contains number of rows and number of columns
            fprintf(fp, "%d %d\n", (int)W.cols(), (int)W.rows());
            // reader assumes the binary part is double precision
            Eigen::MatrixXd Wd = W.template cast<double>();
            fwrite(Wd.data(), sizeof(double), Wd.size(), fp);
        }
        fclose(fp);
        return true;
    }
}