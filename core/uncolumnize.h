#pragma once

#include <Eigen/Dense>
// "UnColumnize" a stack of block matrices. If A = [A1,A2,A3,...,Ak] with each A*
// an m by n block then this produces the column vector whose entries are
// A(i,b*n+j) =  B(j*m*k+i*k+b)
// or if A = [A1;A2;...;Ak] then
// A(i+b*m,j) = B(j*m*k+i*k+b)
//
// Templates:
//   T  should be a eigen matrix primitive type like int or double
// Inputs:
//   B  m*n*k eigen vector of type T values,
//   k  number of blocks
//   dim  dimension in which blocks are stacked
// Output
//   A  m*k by n (dim: 1) or m by n*k (dim: 2) eigen Matrix of type T values
//
// See also: transpose_blocks
template <typename T, const int C>
inline void uncolumnize(
    const Eigen::Matrix<T, Eigen::Dynamic, C> &B,
    const size_t m,
    const size_t n,
    const size_t dim,
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &A);

// Implementation
#include <cassert>

template <typename T, const int C>
inline void uncolumnize(
    const Eigen::Matrix<T, Eigen::Dynamic, C> &B,
    const size_t m,
    const size_t n,
    const size_t dim,
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &A)
{
    // Eigen matrices must be 2d so dim must be only 1 or 2
    assert(dim == 1 || dim == 2);
    assert(B.cols() == 1);
    // number of blocks
    int k = B.rows() / m / n;
    assert((int)(k * m * n) == (int)B.rows());

    if (dim == 1)
    {
        A.resize(m * k, n);
    }
    else // dim == 2
    {
        A.resize(m, n * k);
    }

    for (int b = 0; b < (int)k; b++)
    {
        for (int i = 0; i < (int)m; i++)
        {
            for (int j = 0; j < (int)n; j++)
            {
                if (dim == 1)
                {
                    A(i + b * m, j) = B(j * m * k + i * k + b);
                }
                else
                {
                    A(i, b * n + j) = B(j * m * k + i * k + b);
                }
            }
        }
    }
}
