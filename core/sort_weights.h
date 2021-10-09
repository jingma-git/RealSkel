#pragma once

#include <igl/EPS.h>
#include <igl/sort.h>

#include <Eigen/Dense>
// Sort skinning weights. We want that W contains up to k weights, first being
// made up of *non-zero* original weights in descending order and then extra
// weights in descending order
// Templates:
//   T  should be a eigen matrix primitive type like int or double
// Inputs:
//   OW  #V by #OW original weights
//   EW  #V by #EW extra weights (may be empty)
//   k  maximum number of weights per handle (may be inf)
// Outputs:
//   W  #V by min(#OW+#EW,k)  sorted weights
//   WI  #V by min(#OW+#EW,k)  sorted weight indices such that if OEW = [OW EW]
//     then W(:,i) = OEW(:,WI(i))
template <typename T>
inline void sort_weights(
    const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &OW,
    const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &EW,
    const int k,
    const double epsilon,
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &W,
    Eigen::MatrixXi &WI)
{

    assert(k > 0);
    // number of domain positions
    int n = OW.rows();
    assert(n == EW.rows());
    // number of original weights
    int no = OW.cols();
    // number of extra weights
    int ne = EW.cols();
    // Find maximum in EW
    T max_ew = 0;
    if (ne > 0)
    {
        max_ew = EW.maxCoeff();
    }

    // Add max_ew to all significant original weights
    // OEW = [OW+(OW>0)*max(EW(:)) EW];
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> OEW(n, no + ne);
    OEW.block(0, 0, n, no) = (OW.array() > epsilon).select(OW.array() + max_ew, OW.array());
    // Fill in EW
    if (ne > 0)
    {
        OEW.block(0, no, n, ne) = EW;
    }

    igl::sort(OEW, 2, false, W, WI);

    // Set all original weights in W their value without max_ew
    W =
        (WI.array() < no).select((W.array() > epsilon).select(W.array() - max_ew, W.array()), W.array());
    // only keep k weights
    if (k < W.cols())
    {
        W = W.leftCols(k).eval();
        WI = WI.leftCols(k).eval();
    }
}