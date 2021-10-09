#pragma once
#include <igl/ray_mesh_intersect.h>

#include <Eigen/Eigen>
#include <iostream>

using namespace std;

namespace EGL
{
    // sx, sy: screen point
    // width, height: screen width and height
    // mvp: projectionMatrix* viewMatrix * modelMatrix
    // x, y, z: world coordinate
    template <typename MVPScalar, typename Scalar>
    inline void unproject(const int sx, const int sy,
                          const int width, const int height,
                          MVPScalar *mvp,
                          Scalar &x, Scalar &y, Scalar &z)
    {
        using namespace Eigen;
        // screen-space to NDC-spac
        float xproj = (sx - (float)width / 2.f) / ((float)width / 2.f);
        float yproj = (-sy + (float)height / 2.f) / ((float)height / 2.f);

        Matrix<MVPScalar, 4, 4> mvpMat = Map<Matrix<MVPScalar, 4, 4>>(mvp);
        // acquire projected coordinate for world_center (get projected z (NDC_z) for world_center)
        Matrix<MVPScalar, 4, 1> world_center = mvpMat * Matrix<MVPScalar, 4, 1>(0, 0, 0, 1);
        Matrix<MVPScalar, 4, 1> sp(xproj, yproj, world_center.z(), 1); // screen point
        Matrix<MVPScalar, 4, 1> wp = mvpMat.inverse() * sp;

        x = wp.x();
        y = wp.y();
        z = wp.z();
        // cout << __FILE__ << " " << __LINE__ << " sp=" << sp.transpose() << " wp=" << wp.transpose() << endl;
        // cout << "----mvp inverse\n"
        //      << mvpMat.inverse() << endl;
    }

    // sx, sy: screen point
    // width, height: screen width and height
    // mvp: projectionMatrix* viewMatrix * modelMatrix
    // x, y, z: world coordinate
    template <typename MVPScalar, typename Scalar>
    inline void unproject(const int sx, const int sy, const Scalar sz,
                          const int width, const int height,
                          MVPScalar *mvp,
                          Scalar &x, Scalar &y, Scalar &z)
    {
        using namespace Eigen;
        // screen-space to NDC-spac
        float xproj = (sx - (float)width / 2.f) / ((float)width / 2.f);
        float yproj = (-sy + (float)height / 2.f) / ((float)height / 2.f);

        Matrix<MVPScalar, 4, 4> mvpMat = Map<Matrix<MVPScalar, 4, 4>>(mvp);
        Matrix<MVPScalar, 4, 1> sp(xproj, yproj, sz, 1); // screen point
        Matrix<MVPScalar, 4, 1> wp = mvpMat.inverse() * sp;

        x = wp.x();
        y = wp.y();
        z = wp.z();
        // cout << __FILE__ << " " << __LINE__ << " sp=" << sp.transpose() << " wp=" << wp.transpose() << endl;
        // cout << "----mvp inverse\n"
        //      << mvpMat.inverse() << endl;
    }

    // width, height: screen width and height
    // mvp: projectionMatrix* viewMatrix * modelMatrix
    // x, y, z: world coordinate
    // sx, sy: screen point
    template <typename Scalar, typename MVPScalar>
    inline void project(const Scalar x, const Scalar y, const Scalar z,
                        const int width, const int height,
                        MVPScalar *mvp,
                        Scalar &sx, Scalar &sy, Scalar &sz)
    {
        using namespace Eigen;
        Matrix<Scalar, 4, 1> wp4(x, y, z, 1.0);
        Matrix<MVPScalar, 4, 4> mvpMat = Map<Matrix<MVPScalar, 4, 4>>(mvp);
        Matrix<Scalar, 4, 1> v_ndc = mvpMat.template cast<Scalar>() * wp4;

        sx = (v_ndc.x() + 1.0) * 0.5 * width;
        sy = (v_ndc.y() + 1.0) * 0.5 * -height + height;
        sz = v_ndc.z();
    }

    template <typename MatScalar, typename Scalar>
    inline void view_dir(MatScalar *view, Scalar &dir_x, Scalar &dir_y, Scalar &dir_z)
    {
        using namespace Eigen;
        Matrix<MatScalar, 4, 4> viewMatrix = Map<Matrix<MatScalar, 4, 4>>(view);
        dir_x = -viewMatrix(2, 0);
        dir_y = -viewMatrix(2, 1);
        dir_z = -viewMatrix(2, 2);
    }

    template <typename MVPScalar,
              typename Derivedbc>
    inline bool unproject_onto_mesh(
        const int sx, const int sy, //screen_x, screen_y
        const int width, const int height,
        MVPScalar *mvp,
        const std::function<
            bool(
                const Eigen::Vector3f &,
                const Eigen::Vector3f &,
                igl::Hit &)> &shoot_ray,
        int &fid,
        Eigen::PlainObjectBase<Derivedbc> &bc)
    {
        using namespace std;
        using namespace Eigen;

        Eigen::Vector3f s, d;
        EGL::unproject(sx, sy, 0.f,
                       width, height, mvp,
                       s[0], s[1], s[2]);
        EGL::unproject(sx, sy, 1.f,
                       width, height, mvp,
                       d[0], d[1], d[2]);

        Eigen::Vector3f dir = d - s;

        igl::Hit hit;
        if (!shoot_ray(s, dir, hit))
        {
            return false;
        }
        bc.resize(3, 1);
        bc << 1.0 - hit.u - hit.v, hit.u, hit.v;
        fid = hit.id;
        return true;
    }

    template <typename MVPScalar,
              typename DerivedV,
              typename DerivedF,
              typename Derivedbc>
    inline bool unproject_onto_mesh(
        const int sx, const int sy,
        const int width, const int height,
        MVPScalar *mvp,
        const Eigen::MatrixBase<DerivedV> &V,
        const Eigen::MatrixBase<DerivedF> &F,
        int &fid,
        Eigen::PlainObjectBase<Derivedbc> &bc) //bc: barycentric
    {
        using namespace std;
        using namespace Eigen;
        const auto &shoot_ray = [&V, &F](
                                    const Eigen::Vector3f &s,
                                    const Eigen::Vector3f &dir,
                                    igl::Hit &hit) -> bool {
            std::vector<igl::Hit> hits;
            if (!igl::ray_mesh_intersect(s, dir, V, F, hits))
            {
                return false;
            }
            hit = hits[0];
            return true;
        };
        return unproject_onto_mesh(sx, sy, width, height, mvp, shoot_ray, fid, bc);
    }

}; // namespace EGL
