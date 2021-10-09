#include <set>
#include <igl/cotmatrix_entries.h>
#include <igl/cotmatrix.h>
#include <igl/massmatrix.h>
#include <igl/doublearea.h>
#include <igl/cat.h>
#include <igl/barycenter.h>
#include <igl/writeOBJ.h>

#include "TMesh.h"
#include "XFunctions.h"
#include "ArapDeform.h"
#include "cylinder_skel.h"
#include "cdt_skel.h"
#define DEBUG

TMesh::TMesh()
{
    m_vNormals = add_property_map<Vd, Vec3>("v:normal").first;
    m_fNormals = add_property_map<Fd, Vec3>("f:normal").first;

    //------------------- Teddy inflate --------------------------------------------------
    m_fTypes = add_property_map<Fd, FaceType>("f:type").first;
    m_eVisit = add_property_map<Ed, bool>("e:visit", false).first; // ToDO: remove
    m_bIsFaceTypeSet = false;

    m_V.resize(0, 3);
    m_F.resize(0, 3);
}

void TMesh::CreateHalfedgeMesh(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F)
{
    this->clear();

    for (int i = 0; i < V.rows(); i++)
    {
        this->add_vertex(Pnt3(V(i, 0), V(i, 1), V(i, 2)));
    }
    for (int i = 0; i < F.rows(); i++)
    {
        int fid = this->add_face(F(i, 1), F(i, 2), F(i, 0));
        // Fd fd(fid);
        // Hd hd = halfedge(fd);
        // M_DEBUG << i << " " << source(hd) << " " << target(hd) << " " << target(next(hd)) << endl;
    }
}

TMesh &TMesh::operator=(const TMesh &rhs)
{
    SMesh::operator=(rhs);

    m_vNormals = property_map<Vd, Vec3>("v:normal").first;
    m_fNormals = property_map<Fd, Vec3>("f:normal").first;

    //------------------- Teddy inflate --------------------------------------------------
    m_fTypes = property_map<Fd, FaceType>("f:type").first;
    m_eVisit = property_map<Ed, bool>("e:visit").first; // ToDO: remove
    m_bIsFaceTypeSet = rhs.m_bIsFaceTypeSet;

    //------------------- Laplace inflate --------------------------------------------------
    m_V = rhs.m_V;
    m_F = rhs.m_F;
    m_fixedV = rhs.m_fixedV;
    //------------------- ArapLayer Deform --------------------------------------------------
    m_layerV = rhs.m_layerV;
    m_lConstrain = rhs.m_lConstrain;
    return *this;
}

void TMesh::clear()
{
    SMesh::clear();

    m_vNormals = add_property_map<Vd, Vec3>("v:normal").first;
    m_fNormals = add_property_map<Fd, Vec3>("f:normal").first;

    //------------------- Teddy inflate --------------------------------------------------
    m_fTypes = add_property_map<Fd, FaceType>("f:type").first;
    m_eVisit = add_property_map<Ed, bool>("e:visit", false).first; // ToDO: remove
    m_bIsFaceTypeSet = false;

    //------------------- Laplace inflate --------------------------------------------------
    m_V.resize(0, 3); // Notice: If there is a bug, that is where it propabably appears!!!
    m_F.resize(0, 3);
    m_fixedV.clear();
    //------------------- ArapLayer Deform --------------------------------------------------
    m_layerV.clear();
    m_lConstrain.clear();
}

Pnt3 TMesh::barycent(Fd fd)
{
    const Hd &hd = halfedge(fd);
    const Hd &nxtHd = next(hd);
    Pnt3 p0 = point(source(hd));
    Pnt3 p1 = point(target(hd));
    Pnt3 p2 = point(target(nxtHd));
    return (p0 + p1 + p2) / 3.0;
}

Pnt3 TMesh::barycent(const vector<Fd> &fds)
{
    Pnt3 cent;
    cent.setZero();
    double sum = 0.0;
    for (const Fd &fd : fds)
    {
        double area = Get3DFArea(fd);
        cent += area * barycent(fd);
        sum += area;
    }
    return cent / sum;
}

FaceType TMesh::GetFType(Fd fd)
{
    if (!m_bIsFaceTypeSet)
        LabelXFaceType();
    return m_fTypes[fd];
}

FaceType TMesh::GetFType(Fd fd) const
{
    if (m_bIsFaceTypeSet)
    {
        return m_fTypes[fd];
    }
    else
    {
        int iBoundEdgeNum = 0;
        Hd hd = halfedge(fd);
        for (Hd hd : halfedges_around_face(hd))
        {
            Ed ed = edge(hd);
            if (is_border(ed))
            {
                iBoundEdgeNum++;
            }
        }
        if (iBoundEdgeNum == 2)
        {
            return TERMINAL;
        }
        else if (iBoundEdgeNum == 1)
        {
            return SLEEVE;
        }
        else
        {
            return JUNCTION;
        }
    }
}

Float TMesh::edge_len(Vd v0, Vd v1)
{
    return (point(v0) - point(v1)).norm();
}

Float TMesh::edge_len(Hd h)
{
    return edge_len(source(h), target(h));
}

Float TMesh::edge_len(Ed e)
{
    return edge_len(halfedge(e));
}

Float TMesh::max_edge_len()
{
    Float fMaxEdgeLen = -1.0e6, fEdgeLen;
    for (Ed ed : edges())
    {
        fEdgeLen = edge_len(ed);
        if (fEdgeLen > fMaxEdgeLen)
        {
            fMaxEdgeLen = fEdgeLen;
        }
    }
    return fMaxEdgeLen;
}

Float TMesh::avg_edge_len()
{
    Float fLenSum = 0.0;
    for (Ed ed : edges())
    {
        fLenSum += edge_len(ed);
    }
    return fLenSum / (Float)num_edges();
}

Vec3 TMesh::face_normal(Fd fd)
{
    Hd hd = halfedge(fd);
    Pnt3 p0 = point(source(hd));
    Pnt3 p1 = point(target(hd));
    Pnt3 p2 = point(target(next(hd)));

    Vec3 norm = (p1 - p0).cross(p2 - p0);
    return norm.normalized();
}

Vec3 TMesh::face_normal(Fd fd) const
{
    Hd hd = halfedge(fd);
    Pnt3 p0 = point(source(hd));
    Pnt3 p1 = point(target(hd));
    Pnt3 p2 = point(target(next(hd)));

    Vec3 norm = (p1 - p0).cross(p2 - p0);
    return norm.normalized();
}

bool TMesh::is_border_edge(Hd hd)
{
    return is_border(hd) || is_border(opposite(hd));
}

BBox3 TMesh::bbox()
{
    double xmin = std::numeric_limits<double>::max();
    double ymin = std::numeric_limits<double>::max();
    double zmin = std::numeric_limits<double>::max();
    double xmax = std::numeric_limits<double>::min();
    double ymax = std::numeric_limits<double>::min();
    double zmax = std::numeric_limits<double>::min();

    for (const Vd &vd : vertices())
    {
        Pnt3 &p = point(vd);
        if (p.x() < xmin)
        {
            xmin = p.x();
        }
        if (p.y() < ymin)
        {
            ymin = p.y();
        }
        if (p.z() < zmin)
        {
            zmin = p.z();
        }
        if (p.x() > xmax)
        {
            xmax = p.x();
        }
        if (p.y() > ymax)
        {
            ymax = p.y();
        }
        if (p.z() > zmax)
        {
            zmax = p.z();
        }
    }
    return BBox3(xmin, ymin, zmin, xmax, ymax, zmax);
}

void TMesh::SetZDepth(double depth)
{
    for (const Vd &vd : vertices())
    {
        Pnt3 &p = point(vd);
        Pnt3 np(p.x(), p.y(), p.z() + depth);
        p = np;
    }
    m_V.col(2).array() += depth;
}

int TMesh::num_adj(Vd v)
{
    Hd h = halfedge(v);
    int count = 0;
    for (Vd vj : vertices_around_target(h))
    {
        count++;
    }
    return count;
}

Eigen::RowVectorXd TMesh::barycenter()
{
    using namespace Eigen;
    MatrixXd &V = GetVMat();
    return V.colwise().sum() / V.rows();
}

void TMesh::UpdateVMat()
{
    m_V.resize(number_of_vertices(), 3);
    for (Vd vd : vertices())
    {
        int i = vd.idx();
        const Pnt3 &p = point(vd);
        m_V(i, 0) = p.x();
        m_V(i, 1) = p.y();
        m_V(i, 2) = p.z();
    }
}

Eigen::MatrixXd &TMesh::GetVMat()
{
    if (m_V.rows() != number_of_vertices())
    {
        UpdateVMat();
        return m_V;
    }
    else
    {
        return m_V;
    }
}

Eigen::MatrixXi &TMesh::GetFMat()
{
    if (m_F.rows() != number_of_faces())
    {
        m_F.resize(number_of_faces(), 3);
        for (Fd fd : faces())
        {
            int i = fd.idx();
            Hd hd = halfedge(fd);
            Vd v0 = source(hd);
            Vd v1 = target(hd);
            Vd v2 = target(next(hd));
            m_F(i, 0) = v0.idx();
            m_F(i, 1) = v1.idx();
            m_F(i, 2) = v2.idx();
        }
        return m_F;
    }
    else
    {
        return m_F;
    }
}

Eigen::SparseMatrix<double> &TMesh::GetLaplaceMat()
{
    // ToDO: change to halfedge impl
    Eigen::MatrixXd &V = GetVMat();
    Eigen::MatrixXi &F = GetFMat();
    if (m_L.rows() <= 0)
    {
        igl::cotmatrix(V, F, m_L);
        return m_L;
    }

    else
        return m_L;
}

Eigen::SparseMatrix<double> &TMesh::GetMassMat()
{
    // ToDO: change to halfedge impl
    Eigen::MatrixXd &V = GetVMat();
    Eigen::MatrixXi &F = GetFMat();
    if (m_M.rows() <= 0)
    {
        igl::massmatrix(V, F, igl::MASSMATRIX_TYPE_BARYCENTRIC, m_M);
        return m_M;
    }

    else
        return m_M;
}

void TMesh::SetPoints(const Eigen::MatrixXd &V)
{
    assert(V.rows() == number_of_vertices());
    for (Vd vd : vertices())
    {
        int i = vd.idx();
        Pnt3 &p = point(vd);
        p = Pnt3(V(i, 0), V(i, 1), V(i, 2));
    }
}

bool TMesh::IsFixed(int vidx)
{
    return m_fixedV[vidx];
}

int TMesh::GetLayerId(int vidx)
{
    try
    {
        int layerid = m_layerV.at(vidx);
        return layerid;
    }
    catch (std::out_of_range)
    {
        return -1;
    }
}

LConstrainType TMesh::GetLayerConstrainType(int vidx)
{
    try
    {
        LConstrainType type = m_lConstrain.at(vidx);
        return type;
    }
    catch (std::out_of_range)
    {
        return LConstrainType::InvalidConstrain;
    }
}

/**
 * @brief 
 * 
 * @param verts: vertex position to be marked as fixed
 */
void TMesh::MarkFixedVerts(const vector<Pnt2> &verts)
{
    // ToDO: to be improved, find boundary vertex first
    int num_fixed = 0;
    for (const Vd &vd : vertices())
    {
        const Pnt3 &p = point(vd);
        bool isFixed = false;
        for (const Pnt2 &v : verts)
        {
            if ((Pnt2(p.x(), p.y()) - v).norm() < 1e-6)
            {
                isFixed = true;
                num_fixed++;
                break;
            }
        }
        m_fixedV[vd.idx()] = isFixed;
    }
    cout << "mark fixed verts, verts=" << verts.size() << " num_fixed: " << num_fixed << endl;
    assert(num_fixed == verts.size());
}

int TMesh::GetNumFixed()
{
    int num_fixed = 0;
    for (const auto &kv : m_fixedV)
    {
        if (kv.second)
            num_fixed++;
    }
    return num_fixed;
}

Float TMesh::Get3DFArea(Fd fd)
{
    Hd hd = halfedge(fd);
    Pnt3 v0 = point(source(hd));
    Pnt3 v1 = point(target(hd));
    Pnt3 v2 = point(target(next(hd)));

    return 0.25 * (((v1 - v0).cross(v2 - v0)).norm() + ((v1 - v2).cross(v0 - v2)).norm());
}

Float TMesh::Get3DFArea(int f_idx)
{
    return Get3DFArea(Fd(f_idx));
}

Float TMesh::Get3DArea()
{
    Float area = 0.0;
    for (Fd fd : faces())
    {
        area += Get3DFArea(fd);
    }
    return area;
}

void TMesh::UpdatePos(const vector<Pnt3> &newV)
{
    m_V.resize(number_of_vertices(), 3);
    for (Vd vd : vertices())
    {
        int i = vd.idx();
        point(vd) = newV[i];
        m_V(i, 0) = newV[i].x();
        m_V(i, 1) = newV[i].y();
        m_V(i, 2) = newV[i].z();
    }
}

Pnt3 TMesh::GetFPos(const int fid, const Vec3 &bc)
{
    using namespace Eigen;
    MatrixXd &V = GetVMat();
    MatrixXi &F = GetFMat();

    Pnt3 v0 = V.row(F(fid, 0)).transpose();
    Pnt3 v1 = V.row(F(fid, 1)).transpose();
    Pnt3 v2 = V.row(F(fid, 2)).transpose();

    // barycentric coordinates so that
    //   pos = V.row(F(id,0))*(1-u-v)+V.row(F(id,1))*u+V.row(F(id,2))*v;
    return bc(0) * v0 + bc(1) * v1 + bc(2) * v2;
}

void TMesh::Translate(const Vec3 &offset)
{
    for (const Vd &vd : vertices())
    {
        point(vd) += offset;
    }
    UpdateVMat();
}

void TMesh::Rotate(const Eigen::Matrix3d &rot)
{
    for (const Vd &vd : vertices())
    {
        Pnt3 &p = point(vd);
        p = rot * p;
    }
    UpdateVMat();
}

void TMesh::Scale(double scl)
{
    for (const Vd &vd : vertices())
    {
        Pnt3 &p = point(vd);
        p = scl * p;
    }
    UpdateVMat();
}

void TMesh::ComputeFNormals()
{
    for (const Fd &fd : faces())
    {
        m_fNormals[fd] = face_normal(fd);
    }
}

void TMesh::ComputeVNormals()
{
    for (const Fd &fd : faces())
    {
        m_fNormals[fd] = face_normal(fd);
    }
    for (const Vd &vi : vertices())
    {
        Hd hi = halfedge(vi);
        Float area_sum = 0;
        Vec3 norm(0, 0, 0);

        for (const Hd &hj : halfedges_around_target(hi))
        {
            if (is_border(hj))
            {
                continue;
            }
            Fd fj = face(hj);
            Float area = Get3DFArea(fj);
            norm += area * m_fNormals[fj];
            area_sum += area;
        }
        if (area_sum <= 0)
        {
            norm = Vec3(0, 0, 0);
        }
        else
        {
            norm /= area_sum;
        }

        m_vNormals[vi] = norm;
    }
}

const Vec3 &TMesh::GetFNormal(Fd fd)
{
    return m_fNormals[fd];
}

const Vec3 &TMesh::GetVNormal(Vd vd)
{
    return m_vNormals[vd];
}

void TMesh::ReverseFaceOrientation()
{
    TMesh tmp;
    for (Vd vd : vertices())
    {
        tmp.add_vertex(point(vd));
    }

    for (Fd fd : faces())
    {
        Hd h = halfedge(fd);
        Vd vd0 = source(h);
        Vd vd1 = target(h);
        Vd vd2 = target(next(h));
        tmp.add_face(vd2, vd1, vd0);
    }
    clear();
    *this = tmp;
}

Fd TMesh::FindLocatedFaceIn2D(const Pnt3 &p)
{
    for (Fd fd : faces())
    {
        Hd hd = halfedge(fd);
        const Pnt3 &p0 = point(source(hd));
        const Pnt3 &p1 = point(target(hd));
        const Pnt3 &p2 = point(target(next(hd)));
        if (IsPointInTriangle(p, p0, p1, p2))
        {
            return fd;
        }
    }
    return null_face();
}

Vd TMesh::FindClosestV2D(const Pnt3 &p)
{
    Fd fd = FindLocatedFaceIn2D(p);
    if (!fd.is_valid())
        return null_vertex();
    Vd closest;
    Float min_dis = std::numeric_limits<Float>::max();
    for (const Vd &vd : vertices_around_face(halfedge(fd)))
    {
        Float dis = (point(vd) - p).norm();
        if (dis < min_dis)
        {
            min_dis = dis;
            closest = vd;
        }
    }
    return closest;
}

Vd TMesh::FindClosestV(const Pnt3 &p)
{
    Vd closest;
    Float min_dis = std::numeric_limits<Float>::max();
    for (const Vd &vd : vertices())
    {
        Float dis = (point(vd) - p).norm();
        if (dis < min_dis)
        {
            min_dis = dis;
            closest = vd;
        }
    }
    std::cout << __FILE__ << " " << __LINE__ << " closest " << closest << " pt=" << point(closest) << ", min_dis=" << min_dis << std::endl;
    return closest;
}

void TMesh::FindBoundary(std::vector<int> &bnd_inds)
{
    Hd stBndHd;
    for (Hd hd : halfedges())
    {
        if (is_border(hd))
        {
            stBndHd = hd;
            break;
        }
    }
    bnd_inds.push_back(source(stBndHd).idx());

    Hd hd = next(stBndHd);
    while (hd != stBndHd)
    {
        bnd_inds.push_back(source(hd).idx());
        hd = next(hd);
    }
    assert(bnd_inds.front() != bnd_inds.back());
}

void TMesh::FindBoundary(std::vector<Pnt3> &bnd_pts)
{
    Hd stBndHd;
    for (Hd hd : halfedges())
    {
        if (is_border(hd))
        {
            stBndHd = hd;
            break;
        }
    }
    bnd_pts.push_back(point(source(stBndHd)));

    Hd hd = next(stBndHd);
    while (hd != stBndHd)
    {
        bnd_pts.push_back(point(source(hd)));
        hd = next(hd);
    }
}

void TMesh::FindBoundary(std::vector<Vd> &bnd_inds)
{
    Hd stBndHd;
    for (Hd hd : halfedges())
    {
        if (is_border(hd))
        {
            stBndHd = hd;
            break;
        }
    }
    bnd_inds.push_back(source(stBndHd));

    Hd hd = next(stBndHd);
    while (hd != stBndHd)
    {
        bnd_inds.push_back(source(hd));
        hd = next(hd);
    }
    assert(bnd_inds.front() != bnd_inds.back());
}

void TMesh::FindSketchBoundary(std::vector<Vd> &sketch_verts)
{
    assert(m_fixedV.size() > 0);
    for (const auto &kv : m_fixedV)
    {
        if (kv.second)
            sketch_verts.push_back(Vd(kv.first));
    }
}

bool TMesh::RefineLocalMesh(vector<int> &flag, bool bIsSmooth)
{
    if (flag.size() != number_of_vertices())
    {
        return false;
    }
    int oriVNum = number_of_vertices();
    int oriFNum = number_of_faces();
    int oriENum = number_of_edges();
    int vnumnew = 0;
    int enumnew = 0;
    int fnumnew = 0; // ToDO
    TMesh new_tm;

    vector<bool> flag_f; // flag for faces that should be subdivided
    flag_f.resize(oriFNum, false);
    for (Fd fd : faces())
    {
        Hd hd = halfedge(fd);
        size_t v0 = source(hd).idx();
        size_t v1 = target(hd).idx();
        size_t v2 = target(next(hd)).idx();

        if ((flag[v0] == 1 && flag[v1] == 1 && flag[v2] == 1) ||
            (flag[v0] == 1 && flag[v1] == 1 && flag[v2] != 1) ||
            (flag[v0] == 1 && flag[v1] != 1 && flag[v2] == 1) ||
            (flag[v0] != 1 && flag[v1] == 1 && flag[v2] == 1))
        {
            flag_f[fd.idx()] = true;
            fnumnew++;
        }
    }
    vector<bool> flag_e;
    vector<int> vVID; // IDs of newly added points corresponding to each edge
    flag_e.resize(oriENum, false);
    vVID.resize(oriENum);

    for (Vd vd : vertices())
    {
        new_tm.add_vertex(point(vd));
    }

    for (int i = 0; i < oriENum; i++)
    {
        Ed ed(i);
        Vd vd0 = vertex(ed, 0);
        Vd vd1 = vertex(ed, 1);
        if (flag[vd0.idx()] == 1 && flag[vd1.idx()] == 1)
        {
            flag_e[i] = true;
            Pnt3 mid_pt = (point(vd0) + point(vd1)) / 2.0;
            vVID[i] = new_tm.add_vertex(mid_pt).idx(); // newly added vertex idx
            enumnew++;
        }
    }
    assert(new_tm.number_of_vertices() == (oriVNum + enumnew));

    //									  np0
    //                                    /\
	//                                   /  \
	//	                            nv0 /----\  nv2
    //                                 / \  / \
	//                                /   \/   \                                       
	//                            np1 ---------- np2
    int np0 = -1, np1 = -1, np2 = -1, nv0 = -1, nv1 = -1, nv2 = -1, ne = -1;
    int ne0 = -1, ne1 = -1, ne2 = -1;
    int fid = oriFNum; //Id of newly added face
    for (Fd fd : faces())
    {
        int icase = -1;

        Hd f_hd = halfedge(fd);
        size_t ne0 = edge(f_hd);
        size_t ne1 = edge(next(f_hd));
        size_t ne2 = edge(next(next(f_hd)));

        if (flag_e[ne0] && flag_e[ne1] && flag_e[ne2])
        {
            icase = 3;
        }
        else if ((flag_e[ne0] && flag_e[ne1] && !flag_e[ne2]) || (flag_e[ne0] && !flag_e[ne1] && flag_e[ne2]) || (!flag_e[ne0] && flag_e[ne1] && flag_e[ne2]))
        {
            icase = 2;
        }
        else if ((flag_e[ne0] && !flag_e[ne1] && !flag_e[ne2]) || (!flag_e[ne0] && flag_e[ne1] && !flag_e[ne2]) || (!flag_e[ne0] && !flag_e[ne1] && flag_e[ne2]))
        {
            icase = 1;
        }

        if (icase != -1)
        {
            np0 = source(f_hd).idx();
            np1 = target(f_hd).idx();
            np2 = target(next(f_hd)).idx();
        }

        if (icase == 3) // face will be divived into four triangles
        {
            nv0 = vVID[ne0];
            nv1 = vVID[ne1];
            nv2 = vVID[ne2];

            new_tm.add_face(np0, nv0, nv2);
            new_tm.add_face(np1, nv1, nv0);
            new_tm.add_face(nv0, nv1, nv2);
            new_tm.add_face(np2, nv2, nv1);
        }
        else if (icase == 1) //divide face into two
        {
            int num_f_before = new_tm.num_faces();
            if (flag_e[ne0])
            {
                nv0 = vVID[ne0];
                new_tm.add_face(np0, nv0, np2);
                new_tm.add_face(np1, np2, nv0);
            }
            else if (flag_e[ne1])
            {
                nv1 = vVID[ne1];
                new_tm.add_face(np0, np1, nv1);
                new_tm.add_face(np2, np0, nv1);
            }
            else if (flag_e[ne2])
            {
                nv2 = vVID[ne2];
                new_tm.add_face(np0, np1, nv2);
                new_tm.add_face(np1, np2, nv2);
            }
            assert((new_tm.num_faces() - num_f_before) == 2);
        }
        else if (icase == 2) //divide face into three
        {
            int num_f_before = new_tm.num_faces();
            if (flag_e[ne0] && flag_e[ne1])
            {
                nv0 = vVID[ne0];
                nv1 = vVID[ne1];

                new_tm.add_face(np0, nv0, nv1);
                new_tm.add_face(np1, nv1, nv0);
                new_tm.add_face(np1, nv1, nv0);
            }
            else if (flag_e[ne0] && flag_e[ne2])
            {
                nv0 = vVID[ne0];
                nv2 = vVID[ne2];

                new_tm.add_face(np0, nv0, nv2);
                new_tm.add_face(np1, nv2, nv0);
                new_tm.add_face(np2, nv2, np1);
            }
            else if (flag_e[ne1] && flag_e[ne2])
            {
                nv1 = vVID[ne1];
                nv2 = vVID[ne2];

                new_tm.add_face(np0, nv0, nv2);
                new_tm.add_face(np1, nv1, nv2);
                new_tm.add_face(np2, nv2, nv1);
            }
            assert((new_tm.num_faces() - num_f_before) == 3);
        }
    }

    *this = new_tm;

    flag.clear();
    flag_e.clear();
    flag_f.clear();
    vVID.clear();

    if (bIsSmooth)
    {
        //ToDO
        assert(false);
    }
    return true;
}

bool TMesh::RefineLocalMesh()
{
    vector<int> flag_v;
    flag_v.resize(number_of_vertices(), 1);
    return RefineLocalMesh(flag_v, false);
}

void TMesh::SmoothenMeshNaive(const vector<Vd> &vVtIdx, int nIter, Float fScl, Float len_avg, bool keep_boundary)
{
    if (vVtIdx.size() == 0 || vVtIdx.size() > number_of_vertices())
        return;
    if (len_avg < 0.01)
    {
        len_avg = avg_edge_len();
    }

    Float lmt = len_avg * 0.025;

    // for each vertex, ajdust its position to one-ring center
    for (int m = 0; m < nIter; m++)
    {
        for (Vd vi : vertices())
        {
            if (keep_boundary && point(vi).z() < 0.1)
                continue;

            Pnt3 &pi = point(vi);
            Hd hd = halfedge(vi);
            Pnt3 v(0, 0, 0);
            int adj_num = 0;
            for (Vd vj : vertices_around_target(hd))
            {
                v = v + point(vj);
                adj_num++;
            }
            Vec3 dis = (v / (Float)adj_num - pi) / fScl;
            Float dt = dis.norm();
            if (dt > lmt)
            {
                dis = dis.normalized() * lmt;
            }
            pi += dis;
        }
    }
}

void TMesh::SmoothenMeshEvenFilter(const vector<Vd> &vVtIdx, int nIter, Float fScl, Float len_avg, bool keep_boundary)
{
    if (vVtIdx.size() == 0 || vVtIdx.size() > number_of_vertices())
        return;
    if (len_avg < 0.01)
    {
        len_avg = avg_edge_len();
    }

    Float lmt = len_avg * 0.025;

    vector<Vec3> normF;
    vector<Pnt3> baryF;
    vector<Float> faceArea;
    normF.resize(num_faces());
    baryF.resize(num_faces());
    faceArea.resize(num_faces());
    for (int m = 0; m < nIter; m++)
    {
        for (Fd fd : faces())
        {
            Hd hd = halfedge(fd);
            Vd v0 = source(hd);
            Vd v1 = target(hd);
            Vd v2 = target(next(hd));

            faceArea[fd.idx()] = Get3DFArea(fd);
            baryF[fd.idx()] = (point(v0) + point(v1) + point(v2)) / 3.0;
        }

        for (Fd fd : faces())
        {
            Hd hd = halfedge(fd);
            normF[fd.idx()] = Vec3(0, 0, 0);
            Float fAreaSum = 0;
            for (Fd adjFd : faces_around_face(hd))
            {
                normF[fd.idx()] += faceArea[adjFd.idx()] * face_normal(adjFd); //ToDo
                fAreaSum += faceArea[adjFd.idx()];
            }
            normF[fd.idx()] /= fAreaSum;
            normF[fd.idx()] = normF[fd.idx()].normalized();
        }

        for (Vd vi : vVtIdx)
        {
            if (keep_boundary && point(vi).z() < 0.1)
                continue;

            Hd hi = halfedge(vi);
            Pnt3 &pi = point(vi);
            Vec3 fNorm(0, 0, 0);
            Float fAreaSum = 0;
            for (Fd fj : faces_around_target(hi))
            {
                Vec3 tagent = (baryF[fj.idx()] - pi);
                Vec3 w_norm = (tagent.dot(normF[fj.idx()])) * normF[fj.idx()]; //ToDO: validate this
                fNorm += faceArea[fj.idx()] * w_norm;
                fAreaSum += faceArea[fj.idx()];
            }
            fNorm /= fAreaSum;

            Vec3 dis = fNorm * fScl;
            Float dt = dis.norm();
            if (dt > lmt)
                dis = dis.normalized() * lmt;
            pi += dis;
        }
    }
}

void TMesh::GetChordalAxisLines(vector<C3DLine> &lines)
{
    // ToDO: need to be tested
    if (!m_bIsFaceTypeSet)
    {
        LabelXFaceType();
    }
    lines.clear();
    ResetEVisit();

    vector<Fd> vJFaceID;
    GetJunctionFaces(vJFaceID);
    vector<C3DLine> rawLines;
    cout << __FILE__ << " " << __LINE__ << " numJunctionFaces=" << vJFaceID.size() << endl;
    if (vJFaceID.size() == 0) // no junction face, there should be only two terminal faces
    {
        vector<Fd> vTFaceID;
        for (Fd fd : faces())
        {
            if (m_fTypes[fd] == TERMINAL)
            {
                vTFaceID.push_back(fd);
            }
        }
        assert(vTFaceID.size() == 2);
        SearchChordalAxisLinesBetweenTwoTFace(vTFaceID[0], vTFaceID[1], rawLines);
    }
    else
    {
        for (int i = 0; i < vJFaceID.size(); i++)
        {
            cout << i << "------------------search axis line in junction face: " << vJFaceID[i] << " lines=" << rawLines.size() << endl;
            SearchChordalAxisLinesFromAJFace(vJFaceID[i], rawLines);
        }
    }

    for (int l = 0; l < rawLines.size(); l++)
    {
        lines.push_back(rawLines[l]);
    }
}

void TMesh::GetChordalAxisLinesCylinder(vector<C3DLine> &lines)
{
    // ToDO: need to be tested
    if (!m_bIsFaceTypeSet)
    {
        LabelXFaceType();
    }
    lines.clear();
    ResetEVisit();

    vector<Fd> vJFaceID;
    GetJunctionFaces(vJFaceID);
    vector<C3DLine> rawLines;
    cout << __FILE__ << " " << __LINE__ << " numJunctionFaces=" << vJFaceID.size() << endl;
    if (vJFaceID.size() == 0) // no junction face, there should be only two terminal faces
    {
        vector<Fd> vTFaceID;
        for (Fd fd : faces())
        {
            if (m_fTypes[fd] == TERMINAL)
            {
                vTFaceID.push_back(fd);
            }
        }
        assert(vTFaceID.size() == 2);
        SearchChordalAxisLinesBetweenTwoTFace(vTFaceID[0], vTFaceID[1], lines);
        return;
    }
    else
    {
        for (int i = 0; i < vJFaceID.size(); i++)
        {
            cout << i << "------------------search axis line in junction face: " << vJFaceID[i] << " lines=" << rawLines.size() << endl;
            SearchChordalAxisLinesFromAJFace(vJFaceID[i], rawLines);
        }
    }
    // PruneChordalAxis(vJFaceID, rawLines, lines);
    C3DLine lineTmp;
    // CylinderSkel skeletor(*this, vJFaceID, rawLines);
    // skeletor.skeletonization(lines);
    CDTSkel skeletor(*this, vJFaceID, rawLines);
    skeletor.skeletonize(lines);
    // lines.push_back(lineTmp);

    // for (int l = 0; l < rawLines.size(); l++)
    // {
    //     lines.push_back(rawLines[l]);
    // }
}

void TMesh::LabelXFaceType()
{
    for (Fd fd : faces())
    {
        int iBoundEdgeNum = 0;
        Hd hd = halfedge(fd);
        for (Hd hd : halfedges_around_face(hd))
        {
            Ed ed = edge(hd);
            if (is_border(ed))
            {
                iBoundEdgeNum++;
            }
        }
        if (iBoundEdgeNum == 2)
        {
            m_fTypes[fd] = TERMINAL;
        }
        else if (iBoundEdgeNum == 1)
        {
            m_fTypes[fd] = SLEEVE;
        }
        else
        {
            m_fTypes[fd] = JUNCTION;
        }
        // cout << fd << ": " << (m_fTypes[fd] == TERMINAL ? "TERMINAL" : (m_fTypes[fd] == SLEEVE ? "SLEEVE" : "JUNCTION"))
        //      << endl;
    }
    m_bIsFaceTypeSet = true;
}

void TMesh::GetJunctionFaces(vector<Fd> &vJFaceId)
{
    vJFaceId.clear();
    if (!m_bIsFaceTypeSet)
    {
        LabelXFaceType();
    }
    for (Fd fd : faces())
    {
        if (m_fTypes[fd] == JUNCTION)
        {
            vJFaceId.push_back(fd);
        }
    }
}

void TMesh::SearchChordalAxisLinesFromAJFace(Fd iStFace, vector<C3DLine> &lines)
{
    // 1. get face vertices and center
    Hd stF_hd = halfedge(iStFace);
    Pnt3 v0 = point(source(stF_hd));
    Pnt3 v1 = point(target(stF_hd));
    Pnt3 v2 = point(target(next(stF_hd)));
    Pnt3 v_centSt = (v0 + v1 + v2) / 3.0;
    cout << __FILE__ << " " << __LINE__ << " starting... " << iStFace << " " << edge(stF_hd) << endl;
    // 2. start from curF's three adjacent faces, and continue founding ChordalLines
    int count = 0;
    for (Hd hd : halfedges_around_face(stF_hd))
    {
        cout << "visit " << edge(hd) << " " << face(hd) << endl;
        if (m_eVisit[edge(hd)])
            continue;

        C3DLine line;
        // add the line btw edge's mid-point and face's center to ChordalLine
        //     set face's center as ChordalLine's first control point
        //     set another three edge points as intersection points
        //     set intersection edge
        line.Setcpt(v_centSt, 0), line.Setcf(iStFace, 0);
        Pnt3 v_mid = (point(source(hd)) + point(target(hd))) / 2.0;
        line.Setipt(v_mid), line.Setif(iStFace);
        line.Setedges(edge(hd));
        line.SetHalfedges(hd);
        m_eVisit[edge(hd)] = true;
        Ed edge_d = edge(hd);
        cout << "start " << iStFace << " " << edge_d
             << " cpt0=" << line.Getcpt(0).transpose()
             << " ipt=" << line.m_ipt.back().transpose() << endl;
        // cout << "line.Getcf(0): " << line.Getcf(0) << " line.Getif(0): " << line.Getif(0) << " line.Getedge(0): " << line.Getedge(0)
        //      << " Hd_vertex: " << source(Hd) << ", " << target(Hd) << " edge_vertex: " << vertex(edge_d, 0) << ", " << vertex(edge_d, 1) << endl;

        int iCount = 0;
        bool bIsLastCptFound = false;
        Fd cur_fd = iStFace;
        Fd next_fd = null_face();
        Hd cur_Hd = hd;
        Hd next_Hd = null_halfedge();
        m_eVisit[edge(cur_Hd)] = true;
        do
        {
            // visit edge's next face, nextF
            next_fd = face(opposite(cur_Hd));
            // cout << "visit next face " << next_fd << endl;
            // if nextF is Terminal, set boundary-vertex as the ChordalLine's last ctrl-point
            // if nextF is Junction, set nextF's center-point as last ctrl-point
            // if is nextF is Sleeve, set nextE's mid-point as insect-point and add 'the edge' and face to ChordalLine
            // found non-boundary edge on nextF, set it as nextE
            if (m_fTypes[next_fd] == TERMINAL)
            {
                Hd cur_Hd_opp = opposite(cur_Hd);
                Vd end_vd = target(next(cur_Hd_opp));
                assert(is_border(edge(next(cur_Hd_opp))) && is_border(edge(next(next(cur_Hd_opp)))));
                Pnt3 end_p = point(end_vd);
                line.Setcpt(end_p, 1), line.Setcf(next_fd, 1);
                cout << "end Terminal: " << edge(cur_Hd_opp) << " " << end_vd << " " << next_fd << " cpt1=" << line.Getcpt(1).transpose() << endl;
                bIsLastCptFound = true;
                break;
            }
            else if (m_fTypes[next_fd] == JUNCTION)
            {
                Hd cur_Hd_opp = opposite(cur_Hd);
                assert(!is_border(edge(next(cur_Hd_opp))) && !is_border(edge(next(next(cur_Hd_opp)))));
                v0 = point(source(cur_Hd_opp));
                v1 = point(target(cur_Hd_opp));
                v2 = point(target(next(cur_Hd_opp)));
                Pnt3 v_centEnd = (v0 + v1 + v2) / 3.0;
                line.Setcpt(v_centEnd, 1), line.Setcf(next_fd, 1);
                cout << "end Junction: " << edge(cur_Hd_opp) << " " << next_fd << " cpt1=" << line.Getcpt(1).transpose() << endl;
                bIsLastCptFound = true;
                break;
            }
            else
            {
                assert(m_fTypes[next_fd] == SLEEVE);
                assert(edge(cur_Hd) == edge(opposite(cur_Hd)));
                next_Hd = next(opposite(cur_Hd));
                if (is_border(edge(next_Hd)))
                {
                    next_Hd = next(next_Hd);
                }

                assert(!is_border(edge(next_Hd)));
                assert(edge(cur_Hd) != edge(next_Hd));

                v_mid = (point(source(next_Hd)) + point(target(next_Hd))) / 2.0;
                line.Setipt(v_mid), line.Setif(face(next_Hd));
                line.Setedges(edge(next_Hd));
                line.SetHalfedges(next_Hd);
                // cout << "intersect " << edge(next_Hd) << " " << next_fd << " ipt=" << line.m_ipt.back().transpose() << endl;
                //cout << "before update: " << cur_Hd << "| current face=" << cur_fd << endl;
                m_eVisit[edge(next_Hd)] = true;
                cur_Hd = next_Hd;
                cur_fd = next_fd;
                //cout << "after update: " << cur_Hd << "| current face=" << cur_fd << endl;
            }

        } while (iCount < num_faces() && !bIsLastCptFound);

        lines.push_back(line);
        cout << __FILE__ << " " << __LINE__ << " " << count << ": " << line.Getcf(0) << "-" << line.Getcf(1) << " isects=" << line.GetiptCount() << endl;
        count++;
    }
    cout << __FILE__ << " " << __LINE__ << " ..........................end searching " << iStFace << " lines=" << lines.size() << endl;
}

void TMesh::ResetEVisit()
{
    for (Ed ed : edges())
    {
        m_eVisit[ed] = false;
    }
}

void TMesh::ReverseOrientation()
{
    TMesh tmp = *this;
    this->clear();
    for (Vd vd : tmp.vertices())
    {
        Vd vnew = this->add_vertex(tmp.point(vd));
    }
    for (Fd fd : tmp.faces())
    {
        Hd hd = tmp.halfedge(fd);
        Vd v0 = tmp.source(hd);
        Vd v1 = tmp.target(hd);
        Vd v2 = tmp.target(tmp.next(hd));
        this->add_face(v2.idx(), v1.idx(), v0.idx());
    }
}

void TMesh::SearchChordalAxisLinesBetweenTwoTFace(Fd iStF, Fd iEndF, vector<C3DLine> &lines)
{
    cout << __FILE__ << " " << __LINE__ << " SearchChordalAxisLinesBetweenTwoTFace " << iStF << "-" << iEndF << endl;
    ResetEVisit();
    Hd stF_hd = halfedge(iStF);
    Pnt3 v0, v1, v2, vMid;
    vector<Hd> boundE;
    Hd edgeSt;
    for (const Hd &hd : halfedges_around_face(stF_hd))
    {
        if (is_border(hd))
        {
            boundE.push_back(hd);
        }
        else
        {
            edgeSt = hd;
        }
    }
    if (boundE.size() != 2)
        return;

    Pnt3 stPnt;
    Pnt3 e0St = point(source(boundE[0]));
    Pnt3 e0End = point(target(boundE[0]));
    Pnt3 e1St = point(source(boundE[1]));
    Pnt3 e1End = point(target(boundE[1]));
    if ((e0St - e1St).norm() < 1e-3 || (e0St - e1End).norm() < 1e-3)
    {
        stPnt = e0St;
    }
    else
    {
        stPnt = e0End;
    }

    C3DLine line;
    line.Setcpt(stPnt, 0);
    line.Setcf(iStF, 0);
    v0 = point(source(edgeSt));
    v1 = point(target(edgeSt));
    vMid = (v0 + v1) / 2.0;
    line.Setipt(vMid);
    line.Setif(iStF);
    line.Setedges(edge(edgeSt));
    line.SetHalfedges(edgeSt);

    bool bIsLastCptFound = false;
    Fd cur_fd = iStF;
    Fd next_fd = null_face();
    Hd cur_Hd = stF_hd;
    Hd next_Hd = null_halfedge();
    m_eVisit[edge(cur_Hd)] = true;
    do
    {
        // visit edge's next face, nextF
        next_fd = face(opposite(cur_Hd));
        // cout << "visit next face " << next_fd << endl;
        // if nextF is Terminal, set boundary-vertex as the ChordalLine's last ctrl-point
        // if is nextF is Sleeve, set nextE's mid-point as insect-point and add 'the edge' and face to ChordalLine
        // found non-boundary edge on nextF, set it as nextE
        if (m_fTypes[next_fd] == TERMINAL)
        {
            Hd cur_Hd_opp = opposite(cur_Hd);
            Vd end_vd = target(next(cur_Hd_opp));
            assert(is_border(edge(next(cur_Hd_opp))) && is_border(edge(next(next(cur_Hd_opp)))));
            Pnt3 end_p = point(end_vd);
            line.Setcpt(end_p, 1), line.Setcf(next_fd, 1);
            cout << "end Terminal: " << edge(cur_Hd_opp) << " " << end_vd << " " << next_fd << " cpt1=" << line.Getcpt(1).transpose() << endl;
            bIsLastCptFound = true;
            break;
        }
        else if (m_fTypes[next_fd] == SLEEVE)
        {
            assert(m_fTypes[next_fd] == SLEEVE);
            assert(edge(cur_Hd) == edge(opposite(cur_Hd)));
            next_Hd = next(opposite(cur_Hd));
            if (is_border(edge(next_Hd)))
            {
                next_Hd = next(next_Hd);
            }

            assert(!is_border(edge(next_Hd)));
            assert(edge(cur_Hd) != edge(next_Hd));

            vMid = (point(source(next_Hd)) + point(target(next_Hd))) / 2.0;
            line.Setipt(vMid), line.Setif(face(next_Hd));
            line.Setedges(edge(next_Hd));
            line.SetHalfedges(next_Hd);
            // cout << "intersect " << edge(next_Hd) << " " << next_fd << " ipt=" << line.m_ipt.back().transpose() << endl;
            //cout << "before update: " << cur_Hd << "| current face=" << cur_fd << endl;
            m_eVisit[edge(next_Hd)] = true;
            cur_Hd = next_Hd;
            cur_fd = next_fd;
            //cout << "after update: " << cur_Hd << "| current face=" << cur_fd << endl;
        }
        else
        {
            assert(false && "There should not be junction face!");
        }

    } while (!bIsLastCptFound);
    lines.push_back(line);
}

void TMesh::PruneChordalAxis(const vector<Fd> &junctionFs, const vector<C3DLine> &rawLines, vector<C3DLine> &lines)
{
    auto GetLineLen = [this](const C3DLine &line)
    {
        double sum = 0;
        int num_ipts = line.GetiptCount();
        sum += (line.Getcpt(0) - line.Getipt(0)).norm();
        for (int i = 1; i < num_ipts; i++)
        {
            sum += (line.Getipt(i) - line.Getipt(i - 1)).norm();
        }
        sum += (line.Getipt(num_ipts - 1) - line.Getcpt(1)).norm();
        return sum;
    };

    auto GetLineArea = [this](const C3DLine &line)
    {
        double sum = 0;
        for (int i = 1; i < line.GetiptCount(); i++)
        {
            sum += Get3DFArea(line.Getif(i));
        }
        return sum;
    };

    auto MergeLines = [this, &lines](const std::vector<int> &mLines, const Fd &fd, int pruIdx) { // fd: junction face mLines lie on
        int l0, l1;                                                                              // l0: master line, l1: slave line, slave line will be connected to master line
        C3DLine cLine;
        if ((lines[mLines[0]].Getcf(0) == fd && GetFType(lines[mLines[0]].Getcf(1)) == JUNCTION) ||
            (lines[mLines[0]].Getcf(1) == fd && GetFType(lines[mLines[0]].Getcf(0)) == JUNCTION))
        {
            // mLines[0] is Junction Line, set it as master
            l0 = mLines[0];
            l1 = mLines[1];
        }
        else if ((lines[mLines[1]].Getcf(0) == fd && GetFType(lines[mLines[1]].Getcf(1)) == JUNCTION) ||
                 (lines[mLines[1]].Getcf(1) == fd && GetFType(lines[mLines[1]].Getcf(0)) == JUNCTION))
        {
            // mLines[1] is Junction Line, set it as master
            l0 = mLines[1];
            l1 = mLines[0];
        }
        else
        {
            // both lines to be connected are terminal lines
            l0 = mLines[0];
            l1 = mLines[1];
        }

        cout << __FILE__ << " " << __LINE__
             << " master line=" << l0 << " st=" << lines[l0].Getcf(0) << " end=" << lines[l0].Getcf(1)
             << " slave line=" << l1 << " st=" << lines[l1].Getcf(0) << " end=" << lines[l1].Getcf(1) << endl;
        // cpt0 is
        if (lines[l0].Getcf(0) == fd)
        {
            assert(lines[l0].Getcf(1) != fd);
            if (lines[l1].Getcf(0) == fd)
            {
                assert(lines[l1].Getcf(1) != fd);
                // l0-cf(1)-reversed_ipt-cf(0)---l1-cf(0)-ipt-cf(1)
                cLine.Setcpt(lines[l0].Getcpt(1), 0);
                cLine.Setcf(lines[l0].Getcf(1), 0);
                for (int j = lines[l0].GetiptCount() - 1; j >= 0; j--)
                {
                    cLine.Setipt(lines[l0].Getipt(j));
                    cLine.Setif(lines[l0].Getif(j));
                    cLine.Setedges(lines[l0].Getedge(j));
                }
                // ToDO
                for (int j = 0; j < lines[l1].GetiptCount(); j++)
                {
                    cLine.Setipt(lines[l1].Getipt(j));
                    cLine.Setif(lines[l1].Getif(j));
                    cLine.Setedges(lines[l1].Getedge(j));
                }
                cLine.Setcpt(lines[l1].Getcpt(1), 1);
                cLine.Setcf(lines[l1].Getcf(1), 1);
            }
            else
            {
                assert(lines[l1].Getcf(1) == fd && lines[l1].Getcf(0) != fd);
                // l0-cf(1)-reversed_ipt-cf(0)---l1-cf(1)-reversed ipt-cf(0)
                cLine.Setcpt(lines[l0].Getcpt(1), 0);
                cLine.Setcf(lines[l0].Getcf(1), 0);
                for (int j = lines[l0].GetiptCount() - 1; j >= 0; j--)
                {
                    cLine.Setipt(lines[l0].Getipt(j));
                    cLine.Setif(lines[l0].Getif(j));
                    cLine.Setedges(lines[l0].Getedge(j));
                }
                // ToDO
                for (int j = lines[l1].GetiptCount() - 1; j >= 0; j--)
                {
                    cLine.Setipt(lines[l1].Getipt(j));
                    cLine.Setif(lines[l1].Getif(j));
                    cLine.Setedges(lines[l1].Getedge(j));
                }
                cLine.Setcpt(lines[l1].Getcpt(0), 1);
                cLine.Setcf(lines[l1].Getcf(0), 1);
            }
        }
        else
        {
            assert(lines[l0].Getcf(1) == fd && lines[l0].Getcf(0) != fd);
            if (lines[l1].Getcf(0) == fd)
            {
                assert(lines[l1].Getcf(1) != fd);
                // l0-cf(0)-ipt-cf(1)---l1-cf(0)-ipt-cf(1)
                cLine.Setcpt(lines[l0].Getcpt(0), 0);
                cLine.Setcf(lines[l0].Getcf(0), 0);
                for (int j = 0; j < lines[l0].GetiptCount(); j++)
                {
                    cLine.Setipt(lines[l0].Getipt(j));
                    cLine.Setif(lines[l0].Getif(j));
                    cLine.Setedges(lines[l0].Getedge(j));
                }
                // ToDO
                for (int j = 0; j < lines[l1].GetiptCount(); j++)
                {
                    cLine.Setipt(lines[l1].Getipt(j));
                    cLine.Setif(lines[l1].Getif(j));
                    cLine.Setedges(lines[l1].Getedge(j));
                }
                cLine.Setcpt(lines[l1].Getcpt(1), 1);
                cLine.Setcf(lines[l1].Getcf(1), 1);
            }
            else
            {
                assert(lines[l1].Getcf(1) == fd && lines[l1].Getcf(0) != fd);
                // l0-cf(0)-ipt-cf(1)---l1-cf(1)-reversed ipt-cf(0)
                cLine.Setcpt(lines[l0].Getcpt(0), 0);
                cLine.Setcf(lines[l0].Getcf(0), 0);
                for (int j = 0; j < lines[l0].GetiptCount(); j++)
                {
                    cLine.Setipt(lines[l0].Getipt(j));
                    cLine.Setif(lines[l0].Getif(j));
                    cLine.Setedges(lines[l0].Getedge(j));
                }
                // ToDO
                for (int j = lines[l1].GetiptCount() - 1; j >= 0; j--)
                {
                    cLine.Setipt(lines[l1].Getipt(j));
                    cLine.Setif(lines[l1].Getif(j));
                    cLine.Setedges(lines[l1].Getedge(j));
                }
                cLine.Setcpt(lines[l1].Getcpt(0), 1);
                cLine.Setcf(lines[l1].Getcf(0), 1);
            }
        }

        lines.push_back(cLine);

        //  cout << __FILE__ << " " << __LINE__ << " l1=" << l1 << " before swap lines[" << lines.size() - 2 << "]="
        //      << lines[lines.size() - 2].Getcf(0) << "-" << lines[lines.size() - 2].Getcf(1)
        //      << " lines[l1]=" << lines[l1].Getcf(0) << "-" << lines[l1].Getcf(1) << endl;

        if (pruIdx == -1)
        {
            std::swap(lines[l0], lines[lines.size() - 1]);
            std::swap(lines[l1], lines[lines.size() - 2]);
            lines.erase(lines.begin() + lines.size() - 2, lines.end());
            return;
        }
        else
        {
            std::swap(lines[pruIdx], lines[lines.size() - 1]);

            //----------both position are available
            if (lines[l0] != lines[lines.size() - 2] &&
                lines[l1] != lines[lines.size() - 2] &&
                lines[l0] != lines[lines.size() - 3] &&
                lines[l1] != lines[lines.size() - 3])
            {
                // pos lines.size() - 2 && pos lines.size() - 3 are neigther occupied by l0 nor l1
                std::swap(lines[l0], lines[lines.size() - 2]);
                std::swap(lines[l1], lines[lines.size() - 3]);
            }
            //----------only one position is available
            else if (lines[l0] == lines[lines.size() - 2] &&
                     lines[l1] != lines[lines.size() - 3])
            {
                // pos lines.size() - 2 is occupied by l0, but pos lines.size()-3 is available for swapping
                std::swap(lines[l1], lines[lines.size() - 3]);
            }
            else if (lines[l0] == lines[lines.size() - 3] &&
                     lines[l1] != lines[lines.size() - 2])
            {
                // pos lines.size() - 3 is occupied by l0, but pos lines.size()-2 is available for swapping
                std::swap(lines[l1], lines[lines.size() - 2]);
            }
            else if (lines[l1] == lines[lines.size() - 2] &&
                     lines[l0] != lines[lines.size() - 3])
            {
                // pos lines.size() - 2 is occupied by l1, but pos lines.size()-3 is available for swapping
                std::swap(lines[l0], lines[lines.size() - 3]);
            }
            else if (lines[l1] == lines[lines.size() - 3] &&
                     lines[l0] != lines[lines.size() - 2])
            {
                // pos lines.size() - 3 is occupied by l1, but pos lines.size()-2 is available for swapping
                std::swap(lines[l0], lines[lines.size() - 2]);
            }
            //----------both position are occupied
            else
            {
                assert((lines[l0] == lines[lines.size() - 2] && lines[l1] == lines[lines.size() - 3]) ||
                       (lines[l1] == lines[lines.size() - 2] && lines[l0] == lines[lines.size() - 3]));
            }
            lines.erase(lines.begin() + lines.size() - 3, lines.end());
        }

    };

    cout << "**************************************************prune..." << endl;
    if (junctionFs.size() == 1)
    {
        cout << __FILE__ << " " << __LINE__ << " only 1 junction faces" << endl;
        assert(rawLines.size() == 3);
        double maxLen = std::numeric_limits<double>::min();
        int maxIdx = -1;
        for (int i = 0; i < rawLines.size(); i++)
        {
            const C3DLine &line = rawLines[i];
            double len = GetLineLen(line);
            if (len > maxLen)
            {

                maxLen = len;
                maxIdx = i;
            }
        }
        assert(maxIdx != -1);
        lines.push_back(rawLines[maxIdx]);
        return;
    }

    double avgArea = Get3DArea() / num_faces();
    cout << __FILE__ << " " << __LINE__ << " avgArea=" << avgArea << endl;
    double maxArea = std::numeric_limits<double>::min();
    map<Fd, double> fAreas;
    for (const Fd &fd : junctionFs)
    {
        fAreas[fd] = Get3DFArea(fd);
        if (fAreas[fd] > maxArea)
        {
            maxArea = fAreas[fd];
        }
    }
    cout << __FILE__ << " " << __LINE__ << " maxArea=" << maxArea << endl;
    vector<double> lineAreas;
    for (int l = 0; l < rawLines.size(); l++)
    {
        const C3DLine &line = rawLines[l];
        lineAreas.push_back(GetLineArea(line));
        cout << __FILE__ << " " << __LINE__ << " line" << l << " area=" << lineAreas[l] << endl;
    }

    // for each junction:
    //     decides delete, prune or snap
    //     if delete:
    //        only retain the line connected to junction face
    //     if prune:
    //        the line with smallest area (connecting to terminal face) is deleted,
    //        the other twos is merged into one by connecting edge midpoints
    //     if snap:
    //        find the best-connect-line minimizing curvature change
    //        two lines is merged into one by connecting edge midpoints (derived from best-connect-line)
    //        find 'snap' point for the remaining line (the intersection between best-connect-line and the remaining line)

    std::vector<Fd> toBePruned, toBeSnapped, toBeDeleted;
    std::map<int, bool> isVisit; // whether this line was visit int: line index in rawLines
    for (const Fd &fd : junctionFs)
    {
        cout << "--------------------------" << fd << " area=" << fAreas[fd] << " thresh=" << fAreas[fd] * 0.333 << endl;
        // find three chordals belonging to fd
        std::vector<int> fLines; // line index in rawLines

        std::vector<int> terminalLines, junctionLines;
        // terminal line: one end is connected to a terminal face
        // junction line: two ends connect to junction faces
        for (int i = 0; i < rawLines.size(); i++)
        {
            const C3DLine &line = rawLines[i];
            const Fd &stF = line.Getcf(0);
            const Fd &endF = line.Getcf(1);
            if (stF == fd || endF == fd)
            {
                fLines.push_back(i);
                if (stF == fd)
                {
                    const FaceType &fType = GetFType(endF);
                    if (fType == TERMINAL)
                    {
                        terminalLines.push_back(i);
                    }
                    else
                    {
                        assert(fType == JUNCTION);
                        junctionLines.push_back(i);
                    }
                }
                else
                {
                    assert(endF == fd);
                    const FaceType &fType = GetFType(stF);
                    if (fType == TERMINAL)
                    {
                        terminalLines.push_back(i);
                    }
                    else
                    {
                        assert(fType == JUNCTION);
                        junctionLines.push_back(i);
                    }
                }
            }
        }
        assert(fLines.size() == 3);
        assert(terminalLines.size() + junctionLines.size() == 3);
        cout << "terminalLines: ";
        for (const int &tl : terminalLines)
        {
            cout << tl << " ";
        }
        cout << endl;
        cout << "junctionLines: ";
        for (const int &jl : junctionLines)
        {
            cout << jl << " ";
        }
        cout << endl;

        double thresh = fAreas[fd] * 0.333;
        cout << "lin1Area=" << lineAreas[fLines[0]] << " lin2Area=" << lineAreas[fLines[1]] << " lin3Area=" << lineAreas[fLines[2]] << endl;
        if ((lineAreas[fLines[0]] > thresh && lineAreas[fLines[1]] > thresh && lineAreas[fLines[2]] > thresh) ||
            junctionLines.size() == 3) //ToBe Snapped
        {
            toBeSnapped.push_back(fd);
            for (const int &lidx : fLines)
            {
                if (!isVisit[lidx])
                {
                    lines.push_back(rawLines[lidx]);
                    isVisit[lidx] = true;
                }
            }
        }
        else
        {

            if (fAreas[fd] < avgArea && terminalLines.size() == 2)
            {
                toBeDeleted.push_back(fd);
                for (const int &lidx : fLines)
                {
                    isVisit[lidx] = true;
                }
            }
            else
            {
                toBePruned.push_back(fd);
                for (const int &lidx : fLines)
                {
                    if (!isVisit[lidx])
                    {
                        lines.push_back(rawLines[lidx]);
                        isVisit[lidx] = true;
                    }
                }
            }
        }
    }

    assert(lines.size() <= rawLines.size());
    cout << __FILE__ << " " << __LINE__ << " lines.size()=" << lines.size() << endl;

    cout << __FILE__ << __LINE__ << " toBeDeleted Size=" << toBeDeleted.size() << endl;
    for (const Fd &fd : toBeDeleted)
    {
        cout << __FILE__ << __LINE__ << " toBeDeleted " << fd << endl;
    }
    cout << __FILE__ << __LINE__ << " toBePruned Size=" << toBePruned.size() << endl;
    for (const Fd &fd : toBePruned)
    {
        cout << __FILE__ << __LINE__ << " toBePruned " << fd << endl;
        // find three chordals belonging to fd
        std::vector<int> fLines; // line index in lines
        std::vector<int> junctionLines, terminalLines;
        std::map<int, double> lAreas; // int: line index in lines
        for (int i = 0; i < lines.size(); i++)
        {

            const C3DLine &line = lines[i];
            const Fd &stF = line.Getcf(0);
            const Fd &endF = line.Getcf(1);
            cout << "fd=" << fd << " stF=" << stF << " endF=" << endF << " line=" << i << endl;
            if (stF == fd || endF == fd)
            {
                fLines.push_back(i);
                lAreas[i] = GetLineArea(lines[i]);
                cout << "---------------------fd="
                     << fd << " stF=" << stF << " endF=" << endF << " line=" << i
                     << " lineArea=" << lAreas[i] << endl;
                const FaceType &stFType = GetFType(stF);
                const FaceType &endFType = GetFType(endF);
                if (stFType == JUNCTION && endFType == JUNCTION)
                {
                    junctionLines.push_back(i);
                }
                else
                {
                    terminalLines.push_back(i);
                }
            }
        }
        cout << __FILE__ << " " << __LINE__ << " fLines=" << fLines.size() << endl;
        double thresh = 0.1 * avgArea;
        if (fLines.size() == 1)
        {
            int lidx = fLines[0];
            if (lAreas[lidx] < thresh)
            {
                std::swap(lines[lidx], lines[lines.size() - 1]);
                lines.erase(lines.begin() + lines.size() - 1, lines.end());
            }
            continue;
        }

        if (fLines.size() == 2)
        {
            int l0 = fLines[0];
            int l1 = fLines[1];
            if (lAreas[l0] < thresh && lAreas[l1] < thresh)
            {
                cout << __FILE__ << " " << __LINE__ << " Delete two lines..." << endl;
                if (lines[l0] != lines[lines.size() - 1] &&
                    lines[l1] != lines[lines.size() - 1] &&
                    lines[l0] != lines[lines.size() - 2] &&
                    lines[l1] != lines[lines.size() - 2])
                {
                    std::swap(lines[l0], lines[lines.size() - 1]);
                    std::swap(lines[l1], lines[lines.size() - 2]);
                }
                else if (lines[l0] == lines[lines.size() - 1] &&
                         lines[l1] != lines[lines.size() - 2])
                {
                    std::swap(lines[l1], lines[lines.size() - 2]);
                }
                else if (lines[l0] == lines[lines.size() - 2] &&
                         lines[l1] != lines[lines.size() - 1])
                {
                    std::swap(lines[l1], lines[lines.size() - 1]);
                }
                else if (lines[l1] == lines[lines.size() - 1] &&
                         lines[l0] != lines[lines.size() - 2])
                {
                    std::swap(lines[l0], lines[lines.size() - 2]);
                }
                else if (lines[l1] == lines[lines.size() - 2] &&
                         lines[l0] != lines[lines.size() - 1])
                {
                    std::swap(lines[l0], lines[lines.size() - 1]);
                }
                else
                {
                    assert((lines[l0] == lines[lines.size() - 1] && lines[l1] == lines[lines.size() - 2]) ||
                           (lines[l1] == lines[lines.size() - 1] && lines[l0] == lines[lines.size() - 2]));
                }
                lines.erase(lines.begin() + lines.size() - 2, lines.end());
            }
            else if (lAreas[l0] < thresh && lAreas[l1] > thresh)
            {
                std::swap(lines[l0], lines[lines.size() - 1]);
                lines.erase(lines.begin() + lines.size() - 1, lines.end());
            }
            else if (lAreas[l1] < thresh && lAreas[l0] > thresh)
            {
                std::swap(lines[l1], lines[lines.size() - 1]);
                lines.erase(lines.begin() + lines.size() - 1, lines.end());
            }
            else
            {
                cout << __FILE__ << " " << __LINE__ << " Merge two lines..." << endl;
                MergeLines(fLines, fd, -1);
            }

            continue;
        }
        assert(fLines.size() == 3);

        int pruIdx = -1;
        if (junctionLines.size() == 2 && terminalLines.size() == 1)
        {
            // if there two junction lines and one terminal line
            // int j0 = junctionLines[0];
            // int j1 = junctionLines[1];
            // int t = terminalLines[0];
            // if (lAreas[j0] < lAreas[t] && lAreas[j1] < lAreas[t])
            // {
            //     //j0 and j1 is directly adjacent to fd
            //     Fd fd0 = lines[j0].Getcf(0) == fd ? lines[j0].Getcf(1) : lines[j0].Getcf(0);
            //     Fd fd1 = lines[j1].Getcf(0) == fd ? lines[j1].Getcf(1) : lines[j1].Getcf(0);
            //     if (Get3DFArea(fd0) < Get3DFArea(fd1))
            //     {
            //         // select the line connecting the smallest end junction face to prune
            //         pruIdx = j0;
            //     }
            //     else
            //     {
            //         pruIdx = j1;
            //     }
            // }
            // else
            // {
            //     // Two junction lines will be merged
            //     pruIdx = terminalLines[0];
            // }
            pruIdx = terminalLines[0];
        }
        else if (terminalLines.size() >= 2)
        {
            // if there are more than one terminal lines, line with smallest area will be pruned
            double minArea = std::numeric_limits<double>::max();
            for (const int &lidx : terminalLines)
            {
                if (lAreas[lidx] < minArea)
                {
                    minArea = lAreas[lidx];
                    pruIdx = lidx;
                }
            }
        }
        else
        {
            assert(junctionLines.size() == 3);
            toBeSnapped.push_back(fd);
            continue;
        }
        assert(pruIdx >= 0);

        // the other two lines will be merged by connecting edge-midpoint on Junction Face
        std::vector<int> mLines; // lines to be merged (value is line index in lines)
        for (const int &lidx : fLines)
        {
            if (lidx != pruIdx)
            {
                mLines.push_back(lidx);
            }
        }
        cout << __FILE__ << " " << __LINE__ << " mLines=" << mLines[0] << ", " << mLines[1] << endl;
        MergeLines(mLines, fd, pruIdx);
        cout << __FILE__ << " " << __LINE__ << " lines=" << lines.size() << endl;
    }

    cout << __FILE__ << __LINE__ << " toBeSnapped Size=" << toBeSnapped.size() << endl;
    for (const Fd &fd : toBeSnapped)
    {
        cout << __FILE__ << __LINE__ << " toBeSnapped " << fd << endl;
    }
}

bool TMesh::GetCutLine(const Pnt3 &stPt, const Pnt3 &endPt, C3DLine &line)
{
    // ToDO: when doing intersections, consider on "vertex" and on "edge"
    // find the start face and end face, and get intersection points with edges
    ResetEVisit();
    for (const Fd &fd : faces())
    {
        Hd hd = halfedge(fd);
        Vd v0 = source(hd);
        Vd v1 = target(hd);
        Vd v2 = target(next(hd));
        std::pair<IntersectionType, int> ret = IsPointIntersectTriangle(stPt, fd, *this);
        IntersectionType inter_type = ret.first;
        int idx = ret.second;
        if (inter_type != EMPTY)
        {
            // cout << "st is " << inter_names[inter_type] << " idx=" << idx << " stFd=" << fd << endl;
            line.Setcpt(stPt, 0);
            line.SetcType(inter_type, 0);
            line.SetcIdx(idx, 0);
            line.Setcf(fd, 0);
            break;
        }
    }

    for (const Fd &fd : faces())
    {
        Hd hd = halfedge(fd);
        Vd v0 = source(hd);
        Vd v1 = target(hd);
        Vd v2 = target(next(hd));
        std::pair<IntersectionType, int> ret = IsPointIntersectTriangle(endPt, fd, *this);
        IntersectionType inter_type = ret.first;
        int idx = ret.second;
        if (inter_type != EMPTY)
        {
            // cout << "end is " << inter_names[inter_type] << " idx=" << idx << " endFd=" << fd << endl;
            line.Setcpt(endPt, 1);
            line.SetcType(inter_type, 1);
            line.SetcIdx(idx, 1);
            line.Setcf(fd, 1);
            break;
        }
    }

    const int &stIdx = line.GetcIdx(0);
    const Fd &stFd = line.Getcf(0);
    const IntersectionType &stType = line.GetcType(0);
    const int &endIdx = line.GetcIdx(1);
    const IntersectionType &endType = line.GetcType(1);
    const Fd &endFd = line.Getcf(1);
    Fd curFd;

    if (stType == ON_FACE)
    {
        Hd stHd = halfedge(stFd);
        for (const Hd &hd : halfedges_around_face(stHd))
        {
            Vd v0 = source(hd);
            Vd v1 = target(hd);
            Ed ed = edge(hd);

            if (IsTwoLineIntersectIn2D(point(v0), point(v1), stPt, endPt))
            {
                Pnt3 p = GetTwoLineInterPt(point(v0), point(v1), stPt, endPt);
                line.Setipt(p);
                line.Setedges(ed);
                line.Setif(face(hd));
                line.SetiType(ON_EDGE);
                line.SetiIdx(ed);
                m_eVisit[ed] = true;
                curFd = face(opposite(hd));
                // cout << "...intersect with " << ed << ", " << face(hd) << " on point " << p << endl;
                break;
            }
        }
    }
    else if (stType == ON_EDGE)
    {
        Hd stHd = halfedge(stFd);
        for (const Hd &hd : halfedges_around_face(stHd))
        {
            Ed ed = edge(hd);
            if (ed.idx() == stIdx)
            {
                m_eVisit[ed] = true;
                curFd = face(opposite(hd));
                break;
            }
        }
    }
    else if (stType == ON_VERTEX)
    {
        //ToDO:
    }
    else
    {
        return false;
    }

    do
    {
        Hd curHd = halfedge(curFd);
        for (const Hd &hd : halfedges_around_face(curHd))
        {
            Ed ed = edge(hd);
            if (m_eVisit[ed])
                continue;

            Vd v0 = source(hd);
            Vd v1 = target(hd);
            if (IsTwoLineIntersectIn2D(point(v0), point(v1), stPt, endPt))
            {
                Pnt3 p = GetTwoLineInterPt(point(v0), point(v1), stPt, endPt);
                line.Setipt(p);
                line.Setedges(ed);
                line.Setif(face(hd));
                line.SetiType(ON_EDGE);
                line.SetiIdx(ed);
                m_eVisit[ed] = true;
                // cout << "intersect with " << ed << ", " << face(hd) << " on point " << p << endl;
                curFd = face(opposite(hd));
                // cout << "update curFd=" << curFd << endl;
            }
        }

    } while (curFd != endFd);
}

bool TMesh::CutMeshHalfedge(const C3DLine &line)
{
    // // ToDO:
    // const int &stIdx = line.GetcIdx(0);
    // const Fd &stFd = line.Getcf(0);
    // const IntersectionType &stType = line.GetcType(0);
    // const int &endIdx = line.GetcIdx(1);
    // const IntersectionType &endType = line.GetcType(1);
    // const Fd &endFd = line.Getcf(1);
    // if (stType == ON_FACE)
    // {
    //     // split start face into 4 triangles
    //     const Pnt3 &stPt = line.Getcpt(0);
    //     const Pnt3 &iPt0 = line.Getipt(0);
    //     const Ed &iEd0 = line.Getedge(0);
    //     Hd hd0 = halfedge(iEd0);
    //     if (face(hd0) != stFd)
    //     {
    //         hd0 = opposite(hd0);
    //     }
    //     assert(face(hd0) == stFd);
    //     Hd hd1 = next(hd0);
    //     Hd hd2 = next(hd1);

    //     Vd uV0 = add_vertex(stPt);
    //     Vd uV1 = add_vertex(iPt0);
    //     Vd lV0 = add_vertex(stPt);
    //     Vd lV1 = add_vertex(iPt0);
    //     Hd nh0 = add_edge();
    //     hconn_[nh0].vertex_ = uV1;
    //     vconn_[uV1].halfedge_ = nh0;

    //     // split into 3 triangles
    //     for (int i = 0; i < line.GetiptCount() - 1; i++)
    //     {
    //     }

    //     // split end face into 4 triangles
    //     if (endType == ON_FACE)
    //     {
    //     }
    // }
    // else if (stType == ON_EDGE)
    // {
    //     // split start face into 3 triangles

    //     // split into 3 triangles
    //     for (int i = 0; i < line.GetiptCount() - 1; i++)
    //     {
    //     }

    //     // split end face into 4 triangles
    //     if (endType == ON_FACE)
    //     {
    //     }
    // }
    // else if (stType == ON_VERTEX)
    // {
    //     //ToDO
    // }
    // else
    // {
    //     return false;
    // }
    return false;
}

bool TMesh::CutMesh(const vector<C3DLine> &cutLines, const vector<vector<Pnt3>> &child_contours, vector<vector<int>> &sew_lines)
{
    if (cutLines.size() <= 0)
        return true;

    using namespace Eigen;
    MatrixXd V = GetVMat();
    MatrixXi F = GetFMat();

    int num_newV = number_of_vertices();
    int num_newF = number_of_faces();

    for (const C3DLine &line : cutLines)
    {
        num_newV += (line.GetiptCount() + 2) * 2; // intersection_points + start&end control_points
        num_newF += (line.GetiptCount() - 1) * 2;
        if (line.GetcType(0) == ON_FACE)
        {
            num_newF += 3;
        }
        else if (line.GetcType(0) == ON_EDGE)
        {
            num_newF += 2;
        }
        else
        {
            //ToDO
        }

        if (line.GetcType(1) == ON_FACE)
        {
            num_newF += 3;
        }
        else if (line.GetcType(1) == ON_EDGE)
        {
            num_newF += 2;
        }
        else
        {
            //ToDO
        }
    }

    //--------- split faces --------------
    V.resize(num_newV, 3);
    F.resize(num_newF, 3);
    V.block(0, 0, number_of_vertices(), 3) = m_V;
    F.block(0, 0, number_of_faces(), 3) = m_F;

    int vidx = number_of_vertices();
    int fidx = number_of_faces();

    for (int i = 0; i < cutLines.size(); i++)
    {
        // cout << "------------------------------------------------------------------------------CutLine" << i << endl;
        const C3DLine &line = cutLines[i];
        const vector<Pnt3> &child_contour = child_contours[i];
        vector<int> sew_line;

        const int &stIdx = line.GetcIdx(0);
        const Fd &stFd = line.Getcf(0);
        const IntersectionType &stType = line.GetcType(0);
        const int &endIdx = line.GetcIdx(1);
        const IntersectionType &endType = line.GetcType(1);
        const Fd &endFd = line.Getcf(1);

        if (stType == ON_FACE)
        {
            // cout << ".....................................split startFace " << stFd << endl;
            // split start face into 4 triangles
            Fd prevFd = stFd;
            int prev_v, prev_v_cp, cur_v, cur_v_cp;
            const Pnt3 &stPt = line.Getcpt(0);
            const Pnt3 &iPt0 = line.Getipt(0);
            Ed iEd0 = line.Getedge(0);
            Hd hd0 = halfedge(iEd0, 0);
            if (face(hd0) != stFd)
            {
                hd0 = halfedge(iEd0, 1);
            }

            assert(face(hd0) == stFd);

            int v0 = source(hd0).idx();
            int v1 = target(hd0).idx();
            int v2 = target(next(hd0)).idx();

            int nv0 = vidx, nv1 = vidx + 1;
            int nv0_cp = vidx + 2, nv1_cp = vidx + 3;
            // add 4 vertices
            Copy(V, vidx, stPt), vidx++;
            Copy(V, vidx, iPt0), vidx++;
            Copy(V, vidx, stPt), vidx++;
            Copy(V, vidx, iPt0), vidx++;

            // change 1 face's idxs, and add another 3 faces
            int f0 = Copy(F, stFd.idx(), v0, nv1, nv0);
            int f1 = Copy(F, fidx++, v0, nv0, v2);
            int f2 = Copy(F, fidx++, v1, nv0_cp, nv1_cp);
            int f3 = Copy(F, fidx++, v1, v2, nv0_cp);
            if (!IsPointInPolygon(point(Vd(v0)), child_contour))
            {
                sew_line.push_back(nv0);
                sew_line.push_back(nv1);
            }
            else
            {
                sew_line.push_back(nv0_cp);
                sew_line.push_back(nv1_cp);
            }

            prev_v = nv1;
            prev_v_cp = nv1_cp;
            // split into 3 triangles
            for (int i = 1; i < line.GetiptCount(); i++)
            {
                Fd fd = line.Getif(i);

                Ed eb = line.Getedge(i);
                Hd hb = halfedge(eb, 0);
                const Pnt3 &pb = line.Getipt(i);
                if (face(hb) != fd)
                {
                    hb = halfedge(eb, 1);
                }
                Ed ea = line.Getedge(i - 1);
                Hd ha = halfedge(ea);
                const Pnt3 &pa = line.Getipt(i - 1);

                //  cout << ".....................................split " << fd << endl;
                assert(face(hb) == fd);
                assert(hb != ha);
                assert(face(ha) == prevFd);

                // add 2 vertices
                cur_v = vidx, cur_v_cp = vidx + 1;
                Copy(V, vidx, pb), vidx++;
                Copy(V, vidx, pb), vidx++;
                // change 1 face's idxs and add another two faces
                if (source(ha) == source(hb))
                {
                    v0 = source(hb).idx();
                    v1 = target(hb).idx();
                    v2 = target(ha).idx();

                    nv0 = cur_v, nv1 = prev_v;
                    nv0_cp = cur_v_cp, nv1_cp = prev_v_cp;

                    int f0 = Copy(F, fd.idx(), v0, nv0, nv1);
                    int f1 = Copy(F, fidx++, v1, nv1_cp, nv0_cp);
                    int f2 = Copy(F, fidx++, v2, nv1_cp, v1);

                    if (!IsPointInPolygon(point(Vd(v0)), child_contour))
                    {
                        sew_line.push_back(cur_v);
                    }
                    else
                    {
                        sew_line.push_back(cur_v_cp);
                    }
                }
                else if (target(ha) == target(hb))
                {
                    v0 = target(ha).idx();
                    v1 = source(ha).idx();
                    v2 = source(hb).idx();

                    nv0 = prev_v, nv1 = cur_v;
                    nv0_cp = prev_v_cp, nv1_cp = cur_v_cp;

                    int f0 = Copy(F, fd.idx(), v0, nv0_cp, nv1_cp);
                    int f1 = Copy(F, fidx++, v1, nv1, nv0);
                    int f2 = Copy(F, fidx++, v2, nv1, v1);

                    if (!IsPointInPolygon(point(Vd(v0)), child_contour))
                    {
                        sew_line.push_back(cur_v_cp);
                    }
                    else
                    {
                        sew_line.push_back(cur_v);
                    }
                }
                else
                {
                    assert(false);
                }

                prevFd = fd;
                prev_v = cur_v;
                prev_v_cp = cur_v_cp;
            }

            // split end face into 4 triangles
            if (endType == ON_FACE)
            {
                // split start face into 4 triangles
                const Fd &endFd = line.Getcf(1);
                const Pnt3 &endPt = line.Getcpt(1);
                Ed ed1 = line.m_edges.back();
                Hd hd1 = halfedge(ed1, 0);
                // cout << ".....................................split endFace " << endFd << endl;
                if (face(hd1) != endFd)
                {
                    hd1 = halfedge(ed1, 1);
                }
                assert(face(hd1) == endFd);
                int v0 = source(hd1).idx();
                int v1 = target(hd1).idx();
                int v2 = target(next(hd1)).idx();

                // add 2 vertices
                cur_v = vidx, cur_v_cp = vidx + 1;
                Copy(V, vidx, endPt), vidx++;
                Copy(V, vidx, endPt), vidx++;
                nv0 = prev_v, nv1 = cur_v;
                nv0_cp = prev_v_cp, nv1_cp = cur_v_cp;

                int f0 = Copy(F, endFd.idx(), v0, nv0_cp, nv1_cp);
                int f1 = Copy(F, fidx++, v0, nv1_cp, v2);
                int f2 = Copy(F, fidx++, v1, nv1, nv0);
                int f3 = Copy(F, fidx++, v2, nv1, v1);

                if (!IsPointInPolygon(point(Vd(v0)), child_contour))
                {
                    sew_line.push_back(cur_v_cp);
                }
                else
                {
                    sew_line.push_back(cur_v);
                }
                sew_lines.push_back(sew_line);
            }
        }
        else if (stType == ON_EDGE)
        {
            // split start face into 3 triangles

            // split into 3 triangles
            for (int i = 0; i < line.GetiptCount() - 1; i++)
            {
            }

            // split end face into 4 triangles
            if (endType == ON_FACE)
            {
            }
        }
        else if (stType == ON_VERTEX)
        {
            //ToDO
        }
        else
        {
            return false;
        }
    }
    assert(vidx == num_newV);
    assert(fidx == num_newF);
    m_V = V;
    m_F = F;
    CreateHalfedgeMesh(V, F);

    if (true) // Debug: check consistency between m_V and halfedge mesh
    {
        for (Vd vd : vertices())
        {
            const Pnt3 &p = point(vd);
            RowVector3d v = m_V.row(vd.idx());
            RowVector3d vp(p.x(), p.y(), p.z());
            if ((v - vp).norm() > 1.0e-6)
            {
                assert(false);
            }
        }
        for (Fd fd : faces())
        {
            int f = fd.idx();
            Hd hd = halfedge(fd);
            Vd v0 = source(hd);
            Vd v1 = target(hd);
            Vd v2 = target(next(hd));
            if (m_F(f, 0) != v0.idx() || m_F(f, 1) != v1.idx() || m_F(f, 2) != v2.idx())
            {
                cout << "m_F(" << f << "): " << m_F.row(f) << ", face:" << v0.idx() << ", " << v1.idx() << ", " << v2.idx() << endl;
                assert(false);
            }
        }
    }
}

void TMesh::AttachMeshes(const vector<vector<int>> &sew_idxs, const std::vector<TMesh *> &subMeshes)
{
    // sew submeshes to this mesh by 'remove' sewing line vertexs from submesh, and replace the 'removed' vertex
    // with corresponding vertex from this mesh
    assert(sew_idxs.size() == subMeshes.size());

    using namespace Eigen;
    MatrixXd V;
    MatrixXi F;
    int num_newV = number_of_vertices();
    int num_newF = number_of_faces();

    for (int i = 0; i < subMeshes.size(); i++)
    {
        TMesh *subMesh = subMeshes[i];
        const vector<int> &sew_idx = sew_idxs[i];
        num_newV += subMesh->number_of_vertices();
        num_newV -= sew_idx.size();
        num_newF += subMesh->number_of_faces();
        // cout << "subMesh" << i << ": num_vertices=" << subMesh->num_vertices() << endl;
        // cout << "num_newV: " << num_newV << " num_vertices:" << num_vertices() << " sew_idx:" << sew_idxs[i].size() << endl;
    }

    V.resize(num_newV, 3);
    F.resize(num_newF, 3);
    map<int, bool> fixedV;
    fixedV = m_fixedV;
    // int num_fixed = GetNumFixed();
    // cout << ".......................................................before attach fixedV:" << num_fixed << endl;
    map<int, int> layerV = m_layerV;
    map<int, LConstrainType> constrainV = m_lConstrain;

    V.block(0, 0, number_of_vertices(), 3) = m_V;
    F.block(0, 0, number_of_faces(), 3) = m_F;
    int vidx = number_of_vertices();
    int fidx = number_of_faces();

    for (int i = 0; i < subMeshes.size(); i++)
    {
        TMesh *subMesh = subMeshes[i];
        const MatrixXd &subV = subMesh->GetVMat();
        const MatrixXi &subF = subMesh->GetFMat();
        const vector<int> &sew_idx = sew_idxs[i];
        if (false)
        {
            // std::map<int, int> vert_map;
            // int num_found = 0;
            // for (int j = 0; j < subV.rows(); j++)
            // {
            //     for (int vi : sew_idx)
            //     {
            //         if ((subV.row(j) - V.row(vi)).norm() < 1e-6)
            //         {
            //             vert_map[j] = vi;
            //             cout << "find subV " << j << " 's parent vertex " << vi << endl;
            //             num_found++;
            //         }
            //     }
            // }
            // cout << "-------------------------totoally found " << num_found << " sew vertexs, vert_map=" << vert_map.size() << endl;
        }

        map<int, bool> found;   // key is vidx submesh, whether the corresponding sew_vertex found in this mesh
        map<int, int> vert_map; // key is vidx in subMesh, val is vidx in this mesh
        for (int fj = 0; fj < subF.rows(); fj++)
        {
            int new_v[3];
            for (int k = 0; k < 3; k++)
            {
                // ------------------- update indices---------------------------
                int vk = subF(fj, k);
                if (!found[vk])
                {
                    // try to find corresponding sew vertex in parent mesh
                    RowVectorXd p = subV.row(vk);
                    for (int vi : sew_idx)
                    {
                        RowVectorXd pp = V.row(vi);
                        if ((p - pp).norm() < 1e-6)
                        {
                            found[vk] = true;
                            vert_map[vk] = vi;
                            break;
                        }
                    }
                    if (!found[vk])
                    {
                        // add vertex into this mesh
                        new_v[k] = this->Copy(V, vidx++, subV.row(vk));
                        found[vk] = true;
                        vert_map[vk] = new_v[k];
                        // ------------------- fixed vertices map ---------------------------
                        if (subMesh->IsFixed(vk))
                        {
                            fixedV[new_v[k]] = true;
                            // num_fixed++; //for debug
                        }
                        int layerid = subMesh->GetLayerId(vk);
                        if (layerid != -1)
                        {
                            layerV[new_v[k]] = layerid;
                            constrainV[new_v[k]] = subMesh->GetLayerConstrainType(vk);
                        }
                    }
                    else
                    {
                        // replace with vertex from parent mesh
                        new_v[k] = vert_map[vk];
                    }
                }
                else
                {
                    // replace with vertex from parent mesh
                    new_v[k] = vert_map[vk];
                }
            }
            int nf = Copy(F, fidx++, new_v[0], new_v[1], new_v[2]);
        }
    }

    assert(vidx == num_newV);
    assert(fidx == num_newF);
    m_V = V;
    m_F = F;
    m_fixedV.clear();
    m_fixedV = fixedV;
    m_layerV = layerV;
    m_lConstrain = constrainV;
    // cout << ".......................................................after attach fixedV:" << GetNumFixed() << " count=" << num_fixed << endl;
    cout << ".......................................................after attach m_layerV:" << GetLayerVMap().size() << endl;
    cout << ".......................................................after attach m_lConstrain:" << GetLayerConstrainMap().size() << endl;
    CreateHalfedgeMesh(V, F);
}

void TMesh::CovarianceScatterMatrix()
{
    cout << "----------------------Igl Covariance-----------------------" << endl;
    using namespace Eigen;
    using namespace igl;
    const MatrixXd &V = GetVMat();
    const MatrixXi &F = GetFMat();
    int n = V.rows();
    int m = F.rows();
    int nr = n; // num rotations

    cout << "V" << endl;
    cout << V << endl;
    cout << "F" << endl;
    cout << F << endl;
    MatrixXd C;
    igl::cotmatrix_entries(V, F, C);
    cout << "C" << endl;
    cout << C << endl;

    MatrixXi edges;
    edges.resize(3, 2);
    edges << 1, 2,
        2, 0,
        0, 1;

    SparseMatrix<double> Kx, Ky, Kz;
    vector<Triplet<double>> Kx_IJV, Ky_IJV, Kz_IJV;
    Kx.resize(V.rows(), V.rows());
    Ky.resize(V.rows(), V.rows());
    Kz.resize(V.rows(), V.rows());

    int v = 1;
    int num = 0;
    for (int i = 0; i < m; i++)
    {
        // cout << "......................................face" << i << endl;
        for (int e = 0; e < edges.rows(); e++)
        {
            int source = F(i, edges(e, 0));
            int dest = F(i, edges(e, 1));
            // RowVector3d eij = C(i, e) * (V.row(source) - V.row(dest)) / 3.0;
            RowVector3d eij = (V.row(source) - V.row(dest));
            // cout << e << "------------------> edge:" << source << "-" << dest << ": " << eij << " C(i, e)=" << C(i, e) << endl;
            // if (source == v)
            // {
            //     printf("[%d, %d]=e[%d, %d]\n", source, source, source, dest);
            //     num++;
            // }
            // if (dest == v)
            // {
            //     printf("[%d, %d]=-e[%d, %d]\n", dest, dest, source, dest);
            //     num++;
            // }
            for (int f = 0; f < edges.rows(); f++)
            {
                int Rs = F(i, edges(f, 0));
                int Rd = F(i, edges(f, 1));
                // cout << "---> e " << Rs << "-" << Rd << endl;
                if (Rs == source && Rd == dest)
                {
                    Kx_IJV.push_back(Triplet<double>(Rs, Rd, eij.x()));
                    Kx_IJV.push_back(Triplet<double>(Rd, Rs, -eij.x()));
                    Ky_IJV.push_back(Triplet<double>(Rs, Rd, eij.y()));
                    Ky_IJV.push_back(Triplet<double>(Rd, Rs, -eij.y()));
                    Kz_IJV.push_back(Triplet<double>(Rs, Rd, eij.z()));
                    Kz_IJV.push_back(Triplet<double>(Rd, Rs, -eij.z()));
                    // if (Rs == v)
                    // {
                    //     printf("[%d, %d]=e[%d, %d]\n", Rs, Rd, source, dest);
                    //     num++;
                    // }
                    // if (Rd == v)
                    // {
                    //     printf("[%d, %d]=-e[%d, %d]\n", Rd, Rs, source, dest);
                    //     num++;
                    // }
                }
                else if (Rd == source) // incident to the source vertex
                {
                    Kx_IJV.push_back(Triplet<double>(Rd, Rs, eij.x()));
                    Ky_IJV.push_back(Triplet<double>(Rd, Rs, eij.y()));
                    Kz_IJV.push_back(Triplet<double>(Rd, Rs, eij.z()));
                    // if (Rd == v)
                    // {
                    //     printf("[%d, %d]=e[%d, %d]\n", Rd, Rs, source, dest);
                    //     num++;
                    // }
                }
                else if (Rs == dest) // incident to the Rd vertex
                {
                    Kx_IJV.push_back(Triplet<double>(Rs, Rd, -eij.x()));
                    Ky_IJV.push_back(Triplet<double>(Rs, Rd, -eij.y()));
                    Kz_IJV.push_back(Triplet<double>(Rs, Rd, -eij.z()));
                    // if (Rs == v)
                    // {
                    //     printf("[%d, %d]=-e[%d, %d]\n", Rs, Rd, source, dest);
                    //     num++;
                    // }
                }
                // cout << "---> end e" << Rs << "-" << Rd << endl;
            }
            Kx_IJV.push_back(Triplet<double>(source, source, eij.x()));
            Kx_IJV.push_back(Triplet<double>(dest, dest, -eij.x()));
            Ky_IJV.push_back(Triplet<double>(source, source, eij.y()));
            Ky_IJV.push_back(Triplet<double>(dest, dest, -eij.y()));
            Kz_IJV.push_back(Triplet<double>(source, source, eij.z()));
            Kz_IJV.push_back(Triplet<double>(dest, dest, -eij.z()));
            // cout << e << "------------------> end edge:" << source << "-" << dest << endl;
        }
        // cout << "......................................end face" << i << endl;
    }
    cout << "......................................num=" << num << endl;
    Kx.setFromTriplets(Kx_IJV.begin(), Kx_IJV.end());
    Ky.setFromTriplets(Ky_IJV.begin(), Ky_IJV.end());
    Kz.setFromTriplets(Kz_IJV.begin(), Kz_IJV.end());
    for (int i = 0; i < n; i++)
    {
        cout << "v" << i << ": " << Kx.coeff(i, i) << ", " << Ky.coeff(i, i) << ", " << Kz.coeff(i, i) << endl;
    }

    SparseMatrix<double> Z(n, nr);
    SparseMatrix<double> ZZ(n, nr * 2);
    // std::cout << Kx.toDense() << std::endl;
    SparseMatrix<double> CSM =
        cat(1, cat(1, cat(2, Kx, ZZ), cat(2, cat(2, Z, Ky), Z)), cat(2, ZZ, Kz)).transpose();
    std::cout << "CSM" << std::endl;
    std::cout << CSM.toDense() << std::endl;
}

void TMesh::ArapDeformTest()
{
    using namespace Eigen;
    int n = number_of_vertices();
    int m = number_of_faces();
    std::vector<Vector3d> V, U; // V is oringal vertex position, U is deformed vertex position
    // calculate cotagent weight for each edge
    for (Hd hd : halfedges())
    {
        m_C.push_back(CotWeight(hd));
    }

    // Debug: ToDO remove U
    for (Vd vi : vertices())
    {
        const Pnt3 &p = point(vi);
        V.push_back(Vector3d(p.x(), p.y(), p.z()));
        U.push_back(Vector3d(p.x(), p.y(), p.z()));
    }
    U[0] = Vector3d(0, 0, 1);

    std::vector<Matrix3d> rot_mtr(n);
    // Local-step: find the best rotation matrix
    CovarianceScatterMatrixOneRing(V, U, rot_mtr);
}

void TMesh::CovarianceScatterMatrixOneRing(
    const std::vector<Eigen::Vector3d> &V,
    const std::vector<Eigen::Vector3d> &U,
    std::vector<Eigen::Matrix3d> &rot_mtr)
{
    // Local-step: find the best rotation matrix
    // Spokes and Rims
    using namespace Eigen;

    for (Vd vi : vertices())
    {
        // cout << "------------------" << vi << endl;
        MatrixXd si = Matrix3d::Zero();
        for (Hd hd : halfedges_around_target(halfedge(vi)))
        {
            Vd vj = source(hd); // spokes
            Vector3d eij = V[vi.idx()] - V[vj.idx()];
            Vector3d eij_new = U[vi.idx()] - U[vj.idx()];
            si += m_C[hd.idx()] * eij * eij_new.transpose();
            // cout << vi << "-" << vj << ": " << eij.transpose() << " wij=" << weight[hd.idx()] << " wji=" << weight[opposite(hd).idx()] << endl;
            if (!is_border(prev(hd))) // rims
            {
                Vd vk = source(prev(hd));
                Vector3d ejk = V[vj.idx()] - V[vk.idx()];
                Vector3d ejk_new = U[vj.idx()] - U[vk.idx()];
                si += m_C[prev(hd).idx()] * ejk * ejk_new.transpose();
                // cout << vj << "-" << vk << ": " << ejk.transpose() << " wjk=" << weight[prev(hd).idx()] << " wkj=" << weight[opposite(prev(hd)).idx()] << endl;
            }

            // cout << vj << "-" << vk << ": " << eij.transpose() << " wij=" << weight[hd.idx()] << endl;
        }
        JacobiSVD<Matrix3d> sol(si, Eigen::ComputeFullU | Eigen::ComputeFullV);
        rot_mtr[vi.idx()] = sol.matrixV() * sol.matrixU().transpose();
        // cout << "si:" << endl;
        // cout << si << endl;
        // cout << "-----------------" << endl;
    }
}

double TMesh::CotVal(Vd v0, Vd v1, Vd v2)
{
    Vec3 a = point(v0) - point(v1);
    Vec3 b = point(v2) - point(v1);

    double dot_ab = a.dot(b);
    Vec3 cross_ab = a.cross(b);
    double divider = cross_ab.dot(cross_ab);

    if (divider == 0)
    {
        assert(false);
    }
    return dot_ab / divider;
}

double TMesh::CotWeight(Hd he)
{

    Vd v0 = target(he);
    Vd v1 = source(he);

    if (is_border_edge(he))
    {
        Hd he_cw = opposite(next(he));
        Vd v2 = source(he_cw);
        if (is_border_edge(he_cw))
        {
            Hd he_ccw = prev(opposite(he));
            v2 = source(he_ccw);
        }
        return (CotVal(v0, v2, v1) / 2.0);
    }
    else
    {
        Hd he_cw = opposite(next(he));
        Vd v2 = source(he_cw);
        Hd he_ccw = prev(opposite(he));
        Vd v3 = source(he_ccw);

        return (CotVal(v0, v2, v1) / 2.0 + CotVal(v0, v3, v1) / 2.0);
    }
}

void TMesh::MarkLayeredVertex(const vector<vector<Pnt3>> &child_contours, int layerID)
{
    vector<Vd> bnd_inds;
    FindSketchBoundary(bnd_inds);
    for (const Vd &vd : bnd_inds)
    {
        for (int i = 0; i < child_contours.size(); i++)
        {
            if (IsPointInPolygon(point(vd), child_contours[i]))
            {
                m_layerV[vd.idx()] = layerID;
                m_lConstrain[vd.idx()] = LConstrainType::UpperConstrain;
            }
        }
    }
}

void TMesh::MarkLayeredVertex(const vector<Pnt3> &parent_contour, int layerID)
{
    vector<Vd> bnd_inds;
    FindSketchBoundary(bnd_inds);
    for (const Vd &vd : bnd_inds)
    {
        if (IsPointInPolygon(point(vd), parent_contour))
        {
            m_layerV[vd.idx()] = layerID;
            m_lConstrain[vd.idx()] = LConstrainType::LowerConstrain;
        }
    }
}

void GenSymMesh(const SymetryPlane sym_plane, const double plane_pos, const TMesh &oMesh, TMesh &mesh)
{
    if (sym_plane == SYM_XY_PLANE)
    {
        for (const Vd &vd : oMesh.vertices())
        {
            const Pnt3 &p = oMesh.point(vd);
            mesh.add_vertex(Pnt3(p.x(), p.y(), 2 * plane_pos - p.z()));
        }
    }
    else if (sym_plane == SYM_YZ_PLANE)
    {
        for (const Vd &vd : oMesh.vertices())
        {
            const Pnt3 &p = oMesh.point(vd);
            mesh.add_vertex(Pnt3(2 * plane_pos - p.x(), p.y(), p.z()));
        }
    }

    for (const Fd &fd : oMesh.faces())
    {
        const Hd &hd = oMesh.halfedge(fd);
        const Vd &v0 = oMesh.source(hd);
        const Vd &v1 = oMesh.target(hd);
        const Vd &v2 = oMesh.target(oMesh.next(hd));

        mesh.add_face(v2.idx(), v1.idx(), v0.idx());
    }
}

double TMesh::volume()
{
    // https://stackoverflow.com/questions/9866452/calculate-volume-of-any-tetrahedron-given-4-points
    Float sum = 0;
    for (const Fd &fd : faces())
    {
        const Hd &hd = halfedge(fd);
        const Pnt3 &a = point(source(hd));
        const Pnt3 &b = point(target(hd));
        const Pnt3 &c = point(target(next(hd)));
        sum += a.x() * b.y() * c.z() +
               a.z() * b.x() * c.y() +
               a.y() * c.x() * b.z() -
               c.x() * b.y() * a.z() -
               c.y() * b.z() * a.x() -
               c.z() * b.x() * a.y();
    }
    return sum;
}

double TMesh::one_ring_area(int vidx)
{
    Hd hd = halfedge(Vd(vidx));
    double a = 0;
    for (const Fd &fd : faces_around_target(hd))
    {
        a += Get3DFArea(fd);
    }
    return a;
}

double TMesh::one_ring_area(int vidx, const Eigen::MatrixXd &U)
{
    using namespace Eigen;
    Hd hd = halfedge(Vd(vidx));
    double a = 0;

    for (const Hd &adjHd : halfedges_around_target(hd))
    {
        if (!is_border(adjHd))
        {
            const RowVector3d &v0 = U.row(source(adjHd).idx());
            const RowVector3d &v1 = U.row(target(adjHd).idx());
            const RowVector3d &v2 = U.row(target(next(adjHd)).idx());

            double area = 0.25 * (((v1 - v0).cross(v2 - v0)).norm() + ((v1 - v2).cross(v0 - v2)).norm());
            if (!isnan(area))
            {
                a += area;
            }
        }
    }

    return a;
}

//-----------------ToDO: Utility for Eigen--------------------------------
static void MakeDiag(Eigen::SparseMatrix<double> &M, double val, int n)
{
    typedef Eigen::Triplet<double> T;
    vector<T> l;
    l.reserve(n);
    for (int i = 0; i < n; i++)
    {
        l.push_back(T(i, i, val));
    }
    M.resize(n, n);
    M.setFromTriplets(l.begin(), l.end());
}

static void FillDiag(Eigen::SparseMatrix<double> &M, const vector<double> &vals)
{
    using namespace Eigen;
    // M.setZero();
    for (int k = 0; k < M.outerSize(); k++)
        for (SparseMatrix<double>::InnerIterator it(M, k); it; ++it)
        {
            it.valueRef() = vals[k];
            // cout << k << " " << it.row() << ", " << it.col() << " " << it.value() << " " << vals[k] << " " << it.valueRef() << endl;
        }
}

#define DEBUG
void TMesh::Contract(MeshContractData &data)
{
    using namespace Eigen;
    int max_iter = data.max_iter;
    int n = number_of_vertices(); // num vertices

    MatrixXd U = GetVMat();
    RowVector3d bl_bbox = U.colwise().minCoeff(); // bottom left
    RowVector3d tr_bbox = U.colwise().maxCoeff(); // top right
    RowVector3d Ucent = (bl_bbox + tr_bbox) / 2.0;
    RowVector3d extent = tr_bbox - bl_bbox;
    double max_axis = extent.maxCoeff();
    U.rowwise() -= Ucent;
    U.array() /= max_axis;
    MatrixXd Uold = U;

    std::cout << "model orig size=" << extent << " cent=" << Ucent << std::endl;
    std::cout << "after normalized=" << U.colwise().maxCoeff() - U.colwise().minCoeff()
              << " cent=" << (U.colwise().maxCoeff() + U.colwise().minCoeff()) / 2 << std::endl;
    const MatrixXi &F = GetFMat();
    VectorXd dblA;

    m_adjV.resize(n);
    for (Vd vd : vertices())
    {
        Hd thd = halfedge(vd);
        for (Hd hd : halfedges_around_target(thd))
        {
            m_adjV[vd.idx()].push_back(target(hd).idx());
        }
    }

    int iter = 0;
    double originalVolume = Volume(U, F);
    double currentVolume = 0;
    igl::doublearea(U, F, dblA);
    double origMeshArea = 0.5 * dblA.sum();
    double meshArea = origMeshArea;
    double avgFaceArea = 0.5 * dblA.sum() / F.rows();
    MatrixXd BC;
    igl::barycenter(U, F, BC);
    RowVector3d centroid(0, 0, 0);
    for (int i = 0; i < BC.rows(); i++)
    {
        centroid += 0.5 * dblA(i) / meshArea * BC.row(i);
    }

    SparseMatrix<double> L, WH;
    double WL;
    data.WL0 = 1 / (10 * sqrt(avgFaceArea));
    WL = data.WL0;
    cout << "avgArea=" << avgFaceArea << " WL=" << WL
         << " original volume=" << originalVolume
         << " centroid=" << centroid
         << " origMeshArea=" << origMeshArea << endl;
    vector<double> origOneRingArea(n, 0), changeOneRingArea(n, 0);
    for (int i = 0; i < n; i++)
    {
        origOneRingArea[i] = one_ring_area(i, U);
        changeOneRingArea[i] = 1.0;
    }
    MakeDiag(WH, data.WH0, n);
    // SparseQR<SparseMatrix<double>, COLAMDOrdering<int>> solver;
    // SimplicialLDLT<SparseMatrix<double>> ldlt;
    // https://stackoverflow.com/questions/46014719/eigen-lscg-solver-performance-issue
    // https://forum.kde.org/viewtopic.php?f=74&t=107796
    SimplicialCholesky<SparseMatrix<double>> cholesky; // since it is a rectangular matrix, solve it in least-square fashion
    do
    {
        // build system matrix
        igl::cotmatrix(U, F, L);
        // M_DEBUG << "L:\n"
        //         << L << endl;

#ifdef DEBUG
        double maxCoeff = std::numeric_limits<double>::min();
        double minCoeff = std::numeric_limits<double>::max();
#endif
        std::vector<double> tot(n, 0);
        for (int k = 0; k < L.outerSize(); k++)
        {
            for (SparseMatrix<double>::InnerIterator it(L, k); it; ++it)
            {
                int i = it.row();
                int j = it.col();
                if (i != j)
                    tot[i] += it.value();
            }
        }

        std::set<int> degenerate_sets;
        for (int k = 0; k < L.outerSize(); k++)
        {
            for (SparseMatrix<double>::InnerIterator it(L, k); it; ++it)
            {
                int i = it.row();
                int j = it.col();

                if (tot[i] > 10000)
                {
                    // callapsed[i] = true
                    it.valueRef() /= (tot[i] / 10000.0);
                    //cout << "tot" << i << "=" << tot[i] << endl;
                    degenerate_sets.insert(i);
                }
                it.valueRef() *= WL;
                if (std::isnan(it.value()) || std::isinf(it.value()))
                {
                    cout << __FILE__ << " " << __LINE__ << " vertex" << i << " degenerate! isInf=" << std::isinf(it.value())
                         << " isnan=" << std::isnan(it.value()) << " m_adjV[i].size()=" << m_adjV[i].size() << endl;
                    degenerate_sets.insert(i);
                    if (i == j)
                        it.valueRef() = -1 * WL;
                    else
                        it.valueRef() = 1.0 / (double)m_adjV[i].size() * WL;
                }
#ifdef DEBUG
                else
                {

                    if (it.value() < minCoeff)
                        minCoeff = it.value();
                    if (it.value() > maxCoeff)
                        maxCoeff = it.value();
                }
#endif
            }
        }
#ifdef DEBUG
        cout << __FILE__ << " " << __LINE__ << " iter" << iter << " WL=" << WL
             << " minCoeff=" << minCoeff << " maxCoeff=" << maxCoeff
             << " degenerate" << degenerate_sets.size() << endl;
#endif

        SparseMatrix<double> A, Q;
        // L_iter = WL * L;
        igl::cat(1, L, WH, A);
        Q = A.transpose() * A;
        //cout << "iter" << iter << " A=" << A.rows() << ", " << A.cols() << " WL=" << WL << " A.nonZeros=" << A.nonZeros() << endl;

        // lhs
        MatrixXd b, zeros(n, 3);
        zeros.setZero();
        MatrixXd attrU = WH * U;
        igl::cat(1, zeros, attrU, b);
        b = A.transpose() * b;

        cholesky.compute(Q);
        if (cholesky.info() != Success)
        {
            cout << __FILE__ << " " << __LINE__ << " decomposition failed!" << endl;
            return;
        }
        cout << "solve system matrix success!" << endl;
        U = cholesky.solve(b);
        for (int vidx : degenerate_sets)
        {
            U.row(vidx) = Uold.row(vidx);
        }
        Uold = U;
        // update contraction & attraction weight
        WL *= data.SL;
        if (WL > 2048)
        {
            WL = 2048;
        }
        for (int i = 0; i < n; i++)
        {
            double curA = one_ring_area(i, U);
            double areaRatio = abs(curA / origOneRingArea[i]);
            double posWeight = 1.0 / sqrt(areaRatio); // position constraint weight, smaller area should retain their current position more strongly
            if (i == 0)
            {
                cout << "posWeight=" << posWeight << endl;
            }
            if (!isnan(posWeight) && !isinf(posWeight))
            {
                changeOneRingArea[i] = data.WH0 * posWeight;
            }

            if (posWeight > 10000)
            {
                changeOneRingArea[i] = 10000;
            }
        }
        FillDiag(WH, changeOneRingArea);

        igl::doublearea(U, F, dblA);
        meshArea = 0.5 * dblA.sum();
        if (meshArea > 1e-6)
        {
            igl::barycenter(U, F, BC);
            centroid.setZero();
            for (int i = 0; i < BC.rows(); i++)
            {
                centroid += 0.5 * dblA(i) / meshArea * BC.row(i);
            }
        }

        currentVolume = Volume(U, F);
        cout << "Iter" << iter
             << " volumeRatio=" << currentVolume / originalVolume
             << " meshArea=" << meshArea
             << " areaRatio=" << meshArea / origMeshArea
             << " centroid=" << centroid << endl;
        igl::writeOBJ("output/skel" + to_string(iter) + ".obj", U, F);
        iter++;

        // } while (iter < max_iter && (currentVolume / originalVolume > 1e-4) && (meshArea / origMeshArea) > 1e-1);
    } while (iter < max_iter && (meshArea / origMeshArea) > 1e-2);

    igl::writeOBJ("output/skel.obj", U, F);
}

void TMesh::InitTMeshTexPaint(int texWidth, int blockWidth, int colorIdx)
{
    int fSize = number_of_faces();

    //set texture image size
    int blockSize = texWidth / blockWidth;
    m_texPaint.m_texWidth = texWidth;
    m_texPaint.m_blockWidth = blockWidth;
    m_texPaint.m_blockSize = blockSize;

    CPoint blockPos((colorIdx % (blockSize / 3)) * blockWidth * 3, (colorIdx / (blockSize / 3)) * blockWidth);
    m_texPaint.InitTexPaint(fSize, blockPos);

    UpdatePaintFaceInfo();
}

void TMesh::UpdatePaintFaceInfo()
{
    for (const Fd &fd : faces())
    {
        int i = fd.idx();
        FaceInfo &faceInfo = m_texPaint.m_vFaceInfo[i];

        const Hd &hd = halfedge(fd);
        const Vd &v0 = source(hd);
        const Vd &v1 = target(hd);
        const Vd &v2 = target(next(hd));
        Pnt3 p0 = point(v0);
        Pnt3 p1 = point(v1);
        Pnt3 p2 = point(v2);

        faceInfo.p[0] = p0;
        faceInfo.p[1] = p1;
        faceInfo.p[2] = p2;
        faceInfo.p[3] = (faceInfo.p[0] + faceInfo.p[1]) / 2.0;
        faceInfo.p[4] = (faceInfo.p[1] + faceInfo.p[2]) / 2.0;
        faceInfo.p[5] = (faceInfo.p[2] + faceInfo.p[0]) / 2.0;
        faceInfo.p[6] = (faceInfo.p[0] + faceInfo.p[1] + faceInfo.p[2]) / 3.0;

        faceInfo.norm[0] = m_vNormals[v0];
        faceInfo.norm[1] = m_vNormals[v1];
        faceInfo.norm[2] = m_vNormals[v2];

        if (face(opposite(hd)) != null_face())
        {
            faceInfo.norm[3] = (faceInfo.norm[0] + faceInfo.norm[1]).normalized();
        }
        else
        {
            faceInfo.norm[3] = m_fNormals[fd];
        }

        if (face(opposite(next(hd))) != null_face())
        {
            faceInfo.norm[4] = (faceInfo.norm[1] + faceInfo.norm[2]).normalized();
        }
        else
        {
            faceInfo.norm[4] = m_fNormals[fd];
        }

        if (face(opposite(next(next(hd)))) != null_face())
        {
            faceInfo.norm[5] = (faceInfo.norm[2] + faceInfo.norm[0]).normalized();
        }
        else
        {
            faceInfo.norm[5] = m_fNormals[fd];
        }

        faceInfo.norm[6] = (faceInfo.norm[3] + faceInfo.norm[4] + faceInfo.norm[5]).normalized();
    }
}

DrawFace TMesh::GetOriDrawFace(const Pnt3 &pt3d, int fid)
{
    Fd fd = Fd(fid);
    FaceInfo &faceInfo = m_texPaint.m_vFaceInfo[fid];
    Vec3 fNorm = GetFNormal(fd);

    int i;
    Pnt3 pos3d[3], norm[3], vec;
    int triangleIdx = -1, blockIdx = -1;
    Pnt3 *p = faceInfo.p;

    for (i = 0; i < 3; i++)
    {
        norm[i] = (p[i] - p[6]).cross(fNorm).normalized();
    }

    //check pt3d in which triangle
    vec = pt3d - p[6];

    if (vec.dot(norm[2]) <= 0)
    {
        if (vec.dot(norm[1]) >= 0)
        {
            if (vec.dot(norm[0]) <= 0)
            {
                triangleIdx = 0;
                pos3d[0] = p[0];
                pos3d[1] = p[3];
                pos3d[2] = p[6];
            }
            else
            {
                triangleIdx = 1;
                pos3d[0] = p[6];
                pos3d[1] = p[5];
                pos3d[2] = p[0];
            }
        }
        else
        {
            triangleIdx = 4;
            pos3d[0] = p[2];
            pos3d[1] = p[5];
            pos3d[2] = p[6];
        }
    }
    else
    {
        if (vec.dot(norm[0]) <= 0)
        {
            if (vec.dot(norm[1]) <= 0)
            {
                triangleIdx = 2;
                pos3d[0] = p[1];
                pos3d[1] = p[4];
                pos3d[2] = p[6];
            }
            else
            {
                triangleIdx = 3;
                pos3d[0] = p[6];
                pos3d[1] = p[3];
                pos3d[2] = p[1];
            }
        }
        else
        {
            triangleIdx = 5;
            pos3d[0] = p[6];
            pos3d[1] = p[4];
            pos3d[2] = p[2];
        }
    }

    blockIdx = triangleIdx / 2;

    //get st of pt3d in triangle
    Pnt2 st = GetPtstOfTriangle(pt3d, pos3d[0], pos3d[1], pos3d[2]);
    if (triangleIdx % 2 == 0)
    {
        st.y() = 1 - st.y();
    }
    else
    {
        st.x() = 1 - st.x();
    }

    int blockWidth = m_texPaint.m_blockWidth;
    CPoint seed = CPoint(st.x() * blockWidth, st.y() * blockWidth);

    //draw around line segment
    DrawFace drawFace;
    drawFace.fid = fid;
    for (i = 0; i < 3; i++)
    {
        DrawBlock &drawBlock = drawFace.block[i];
        drawBlock.drawFlag = false;
        drawBlock.seeds.clear();
        drawBlock.pixelFlag.resize(blockWidth * blockWidth, false);
    }

    drawFace.block[blockIdx].seeds.push_back(seed);
    drawFace.block[blockIdx].pixelFlag[seed.x() + seed.y() * blockWidth] = true;

    return drawFace;
}

void TMesh::SearchAdjBlock(int blockIdx, int drawFidx, vector<DrawFace> &drawFaces, vector<bool> &vDrawFaceFlag)
{
    // int i, j;
    // int blockWidth = m_texPaint.m_blockWidth;
    // DrawFace drawFace = drawFaces[drawFidx];
    // int fid = drawFace.fid;
    // cvector<bool> &pixelFlag = drawFace.block[blockIdx].pixelFlag;
    // Fd fd(fid);

    // DrawBlock *pDrawBlock;

    // //(0,0)
    // if (pixelFlag[0])
    // {
    //     TVert &vert = GetV(face.p(blockIdx));
    //     int adjFSize = vert.GetAdjFNum();

    //     for (i = 0; i < adjFSize; i++)
    //     {
    //         pDrawBlock = NULL;0

    //         int fidTmp = vert.AdjF(i);
    //         if (fidTmp > -1 && fidTmp != fid && !vDrawFaceFlag[fidTmp])
    //         {
    //             int idx = FindDrawFace(fidTmp, drawFaces);
    //             assert(idx == -1 || idx > drawFidx);

    //             if (idx == -1)
    //             {
    //                 DrawFace drawFaceTmp;
    //                 drawFaceTmp.fid = fidTmp;

    //                 for (j = 0; j < 3; j++)
    //                 {
    //                     DrawBlock &drawBlock = drawFaceTmp.block[j];
    //                     drawBlock.drawFlag = false;
    //                     drawBlock.seeds.clear();
    //                     drawBlock.pixelFlag.resize(blockWidth * blockWidth, false);
    //                 }

    //                 drawFaces.push_back(drawFaceTmp);
    //                 idx = (int)(drawFaces.size() - 1);
    //             }

    //             TFace &faceTmp = GetF(fidTmp);
    //             DrawFace &drawFaceTmp = drawFaces[idx];

    //             idx = -1;
    //             for (j = 0; j < 3; j++)
    //             {
    //                 if (faceTmp.p(j) == face.p(blockIdx))
    //                 {
    //                     idx = j;
    //                     break;
    //                 }
    //             }

    //             pDrawBlock = &(drawFaceTmp.block[idx]);

    //             if (!pDrawBlock->pixelFlag[0])
    //             {
    //                 pDrawBlock->seeds.push_back(CPoint(0, 0));
    //                 pDrawBlock->pixelFlag[0] = true;
    //             }
    //         }
    //     }
    // }

    // //(0, blockWidth-1)
    // if (pixelFlag[(blockWidth - 1) * blockWidth])
    // {
    //     //adjacent block
    //     pDrawBlock = NULL;
    //     pDrawBlock = &(drawFace.block[(blockIdx + 1) % 3]);

    //     if (!pDrawBlock->pixelFlag[blockWidth - 1])
    //     {
    //         pDrawBlock->seeds.push_back(CPoint(blockWidth - 1, 0));
    //         pDrawBlock->pixelFlag[blockWidth - 1] = true;
    //     }

    //     //adjacent face
    //     int fidTmp = GetAdjFid(fid, face.GetEdgeRef(blockIdx));
    //     if (fidTmp > -1 && !vDrawFaceFlag[fidTmp])
    //     {
    //         int idx = FindDrawFace(fidTmp, drawFaces);
    //         assert(idx == -1 || idx > drawFidx);

    //         if (idx == -1)
    //         {
    //             DrawFace drawFaceTmp;
    //             drawFaceTmp.fid = fidTmp;

    //             for (i = 0; i < 3; i++)
    //             {
    //                 DrawBlock &drawBlock = drawFaceTmp.block[i];
    //                 drawBlock.drawFlag = false;
    //                 drawBlock.seeds.clear();
    //                 drawBlock.pixelFlag.resize(blockWidth * blockWidth, false);
    //             }

    //             drawFaces.push_back(drawFaceTmp);
    //             idx = (int)(drawFaces.size() - 1);
    //         }

    //         TFace &faceTmp = GetF(fidTmp);
    //         DrawFace &drawFaceTmp = drawFaces[idx];

    //         idx = -1;
    //         for (j = 0; j < 3; j++)
    //         {
    //             if (faceTmp.p(j) == face.p(blockIdx))
    //             {
    //                 idx = j;
    //                 break;
    //             }
    //         }

    //         pDrawBlock = NULL;
    //         pDrawBlock = &(drawFaceTmp.block[idx]);

    //         if (!pDrawBlock->pixelFlag[blockWidth - 1])
    //         {
    //             pDrawBlock->seeds.push_back(CPoint(blockWidth - 1, 0));
    //             pDrawBlock->pixelFlag[blockWidth - 1] = true;
    //         }

    //         pDrawBlock = NULL;
    //         pDrawBlock = &(drawFaceTmp.block[(idx + 2) % 3]);

    //         if (!pDrawBlock->pixelFlag[(blockWidth - 1) * blockWidth])
    //         {
    //             pDrawBlock->seeds.push_back(CPoint(0, blockWidth - 1));
    //             pDrawBlock->pixelFlag[(blockWidth - 1) * blockWidth] = true;
    //         }
    //     }
    // }

    // //(blockWidth-1, 0)
    // if (pixelFlag[blockWidth - 1])
    // {
    //     //adjacent block
    //     pDrawBlock = NULL;
    //     pDrawBlock = &(drawFace.block[(blockIdx + 2) % 3]);

    //     if (!pDrawBlock->pixelFlag[(blockWidth - 1) * blockWidth])
    //     {
    //         pDrawBlock->seeds.push_back(CPoint(0, blockWidth - 1));
    //         pDrawBlock->pixelFlag[(blockWidth - 1) * blockWidth] = true;
    //     }

    //     //adjacent face
    //     int fidTmp = GetAdjFid(fid, face.GetEdgeRef((blockIdx + 2) % 3));
    //     if (fidTmp > -1 && !vDrawFaceFlag[fidTmp])
    //     {
    //         int idx = FindDrawFace(fidTmp, drawFaces);
    //         assert(idx == -1 || idx > drawFidx);

    //         if (idx == -1)
    //         {
    //             DrawFace drawFaceTmp;
    //             drawFaceTmp.fid = fidTmp;

    //             for (i = 0; i < 3; i++)
    //             {
    //                 DrawBlock &drawBlock = drawFaceTmp.block[i];
    //                 drawBlock.drawFlag = false;
    //                 drawBlock.seeds.clear();
    //                 drawBlock.pixelFlag.resize(blockWidth * blockWidth, false);
    //             }

    //             drawFaces.push_back(drawFaceTmp);
    //             idx = (int)(drawFaces.size() - 1);
    //         }

    //         TFace &faceTmp = GetF(fidTmp);
    //         DrawFace &drawFaceTmp = drawFaces[idx];

    //         idx = -1;
    //         for (j = 0; j < 3; j++)
    //         {
    //             if (faceTmp.p(j) == face.p(blockIdx))
    //             {
    //                 idx = j;
    //                 break;
    //             }
    //         }

    //         pDrawBlock = NULL;
    //         pDrawBlock = &(drawFaceTmp.block[idx]);

    //         if (!pDrawBlock->pixelFlag[(blockWidth - 1) * blockWidth])
    //         {
    //             pDrawBlock->seeds.push_back(CPoint(0, blockWidth - 1));
    //             pDrawBlock->pixelFlag[(blockWidth - 1) * blockWidth] = true;
    //         }

    //         pDrawBlock = NULL;
    //         pDrawBlock = &(drawFaceTmp.block[(idx + 1) % 3]);

    //         if (!pDrawBlock->pixelFlag[blockWidth - 1])
    //         {
    //             pDrawBlock->seeds.push_back(CPoint(blockWidth - 1, 0));
    //             pDrawBlock->pixelFlag[blockWidth - 1] = true;
    //         }
    //     }
    // }

    // //(blockWidth-1, blockWidth-1)
    // if (pixelFlag[blockWidth * blockWidth - 1])
    // {
    //     //adjacent block
    //     pDrawBlock = NULL;
    //     pDrawBlock = &(drawFace.block[(blockIdx + 1) % 3]);

    //     if (!pDrawBlock->pixelFlag[blockWidth * blockWidth - 1])
    //     {
    //         pDrawBlock->seeds.push_back(CPoint(blockWidth - 1, blockWidth - 1));
    //         pDrawBlock->pixelFlag[blockWidth * blockWidth - 1] = true;
    //     }

    //     pDrawBlock = NULL;
    //     pDrawBlock = &(drawFace.block[(blockIdx + 2) % 3]);

    //     if (!pDrawBlock->pixelFlag[blockWidth * blockWidth - 1])
    //     {
    //         pDrawBlock->seeds.push_back(CPoint(blockWidth - 1, blockWidth - 1));
    //         pDrawBlock->pixelFlag[blockWidth * blockWidth - 1] = true;
    //     }
    // }

    // //(0, 1) ~ (0, blockWidth-2)
    // pDrawBlock = NULL;
    // for (i = 1; i < blockWidth - 1; i++)
    // {
    //     if (pixelFlag[i * blockWidth])
    //     {
    //         int fidTmp = GetAdjFid(fid, face.GetEdgeRef(blockIdx));
    //         if (fidTmp > -1 && !vDrawFaceFlag[fidTmp])
    //         {
    //             int idx = FindDrawFace(fidTmp, drawFaces);
    //             assert(idx == -1 || idx > drawFidx);

    //             if (idx == -1)
    //             {
    //                 DrawFace drawFaceTmp;
    //                 drawFaceTmp.fid = fidTmp;

    //                 for (j = 0; j < 3; j++)
    //                 {
    //                     DrawBlock &drawBlock = drawFaceTmp.block[j];
    //                     drawBlock.drawFlag = false;
    //                     drawBlock.seeds.clear();
    //                     drawBlock.pixelFlag.resize(blockWidth * blockWidth, false);
    //                 }

    //                 drawFaces.push_back(drawFaceTmp);
    //                 idx = (int)(drawFaces.size() - 1);
    //             }

    //             TFace &faceTmp = GetF(fidTmp);
    //             DrawFace &drawFaceTmp = drawFaces[idx];

    //             idx = -1;
    //             for (j = 0; j < 3; j++)
    //             {
    //                 if (faceTmp.p(j) == face.p(blockIdx))
    //                 {
    //                     idx = j;
    //                     break;
    //                 }
    //             }

    //             pDrawBlock = &(drawFaceTmp.block[idx]);
    //             break;
    //         }
    //     }
    // }

    // if (pDrawBlock)
    // {
    //     for (i = 1; i < blockWidth - 1; i++)
    //     {
    //         if (pixelFlag[i * blockWidth])
    //         {
    //             if (!pDrawBlock->pixelFlag[i])
    //             {
    //                 pDrawBlock->seeds.push_back(CPoint(i, 0));
    //                 pDrawBlock->pixelFlag[i] = true;
    //             }
    //         }
    //     }
    // }

    // //(1, 0) ~ (blockWidth-2,0)
    // pDrawBlock = NULL;
    // for (i = 1; i < blockWidth - 1; i++)
    // {
    //     if (pixelFlag[i])
    //     {
    //         int fidTmp = GetAdjFid(fid, face.GetEdgeRef((blockIdx + 2) % 3));
    //         if (fidTmp > -1 && !vDrawFaceFlag[fidTmp])
    //         {
    //             int idx = FindDrawFace(fidTmp, drawFaces);
    //             assert(idx == -1 || idx > drawFidx);

    //             if (idx == -1)
    //             {
    //                 DrawFace drawFaceTmp;
    //                 drawFaceTmp.fid = fidTmp;

    //                 for (j = 0; j < 3; j++)
    //                 {
    //                     DrawBlock &drawBlock = drawFaceTmp.block[j];
    //                     drawBlock.drawFlag = false;
    //                     drawBlock.seeds.clear();
    //                     drawBlock.pixelFlag.resize(blockWidth * blockWidth, false);
    //                 }

    //                 drawFaces.push_back(drawFaceTmp);
    //                 idx = (int)(drawFaces.size() - 1);
    //             }

    //             TFace &faceTmp = GetF(fidTmp);
    //             DrawFace &drawFaceTmp = drawFaces[idx];

    //             idx = -1;
    //             for (j = 0; j < 3; j++)
    //             {
    //                 if (faceTmp.p(j) == face.p(blockIdx))
    //                 {
    //                     idx = j;
    //                     break;
    //                 }
    //             }

    //             pDrawBlock = &(drawFaceTmp.block[idx]);
    //             break;
    //         }
    //     }
    // }

    // if (pDrawBlock)
    // {
    //     for (i = 1; i < blockWidth - 1; i++)
    //     {
    //         if (pixelFlag[i])
    //         {
    //             if (!pDrawBlock->pixelFlag[i * blockWidth])
    //             {
    //                 pDrawBlock->seeds.push_back(CPoint(0, i));
    //                 pDrawBlock->pixelFlag[i * blockWidth] = true;
    //             }
    //         }
    //     }
    // }

    // //(1, blockWidth-1) ~ (blockWidth-2, blockWidth-1)
    // pDrawBlock = NULL;
    // for (i = 1; i < blockWidth - 1; i++)
    // {
    //     if (pixelFlag[blockWidth * (blockWidth - 1) + i])
    //     {
    //         pDrawBlock = &(drawFace.block[(blockIdx + 1) % 3]);
    //         break;
    //     }
    // }

    // if (pDrawBlock)
    // {
    //     for (i = 1; i < blockWidth - 1; i++)
    //     {
    //         if (pixelFlag[blockWidth * (blockWidth - 1) + i])
    //         {
    //             if (!pDrawBlock->pixelFlag[i * blockWidth + blockWidth - 1])
    //             {
    //                 pDrawBlock->seeds.push_back(CPoint(blockWidth - 1, i));
    //                 pDrawBlock->pixelFlag[i * blockWidth + blockWidth - 1] = true;
    //             }
    //         }
    //     }
    // }

    // //(blockWidth-1, 1)  ~ (blockWidth-1, blockWidth-2)
    // pDrawBlock = NULL;
    // for (i = 1; i < blockWidth - 1; i++)
    // {
    //     if (pixelFlag[i * blockWidth + blockWidth - 1])
    //     {
    //         pDrawBlock = &(drawFace.block[(blockIdx + 2) % 3]);
    //         break;
    //     }
    // }

    // if (pDrawBlock)
    // {
    //     for (i = 1; i < blockWidth - 1; i++)
    //     {
    //         if (pixelFlag[i * blockWidth + blockWidth - 1])
    //         {
    //             if (!pDrawBlock->pixelFlag[(blockWidth - 1) * blockWidth + i])
    //             {
    //                 pDrawBlock->seeds.push_back(CPoint(i, blockWidth - 1));
    //                 pDrawBlock->pixelFlag[(blockWidth - 1) * blockWidth + i] = true;
    //             }
    //         }
    //     }
    // }

    // drawFaces[drawFidx] = drawFace; //update it (because other two blocks are changed)
}

void TMesh::TexPaintDrawPoint(Pnt3 pt3d, int drawFidx, vector<DrawFace> &drawFaces, vector<bool> &vDrawFaceFlag)
{
    // int i, j, k;
    // CPoint point;
    // int position;

    // int texWidth = m_texPaint.m_texWidth;
    // int blockWidth = m_texPaint.m_blockWidth;
    // Float radius3d = m_texPaint.m_painterWidth;

    // //get face to be drawn
    // int fid = drawFaces[drawFidx].fid;

    // FaceInfo &faceInfo = m_texPaint.m_vFaceInfo[fid];
    // Pnt3 *p = faceInfo.p;

    // TFace &face = GetF(fid);

    // //draw face for several case
    // Pnt3 pt3dTmp, interPt;
    // Float distTmp;
    // Pnt2 st;

    // cvector<bool> vertDrawFlag;
    // vertDrawFlag.resize(3, false);

    // //case one: test face
    // for (i = 0; i < 3; i++)
    // {
    //     distTmp = (pt3d - p[i]).norm();
    //     if (distTmp < radius3d)
    //         vertDrawFlag[i] = true;
    // }

    // //draw full face
    // if (vertDrawFlag[0] && vertDrawFlag[1] && vertDrawFlag[2])
    // {
    //     for (i = 0; i < 3; i++)
    //     {
    //         DrawBlock &drawBlock = drawFaces[drawFidx].block[i];

    //         for (j = 0; j < blockWidth; j++)
    //         {
    //             for (k = 0; k < blockWidth; k++)
    //             {
    //                 position = j + k * blockWidth;
    //                 drawBlock.pixelFlag[position] = true;
    //             }
    //         }

    //         drawBlock.drawFlag = true;
    //         SearchAdjBlock(i, drawFidx, drawFaces, vDrawFaceFlag);
    //     }

    //     vDrawFaceFlag[fid] = true;
    //     return;
    // }

    // //case two: test each of 3 blocks
    // Pnt3 vertBlock[4];
    // for (i = 0; i < 3; i++)
    // {
    //     vertDrawFlag.clear();
    //     vertDrawFlag.resize(4, false);

    //     if (i == 0) //blockIdx == 0
    //     {
    //         vertBlock[0] = p[0];
    //         vertBlock[1] = p[3];
    //         vertBlock[2] = p[6];
    //         vertBlock[3] = p[5];
    //     }
    //     else if (i == 1) //blockIdx == 1
    //     {
    //         vertBlock[0] = p[1];
    //         vertBlock[1] = p[4];
    //         vertBlock[2] = p[6];
    //         vertBlock[3] = p[3];
    //     }
    //     else //blockIdx == 2
    //     {
    //         vertBlock[0] = p[2];
    //         vertBlock[1] = p[5];
    //         vertBlock[2] = p[6];
    //         vertBlock[3] = p[4];
    //     }

    //     for (j = 0; j < 4; j++)
    //     {
    //         distTmp = (pt3d - vertBlock[j]).norm();
    //         if (distTmp < radius3d)
    //             vertDrawFlag[j] = true;
    //     }

    //     DrawBlock &drawBlock = drawFaces[drawFidx].block[i];

    //     if (vertDrawFlag[0] && vertDrawFlag[1] && vertDrawFlag[2] && vertDrawFlag[3]) //draw full block
    //     {
    //         for (j = 0; j < blockWidth; j++)
    //         {
    //             for (k = 0; k < blockWidth; k++)
    //             {
    //                 position = j + k * blockWidth;
    //                 drawBlock.pixelFlag[position] = true;
    //             }
    //         }

    //         drawBlock.drawFlag = true;
    //         SearchAdjBlock(i, drawFidx, drawFaces, vDrawFaceFlag);
    //     }
    //     else
    //     {
    //         if (vertDrawFlag[0] && vertDrawFlag[1] && vertDrawFlag[2]) //draw half block
    //         {
    //             for (j = 0; j < blockWidth; j++)
    //             {
    //                 for (k = 0; k < blockWidth; k++)
    //                 {
    //                     if (j <= k)
    //                     {
    //                         position = j + k * blockWidth;
    //                         drawBlock.pixelFlag[position] = true;
    //                     }
    //                 }
    //             }

    //             for (j = 0; j < blockWidth; j++)
    //             {
    //                 drawBlock.seeds.push_back(CPoint(j, j));
    //             }
    //         }
    //         else if (vertDrawFlag[2] && vertDrawFlag[3] && vertDrawFlag[0]) //draw half block
    //         {
    //             for (j = 0; j < blockWidth; j++)
    //             {
    //                 for (k = 0; k < blockWidth; k++)
    //                 {
    //                     if (j >= k)
    //                     {
    //                         position = j + k * blockWidth;
    //                         drawBlock.pixelFlag[position] = true;
    //                     }
    //                 }
    //             }

    //             for (j = 0; j < blockWidth; j++)
    //             {
    //                 drawBlock.seeds.push_back(CPoint(j, j));
    //             }
    //         }
    //         else if (vertDrawFlag[0] && vertDrawFlag[1]) //draw left edge
    //         {
    //             for (j = 0; j < blockWidth; j++)
    //             {
    //                 position = j * blockWidth;
    //                 drawBlock.pixelFlag[position] = true;
    //                 drawBlock.seeds.push_back(CPoint(0, j));
    //             }
    //         }
    //         else if (vertDrawFlag[2] && vertDrawFlag[3]) //draw right edge
    //         {
    //             for (j = 0; j < blockWidth; j++)
    //             {
    //                 position = blockWidth - 1 + j * blockWidth;
    //                 drawBlock.pixelFlag[position] = true;
    //                 drawBlock.seeds.push_back(CPoint(blockWidth - 1, j));
    //             }
    //         }
    //         else if (vertDrawFlag[0] && vertDrawFlag[3]) //draw up edge
    //         {
    //             for (j = 0; j < blockWidth; j++)
    //             {
    //                 position = j;
    //                 drawBlock.pixelFlag[position] = true;
    //                 drawBlock.seeds.push_back(CPoint(j, 0));
    //             }
    //         }
    //         else if (vertDrawFlag[1] && vertDrawFlag[2]) //draw down edge
    //         {
    //             for (j = 0; j < blockWidth; j++)
    //             {
    //                 position = j + (blockWidth - 1) * blockWidth;
    //                 drawBlock.pixelFlag[position] = true;
    //                 drawBlock.seeds.push_back(CPoint(j, blockWidth - 1));
    //             }
    //         }
    //         else if (vertDrawFlag[0] && vertDrawFlag[2]) //draw diagonal
    //         {
    //             for (j = 0; j < blockWidth; j++)
    //             {
    //                 position = j + j * blockWidth;
    //                 drawBlock.pixelFlag[position] = true;
    //                 drawBlock.seeds.push_back(CPoint(j, j));
    //             }
    //         }

    //         //draw by search
    //         if (drawBlock.seeds.size() > 0)
    //         {
    //             DrawBlockBySearch(pt3d, drawBlock, vertBlock);
    //             drawBlock.drawFlag = true;
    //             SearchAdjBlock(i, drawFidx, drawFaces, vDrawFaceFlag);
    //         }
    //     }
    // }

    // ////up to now, at least one block has drawn
    // int blockIdx1 = -1, blockIdx2 = -1, blockIdx3 = -1;
    // for (i = 0; i < 3; i++)
    // {
    //     if (!drawFaces[drawFidx].block[i].drawFlag)
    //     {
    //         blockIdx1 = i;
    //         break;
    //     }
    // }

    // if (blockIdx1 > -1) //not all three blocks are drawn
    // {
    //     for (i = 0; i < 3; i++)
    //     {
    //         if (drawFaces[drawFidx].block[i].drawFlag)
    //         {
    //             blockIdx2 = i;
    //             break;
    //         }
    //     }

    //     assert(blockIdx2 > -1);
    //     blockIdx3 = 3 - blockIdx1 - blockIdx2;

    //     if (drawFaces[drawFidx].block[blockIdx3].drawFlag) //draw only block[blockIdx1]
    //     {
    //         DrawBlock &drawBlock = drawFaces[drawFidx].block[blockIdx1];
    //         if (drawBlock.seeds.size() > 0)
    //         {
    //             if (blockIdx1 == 0) //blockIdx == 0
    //             {
    //                 vertBlock[0] = p[0];
    //                 vertBlock[1] = p[3];
    //                 vertBlock[2] = p[6];
    //                 vertBlock[3] = p[5];
    //             }
    //             else if (blockIdx1 == 1) //blockIdx == 1
    //             {
    //                 vertBlock[0] = p[1];
    //                 vertBlock[1] = p[4];
    //                 vertBlock[2] = p[6];
    //                 vertBlock[3] = p[3];
    //             }
    //             else //blockIdx == 2
    //             {
    //                 vertBlock[0] = p[2];
    //                 vertBlock[1] = p[5];
    //                 vertBlock[2] = p[6];
    //                 vertBlock[3] = p[4];
    //             }

    //             DrawBlockBySearch(pt3d, drawBlock, vertBlock);
    //             drawBlock.drawFlag = true;
    //             SearchAdjBlock(blockIdx1, drawFidx, drawFaces, vDrawFaceFlag);
    //         }
    //     }
    //     else //draw block[blockIdx1] and block[blockIdx3]
    //     {
    //         if (drawFaces[drawFidx].block[blockIdx1].seeds.size() > 0) //draw block[blockIdx1] first
    //         {
    //             DrawBlock &drawBlock = drawFaces[drawFidx].block[blockIdx1];
    //             if (blockIdx1 == 0) //blockIdx == 0
    //             {
    //                 vertBlock[0] = p[0];
    //                 vertBlock[1] = p[3];
    //                 vertBlock[2] = p[6];
    //                 vertBlock[3] = p[5];
    //             }
    //             else if (blockIdx1 == 1) //blockIdx == 1
    //             {
    //                 vertBlock[0] = p[1];
    //                 vertBlock[1] = p[4];
    //                 vertBlock[2] = p[6];
    //                 vertBlock[3] = p[3];
    //             }
    //             else //blockIdx == 2
    //             {
    //                 vertBlock[0] = p[2];
    //                 vertBlock[1] = p[5];
    //                 vertBlock[2] = p[6];
    //                 vertBlock[3] = p[4];
    //             }

    //             DrawBlockBySearch(pt3d, drawBlock, vertBlock);
    //             drawBlock.drawFlag = true;
    //             SearchAdjBlock(blockIdx1, drawFidx, drawFaces, vDrawFaceFlag);

    //             if (drawFaces[drawFidx].block[blockIdx3].seeds.size() > 0) //draw block[blockIdx3] next
    //             {
    //                 DrawBlock &drawBlock = drawFaces[drawFidx].block[blockIdx3];
    //                 if (blockIdx3 == 0) //blockIdx == 0
    //                 {
    //                     vertBlock[0] = p[0];
    //                     vertBlock[1] = p[3];
    //                     vertBlock[2] = p[6];
    //                     vertBlock[3] = p[5];
    //                 }
    //                 else if (blockIdx3 == 1) //blockIdx == 1
    //                 {
    //                     vertBlock[0] = p[1];
    //                     vertBlock[1] = p[4];
    //                     vertBlock[2] = p[6];
    //                     vertBlock[3] = p[3];
    //                 }
    //                 else //blockIdx == 2
    //                 {
    //                     vertBlock[0] = p[2];
    //                     vertBlock[1] = p[5];
    //                     vertBlock[2] = p[6];
    //                     vertBlock[3] = p[4];
    //                 }

    //                 DrawBlockBySearch(pt3d, drawBlock, vertBlock);
    //                 drawBlock.drawFlag = true;
    //                 SearchAdjBlock(blockIdx3, drawFidx, drawFaces, vDrawFaceFlag);
    //             }
    //         }
    //         else if (drawFaces[drawFidx].block[blockIdx3].seeds.size() > 0) //draw block[blockIdx3] first)
    //         {
    //             DrawBlock &drawBlock = drawFaces[drawFidx].block[blockIdx3];
    //             if (blockIdx3 == 0) //blockIdx == 0
    //             {
    //                 vertBlock[0] = p[0];
    //                 vertBlock[1] = p[3];
    //                 vertBlock[2] = p[6];
    //                 vertBlock[3] = p[5];
    //             }
    //             else if (blockIdx3 == 1) //blockIdx == 1
    //             {
    //                 vertBlock[0] = p[1];
    //                 vertBlock[1] = p[4];
    //                 vertBlock[2] = p[6];
    //                 vertBlock[3] = p[3];
    //             }
    //             else //blockIdx == 2
    //             {
    //                 vertBlock[0] = p[2];
    //                 vertBlock[1] = p[5];
    //                 vertBlock[2] = p[6];
    //                 vertBlock[3] = p[4];
    //             }

    //             DrawBlockBySearch(pt3d, drawBlock, vertBlock);
    //             drawBlock.drawFlag = true;
    //             SearchAdjBlock(blockIdx3, drawFidx, drawFaces, vDrawFaceFlag);

    //             if (drawFaces[drawFidx].block[blockIdx1].seeds.size() > 0) //draw block[blockIdx1] next
    //             {
    //                 DrawBlock &drawBlock = drawFaces[drawFidx].block[blockIdx1];
    //                 if (blockIdx1 == 0) //blockIdx == 0
    //                 {
    //                     vertBlock[0] = p[0];
    //                     vertBlock[1] = p[3];
    //                     vertBlock[2] = p[6];
    //                     vertBlock[3] = p[5];
    //                 }
    //                 else if (blockIdx1 == 1) //blockIdx == 1
    //                 {
    //                     vertBlock[0] = p[1];
    //                     vertBlock[1] = p[4];
    //                     vertBlock[2] = p[6];
    //                     vertBlock[3] = p[3];
    //                 }
    //                 else //blockIdx == 2
    //                 {
    //                     vertBlock[0] = p[2];
    //                     vertBlock[1] = p[5];
    //                     vertBlock[2] = p[6];
    //                     vertBlock[3] = p[4];
    //                 }

    //                 DrawBlockBySearch(pt3d, drawBlock, vertBlock);
    //                 drawBlock.drawFlag = true;
    //                 SearchAdjBlock(blockIdx1, drawFidx, drawFaces, vDrawFaceFlag);
    //             }
    //         }
    //     }
    // }

    // vDrawFaceFlag[fid] = true;
}

bool TMesh::TexPaintDrawMesh(vector<ColorFace> &colorFaces, vector<DrawFace> &drawFaces)
{
    FreehandLine &texPaintLine = m_texPaint.m_texPaintLine;
    if (texPaintLine.Pos3d.empty())
        return false;
    if (texPaintLine.Pos3d.size() == 1) //Draw one point
    {
        //original drawFace
        DrawFace drawFace0 = GetOriDrawFace(texPaintLine.Pos3d.front(), texPaintLine.fid.front());

        vector<DrawFace> drawFacesTmp;
        drawFacesTmp.push_back(drawFace0);

        //mark face draw or not
        vector<bool> vDrawFaceFlag;
        vDrawFaceFlag.resize(number_of_faces(), false);

        int drawFidx = 0;
        while ((int)drawFacesTmp.size() > drawFidx)
        {
            if (drawFidx > 1000)
                break;

            DrawFace &drawFace = drawFacesTmp[drawFidx];
            int fidTmp = drawFace.fid;

            //translate draw face position
            FaceInfo &faceInfo = m_texPaint.m_vFaceInfo[fidTmp];
            if (!faceInfo.treatFlag)
            {
                int colorIdx = m_texPaint.GetColorIdx(fidTmp);
                m_texPaint.TransDrawFacePos(fidTmp);

                ColorFace colorFace;
                colorFace.fid = fidTmp;
                colorFace.colorIdx = colorIdx;

                colorFaces.push_back(colorFace);
                faceInfo.treatFlag = true;
            }

            //draw around point
            TexPaintDrawPoint(texPaintLine.Pos3d[0], drawFidx, drawFacesTmp, vDrawFaceFlag);

            drawFidx++;
        }

        for (int j = 0; j < (int)drawFacesTmp.size(); j++)
        {
            drawFaces.push_back(drawFacesTmp[j]);
        }
    }
}