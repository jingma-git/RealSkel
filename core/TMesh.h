/*
 * @Author: your name
 * @Date: 2020-12-10 14:12:10
 * @LastEditTime: 2020-12-11 19:28:55
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /MyCGAL/src/TMesh.h
 */
#pragma once
#include "common.h"
#include "3DLine.h"
#include "Color.h"
#include "TexPaint.h"
#include <Eigen/Sparse>
#include <map>

//--- Teddy Inflate--------
enum FaceType
{
    JUNCTION = 0,
    SLEEVE,
    TERMINAL
};

//--- Arap Layer------------
enum LConstrainType
{
    LowerConstrain = 0,
    UpperConstrain,
    InvalidConstrain
};

class TMesh : public SMesh
{
private:
    Property_map<Vd, Vec3> m_vNormals;
    Property_map<Fd, Vec3> m_fNormals;

    Eigen::MatrixXd m_V;
    Eigen::MatrixXi m_F;
    std::vector<double> m_C;                    // cotangent weights for each halfedge
    Eigen::SparseMatrix<double> m_L;            // #V by #V laplace matrix
    Eigen::SparseMatrix<double> m_M;            // mass matrix
    std::map<int, bool> m_fixedV;               // #V by #V whether the vertex is fixed or not when doing laplace inflation (vertex on the sketchLine)
    std::map<int, int> m_layerV;                // Monster Mash, mark which layer the vertex belongs to [0, 1, 2...]
    std::map<int, LConstrainType> m_lConstrain; // Monster Mash, if the vertex is above parent mesh, it has lower constraint; if it is under child mesh, it has upper constraint

public:
    TMesh();
    void CreateHalfedgeMesh(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F);

    //------------------------------------- Common function ----------------------------------------------------------------
    // double *gl_vertices();

    TMesh &operator=(const TMesh &rhs);
    void clear();
    Pnt3 barycent(Fd fd);
    Pnt3 barycent(const vector<Fd> &fds); // https://bell0bytes.eu/centroid-convex/
    Float edge_len(Vd v0, Vd v1);
    Float edge_len(Ed e);
    Float edge_len(Hd h);
    int num_adj(Vd v);
    Float max_edge_len();
    Float avg_edge_len();
    Vec3 face_normal(Fd fd);
    Vec3 face_normal(Fd fd) const;
    bool is_border_edge(Hd hd);
    BBox3 bbox();
    void SetZDepth(double depth); //for 2D Meshes, set its z value

    Eigen::RowVectorXd barycenter(); // this is not real barycenter

    Pnt3 GetFPos(const int fid, const Vec3 &bc); // barycentric coordinates so that
                                                 //   pos = V.row(F(id,0))*(1-u-v)+V.row(F(id,1))*u+V.row(F(id,2))*v;
    void Translate(const Vec3 &offset);
    void Rotate(const Eigen::Matrix3d &rot);
    void Scale(double scl);
    void ComputeFNormals();
    void ComputeVNormals(); // may need to optimize
    const Vec3 &GetFNormal(Fd fd);
    const Vec3 &GetVNormal(Vd vd);

    void ReverseFaceOrientation();
    void FindBoundary(std::vector<int> &bnd_inds);
    void FindBoundary(std::vector<Vd> &bnd_inds);
    void FindBoundary(std::vector<Pnt3> &pts);
    void FindSketchBoundary(std::vector<Vd> &sketch_verts);
    void UpdateVMat();
    Eigen::MatrixXd &GetVMat();
    Eigen::MatrixXi &GetFMat();
    Eigen::SparseMatrix<double> &GetLaplaceMat();
    Eigen::SparseMatrix<double> &GetMassMat();
    const std::map<int, bool> &GetFixedVMap() const { return m_fixedV; };
    const std::map<int, int> &GetLayerVMap() const { return m_layerV; };
    const std::map<int, LConstrainType> &GetLayerConstrainMap() const { return m_lConstrain; };
    void SetPoints(const Eigen::MatrixXd &V);
    bool IsFixed(int vidx);
    int GetLayerId(int vidx);
    LConstrainType GetLayerConstrainType(int vidx);
    void MarkFixedVerts(const vector<Pnt2> &verts);
    int GetNumFixed(); // for Debug, ToDO: Improve

    //------------------------------------- Selection ----------------------------------------------------------------
    Vd FindClosestV2D(const Pnt3 &p);
    Fd FindLocatedFaceIn2D(const Pnt3 &p);
    Vd FindClosestV(const Pnt3 &p);

    //-------------------------------------Mesh Operation Subdivision|Smoothing|Refining...---------------------------------
    bool RefineLocalMesh(vector<int> &flag, bool bIsSmooth); //subdivision
    bool RefineLocalMesh();                                  //subdivision
    void SmoothenMeshNaive(const vector<Vd> &vVtIdx, int nIter, Float fScl = 20, Float len_avg = 0, bool keep_boundary = true);
    void SmoothenMeshEvenFilter(const vector<Vd> &vVtIdx, int nIter, Float fScl = 20, Float len_avg = 0, bool keep_boundary = true);
    void AdjustMeshToBound(const vector<Pnt3> &cnt_pts);

    //---------------------------------------------------------Teddy Inflate -----------------------------------------------
private:
    bool m_bIsFaceTypeSet;
    Property_map<Fd, FaceType> m_fTypes;
    Property_map<Ed, bool> m_eVisit;

public:
    //--------------------------------- Getters----------------------------------------
    FaceType GetFType(Fd fd);
    void SetFType(const Fd &fd, FaceType type) { m_fTypes[fd] = type; }
    FaceType GetFType(Fd fd) const;
    Float Get3DFArea(Fd fd);
    Float Get3DFArea(int f_idx);
    Float Get3DArea();
    //--------------------------------- Setters ----------------------------------------
    void UpdatePos(const vector<Pnt3> &newV);
    //---------------------------------Chordal Axis Line--------------------------------
    void GetChordalAxisLines(vector<C3DLine> &lines);
    void GetChordalAxisLinesCylinder(vector<C3DLine> &lines);
    void LabelXFaceType();
    void GetJunctionFaces(vector<Fd> &vJFaceId);
    void SearchChordalAxisLinesFromAJFace(Fd iStFace, vector<C3DLine> &lines);
    void SearchChordalAxisLinesBetweenTwoTFace(Fd iStF, Fd iEndF, vector<C3DLine> &lines);
    void PruneChordalAxis(const vector<Fd> &junctionFs, const vector<C3DLine> &rawLines, vector<C3DLine> &prunedLines);
    void ResetEVisit();
    //------------------------------- PMP ----------------------------------------------------
    void ReverseOrientation();

    //--------------------------------- Monster-Mash Laplace Inflation ------------------
    bool GetCutLine(const Pnt3 &stPt, const Pnt3 &endPt, C3DLine &line);
    bool CutMesh(const vector<C3DLine> &cutLines, const vector<vector<Pnt3>> &child_contours, vector<vector<int>> &sew_lines);
    void AttachMeshes(const vector<vector<int>> &sew_lines, const std::vector<TMesh *> &subMeshes); // vector<int> sew_verts
    bool CutMeshHalfedge(const C3DLine &line);
    // ARAP Deform
    void MarkLayeredVertex(const vector<vector<Pnt3>> &child_contours, int layerID);
    void MarkLayeredVertex(const vector<Pnt3> &parent_contours, int layerID);
    void ArapDeformTest();
    double CotVal(Vd v0, Vd v1, Vd v2);
    double CotWeight(Hd he);
    void CovarianceScatterMatrixOneRing(const std::vector<Eigen::Vector3d> &V,
                                        const std::vector<Eigen::Vector3d> &U,
                                        std::vector<Eigen::Matrix3d> &rot_mtr);
    void CovarianceScatterMatrix(); // for Debug

    template <class Derived, class OtherDerived>
    int Copy(Eigen::DenseBase<Derived> &V, int row_idx, const Eigen::DenseBase<OtherDerived> &vec3)
    {
        V.row(row_idx) = vec3;
        return row_idx;
    }

    int Copy(Eigen::MatrixXi &F, int r, int v0, int v1, int v2)
    {
        // r: row_idx
        F(r, 0) = v0;
        F(r, 1) = v1;
        F(r, 2) = v2;
        return r;
    }

public:
    //------------------------------Skeletonization by Mesh Contraction --------------------
    struct MeshContractData
    {
        int max_iter = 200;
        double VolumeRatioThresh = 0.00001;
        double SL = 2.0; // speed for laplace contraction
        double WL0 = 1.0;
        double WH0 = 1.0; // initial attraction weight
    };
    std::vector<std::vector<int>> m_adjV;
    double volume();
    double one_ring_area(int vidx);
    double one_ring_area(int vidx, const Eigen::MatrixXd &U);
    void Contract(MeshContractData &data); // geometry collapse by laplace contraction force and 'attraction' force

public:
    //------------------------------Painting -----------------------------------------------
    TexPaint m_texPaint;
    void InitTMeshTexPaint(int texWidth, int blockWidth, int colorIdx);
    void UpdatePaintFaceInfo();
    DrawFace GetOriDrawFace(const Pnt3 &pt3d, int fid);
    void SearchAdjBlock(int blockIdx, int drawFidx, vector<DrawFace> &drawFaces, vector<bool> &vDrawFaceFlag);
    void TexPaintDrawPoint(Pnt3 pt3d, int drawFidx, vector<DrawFace> &drawFaces, vector<bool> &vDrawFaceFlag);
    bool TexPaintDrawMesh(vector<ColorFace> &colorFaces, vector<DrawFace> &drawFaces);
};

template <class Graph>
void make_plane(Graph &sm, int W = 2, int H = 1, double len = 1.0)
{
    int v_num = (W + 1) * (H + 1);
    Vd V[v_num];

    for (int v_idx = 0; v_idx < v_num; v_idx++)
    {
        int r = v_idx / (W + 1);
        int c = v_idx % (W + 1);
        V[v_idx] = sm.add_vertex();
        sm.point(V[v_idx]) = Pnt3(c * len, r * len, 0);
    }

    for (int i = 0; i < H; i++)
    {
        for (int j = 0; j < W; j++)
        {
            int v0 = i * (W + 1) + j;
            int v1 = i * (W + 1) + (j + 1);
            int v2 = (i + 1) * (W + 1) + j;
            int v3 = (i + 1) * (W + 1) + (j + 1);

            sm.add_face(V[v0], V[v1], V[v2]);
            sm.add_face(V[v1], V[v3], V[v2]);
        }
    }
}

inline void make_cube(SMesh &sm, double edge_len = 1.0)
{

    Vd V[8];
    for (int i = 0; i < 8; i++)
    {
        V[i] = sm.add_vertex();
    }
    double len = edge_len * 0.5;
    sm.point(V[0]) = Pnt3(-len, -len, len);
    sm.point(V[1]) = Pnt3(len, -len, len);
    sm.point(V[2]) = Pnt3(-len, len, len);
    sm.point(V[3]) = Pnt3(len, len, len);

    sm.point(V[4]) = Pnt3(-len, -len, -len);
    sm.point(V[5]) = Pnt3(len, -len, -len);
    sm.point(V[6]) = Pnt3(-len, len, -len);
    sm.point(V[7]) = Pnt3(len, len, -len);

    // front
    sm.add_face(V[0], V[1], V[2]);
    sm.add_face(V[1], V[3], V[2]);
    // back
    sm.add_face(V[6], V[5], V[4]);
    sm.add_face(V[6], V[7], V[5]);
    // top
    sm.add_face(V[6], V[2], V[3]);
    sm.add_face(V[6], V[3], V[7]);
    // bottom
    sm.add_face(V[4], V[1], V[0]);
    sm.add_face(V[4], V[5], V[1]);
    // left
    sm.add_face(V[2], V[6], V[4]);
    sm.add_face(V[2], V[4], V[0]);
    // right
    sm.add_face(V[3], V[5], V[7]);
    sm.add_face(V[3], V[1], V[5]);
}

// Generate symmetric mesh
void GenSymMesh(const SymetryPlane sym_plane, const double plane_pos, const TMesh &oMesh, TMesh &mesh);