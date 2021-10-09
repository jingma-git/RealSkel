/*
 * @Author: your name
 * @Date: 2020-12-10 14:24:01
 * @LastEditTime: 2020-12-11 19:30:19
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /MyCGAL/src/XSubToy.h
 */
#pragma once

#include "common.h"
#include "TMesh.h"
#include "CSpline.h"
#include "Bone.h"
#include "Joint.h"
#include "Skeleton.h"
#include "straight_skeletor.h"
#include "polygon_partition.h"
#include "Plane.h"
#include "Line.h"

#ifdef SSKEL_DEBUG
class SkelDrawer;
#endif

class XSubToy;

namespace cv
{
    class Mat;
}
class XSubToy
{
public:
    int m_toyID = -1;
    int m_parentID = -1;
    int m_symToyID = -1;
    bool m_isSelected = false;
    //------------------------ relationship with other subtoys --------------------------------------------------
    XSubToy *m_parent = nullptr;
    XSubToy *m_symToy = nullptr;
    std::vector<XSubToy *> m_children;
    Pnt3 m_attachPoint; // the intersection point between this subToy and its parent

    //--Inflation...
    int m_layerID = 0; //-2, -1, 0 (Torso), 1, 2...
    double m_layerThick = 1.0;
    double m_layerDepth = 0;
    int m_layerAxis = COORD_AXIS_Z;

    //------------------------ inflation -------------------------------------------------------------------------
    Vec3 m_iniViewDir;         // initial view direction under which user start drawing contour
    Eigen::Matrix3d m_rotToXY; // rotate the 3D line under 6 canonical view to XY plane, notice: may not overlap with the initial sketch shape

    TMesh m_curMesh2DTemp; // coarse 2D Mesh | disrete m_sketchPolyLine with "edge_length=3"
    TMesh m_curMesh2D;     // refined 2D Mesh on 3D space| disrete m_sketchPolyLine with "edge_length=10"
    // TMesh m_curMesh2DXYPlane; // refined 2D Mesh on 3D XY-Plane, temporary: ToBE removed
    TMesh m_defMesh2D; // copy of m_curMesh2D, but deformed with LBS or AutoDof
    TMesh m_curMesh3D;
    TMesh pMesh3DTemp[2]; // temporary: ToBE removed

    CSpline m_sketchSpline;                 // record control point
    vector<vector<Pnt3>> m_sketchPolyLines; // temporary: descritized m_sketchSpline (control point---control point)
    vector<Pnt3> m_sketchPolyLine;          // user draw 3D line under 6 canonical view
    vector<Pnt3> m_sketchPolyLineOnXY;      // rotate m_sketchPolyLine to 2D

    // skeleton
    bool m_autoSkel = false;
    bool m_isChordalInit = false; //ToBE removed
    //------------------------ translation and rotation ----------------
    Pnt3 m_centroid;

    // symmetry
    Line m_symLine;

    XSubToy();
    XSubToy(const XSubToy *other, const SymetryPlane sym_plane, double plane_pos);
    void Save(std::string out_dir);
    void Load(std::string in_dir);

    std::string name() { return "subToy" + std::to_string(m_toyID); }
    bool IsRoot() { return m_parent == nullptr; }

    void SetID(int id) { m_toyID = id; }
    int GetID() { return m_toyID; }

    void SetLayerID(int id) { m_layerID = id; }
    int GetLayerID() { return m_layerID; }

    void SetSymLine(const Line &line) { m_symLine = line; }

    void SetLayerDepth(double depth) { m_layerDepth = depth; }
    double GetLayerDepth() { return m_layerDepth; }
    void SetLayerAxis(int axis) { m_layerAxis = axis; }
    int GetLayerAxis() { return m_layerAxis; }
    Vec3 GetAxisDepthVec();
    Plane GetLayerPlane();
    double CalDepth();
    void SetZDepth(double depth);

    XSubToy *GetParent() { return m_parent; }
    void SetParent(XSubToy *parent);
    void LeaveParent();
    void DeadToChildren();
    void AddChild(XSubToy *child);
    void EraseChild(XSubToy *child);
    std::vector<XSubToy *> &GetChildren() { return m_children; }
    const std::vector<XSubToy *> &GetChildren() const { return m_children; }
    void SetSymToy(XSubToy *symToy);

    double CalSymPlanePos(const SymetryPlane sym_plane);

    void SetAttachPoint(const Pnt3 &attachPoint) { m_attachPoint = attachPoint; }
    double ContourArea();

    //-------------------------------------Translate/Rotate---------------------------------------------
    void Scale(double scl);
    void Translate(const Vec3 &trans_update);
    void Rotate(int axis, double angle);
    void Rotate(const Eigen::Matrix3d &R);
    void RotateX(double angle);
    void RotateY(double angle);
    void RotateZ(double angle);
    void InitContour(const std::vector<Pnt3> &pts);
    void SetCentroid();
    void GetPlaneFromTriag(const std::vector<Pnt3> &pts, Plane &plane);
    void GetPlaneFromLSQ(const std::vector<Pnt3> &pts, Plane &plane);
    void GetRotToXYFromPlane(Eigen::Matrix3d &R);
    void SetRotToXY();
    void SetSketchPolyLineOnXY();
    void ProcessSelfIsectSketchLine();
    void MapToXY(const std::vector<Pnt3> &pts3d, std::vector<Pnt3> &ptsXY);
    void TranslateSSkel(const Vec3 &trans_update);
    void RotateSSkel(const Eigen::Matrix3d &R);
    void ScaleSSkel(double scl);

    void TranslateBranches(const Vec3 &trans_update);
    void RotateBranches(const Eigen::Matrix3d &R);
    void ScaleBranches(double scl);

    //-------------------------------------Inflation----------------------------------------------------
    void SetInitViewDir(const Vec3 &iniViewDir); // calculate rotation matrix to transform 3D sketch line to 2D
    int InitViewMode() const;                    // under which view is this subtoy first created
    bool Get2DDiscreteLine(const vector<Pnt2> &vec2Pts, vector<Pnt2> &vec2Pts_new, int iPixel, double pixel_dis);
    bool Get2DDiscreteLine(const vector<Pnt3> &vec2Pts, vector<Pnt3> &vec2Pts_new, int iPixel, double pixel_dis);
    void MergeTMeshes(TMesh &mesh3D, TMesh meshes[]);
    void TriangulateCoarseFrom3DContour();
    void TriangulateFineFrom3DContour();
    void InflateLaplaceFrom3DPlane(Float height_ratio);
    bool ElevateMeshLaplace(const TMesh &meshRef, TMesh &meshOut, float fScl, bool bIsUpart);

private:
    // EndPoint is used for CalSkelJoints which is the medial axis's start or end point
    struct LinePoint
    {
        int line_idx;
        int pt_idx;
        LinePoint() : line_idx(-1), pt_idx(-1) {}
        LinePoint(int lidx, int pidx) : line_idx(lidx), pt_idx(pidx) {}
        LinePoint(const LinePoint &rhs)
        {
            line_idx = rhs.line_idx;
            pt_idx = rhs.pt_idx;
        }

        LinePoint &operator=(const LinePoint &rhs)
        {
            line_idx = rhs.line_idx;
            pt_idx = rhs.pt_idx;
            return *this;
        }

        bool operator<(const LinePoint &rhs) const
        {
            return (line_idx < rhs.line_idx) || (line_idx == rhs.line_idx && pt_idx < rhs.pt_idx);
        }
    };

public:
    //------------------------------------- Deform Posture ------------------------------------------------------
    Eigen::MatrixXd m_skinW2D;
    Eigen::MatrixXd m_skinM; // m_defMesh2D.V = m_skinM * m_skinT
    Eigen::MatrixXd m_skinT;
    void DeformLBS();

public:
    //------------------------------------- Skeletonization by Chordal Axis ------------------------------------------------------
    bool isSkeletonCreated = false;
    Bone *m_boneRoot = nullptr;
    Bone *m_attachBone = nullptr; // the bone that attach this subtoy to its parent
    Bone *m_attachBone1 = nullptr;
    std::vector<Bone *> m_subB;
    std::vector<Joint> m_joints;
    Eigen::MatrixXi m_bones; // undirected edge

    bool CalSkelJoints(float tol, double z_depth); // tolerance to simplify
    void ConstructSkeleton(Skeleton<Bone> *skel, double z_depth);
    void ConstructSkeleton(Skeleton<Bone> *skel, const Pnt3 &attachPoint, double z_depth);

public:
    // ToDO: remove -------------------------------Inflation old Methods----------------------------------------------------
    bool TriangulateCoarse(const vector<Pnt3> &contPts);
    bool TriangulateFine(const vector<Pnt3> &contPts, float max_triArea = 200);

    //------------------------------------- Inflate Teddy ------------------------------------------------------
    //https://www-ui.is.s.u-tokyo.ac.jp/~takeo/teddy/teddy.htm
    vector<C3DLine> m_vAxisLines;
    bool CalChordalAxisLines();
    bool CalChordalAxisLineCylinder();
    bool CalChordalAxisLineThinning();
    void InflateTeddy(Float height_ratio);
    bool GetElevateValueInToAxisLines(const TMesh &mesh, vector<C3DLine> &lines);
    void Refine2DMesh(TMesh &ms, Float fRatio);
    void ElevateMesh(const TMesh &meshRef, TMesh &meshOut, TMesh &meshCoarse, vector<C3DLine> &lines, float fScl, bool bIsUpart);
    float GetElevateValueInTriangle(const Pnt3 &pi, TMesh &meshCoarse, Fd fd, vector<C3DLine> &lines);
    float GetElevateValue(float fLen, float fLenSum, float fMaxOff);
    float GetElevateValueInSleeveOrTerminalT(Pnt3 vt, TMesh &mesh, Fd iFaceID, vector<C3DLine> &lines);

    //------------------------------------- Inflate Laplace ------------------------------------------------------
    //https://igl.ethz.ch/projects/monster-mash/
    void InflateLaplace(Float height_ratio);
    void ArapDeform(int numLayers);
    void ArapDeformTwoLayer(int numLayers);
    bool ElevateMeshLaplace(const TMesh &meshRef, TMesh &meshOut, Eigen::VectorXi &bndIdxs, float fScl, bool bIsUpart);

public:
    //------------------------------------- Deform Shape ------------------------------------------------------
    int m_vDefCptIdx = -1;
    std::vector<std::vector<double>> m_affineCoeff2D, m_affineCoeff;
    std::vector<Pnt3> m_defContour;
    std::vector<std::vector<Pnt3>> m_SimplifiedSketchPolyLines; // see: m_sketchPolyLines
    CSpline m_defSpline;                                        // 3D SubToy part silhouette under ViewDir
    vector<vector<Pnt3>> m_defPolyLines;                        // Get from m_defSpline
    vector<Pnt3> m_defPolyLine;
    std::vector<std::vector<Pnt3>> m_SimplifiedDefPolyLines; // Get from m_defPolylines

    bool m_bNeedComputeCoeff2D = true;
    void InitializeSimplifiedSketchPolyLines(const vector<vector<Pnt3>> &sketchPolylines, vector<vector<Pnt3>> &simpPolyLines);
    void ReComputeSimplifiedSketchPolyLines(CSpline &spline, vector<vector<Pnt3>> &simpPolyLines, int index);
    void InitContourDeform(const Vec3 &viewDir);
    void DeformContour(const Vec3 &viewDir);
    void GetRotateAngleAxis(const Vec3 &viewDir, Vec3 &axis, float &angle);
    void GetMeshPts2D(const TMesh &mesh, vector<Pnt3> &mesh_pts2D);
    void GetContourPts2D(const vector<Pnt3> &contour3D, vector<Pnt3> &contour2D);
    void ComputeAffineTransformationCoefficient3D(const TMesh &mesh, const vector<Pnt3> &contour,
                                                  const Vec3 &viewDir, vector<vector<double>> &coeff);
    void ComputeAffineTransformationCoefficient2D(const TMesh &mesh, const vector<Pnt3> &contour,
                                                  vector<vector<double>> &coeff);
    void ComputeAffineTransformationCoefficient(const vector<Pnt3> &contour, const vector<Pnt3> &mesh_pts, vector<vector<double>> &coeff);

    void RefereshModifiedMeshVerticesByAffineTransformation3D(const vector<Pnt3> &deformedContour,
                                                              const vector<vector<double>> &coeff,
                                                              const Vec3 &viewDir,
                                                              TMesh &mesh);
    void RefereshModifiedMeshVerticesByAffineTransformation2D(const vector<Pnt3> &deformedContour,
                                                              const vector<vector<double>> &coeff,
                                                              TMesh &mesh);
    void RefereshModifiedMeshVerticesByAffineTransformation(const std::vector<Pnt3> &deformedContour,
                                                            const std::vector<std::vector<double>> &coeff,
                                                            std::vector<Pnt3> &mesh_pts);

public:
    //------------------------------------- skeletonization ------------------------------------------------------
    // Input: 2D polygon under screen space (m_skethcPolylineOnXY)
    // Ouput: 3D coarse-to-fine skeleton under world space (m_sskel)
    typedef SSNode NodeT;
    typedef SSEdge<NodeT> EdgeT;
    typedef SSBranch<NodeT> SSBranchT;
    int m_skelOpt = SKEL_OFFSET_POLYGON;
    const double DP_THRESH = 5; // default dp simplfication threshold

    SSkel<NodeT> m_rawSSkel;
    std::vector<NodeT *> junc_nodes, term_nodes; // nodes collected from m_rawSSkel
    std::list<SSBranchT> branches;               // branches collected from m_rawSSkel
    SSkel<NodeT> m_sskel;                        // skeleton after simplify branches of m_rawSSkel

    void set_skel_opt(int skel_opt) { m_skelOpt = skel_opt; }
    void Skeletonize(int skel_opt);
    void Skeletonize();
    void interactive_simplify_skeleton(double dp_thresh);

private:
    void skeletonize_from_sym_line();
    void create_skeleton_from_line(const Line &l, SSkel<NodeT> &sskel);
    void collect_branches(const std::vector<SSNode *> &junc_nodes,
                          const std::vector<SSNode *> &term_nodes,
                          std::list<SSBranchT> &branches);
#ifdef SSKEL_DEBUG
    std::shared_ptr<SkelDrawer> skelDrawerPtr;

    void simplify_branches(std::list<SSBranchT> &branches,
                           PolyPartition &poly_partition);
    void simplify_bounded_dp(SSBranchT *bPtr,
                             const vector<Pnt3> &oriPArr,
                             const vector<Slice> &slices,
                             vector<Pnt3> &resPArr, float tol);
    void bounded_dp(SSBranchT *bPtr,
                    int iter,
                    const vector<Pnt3> &PArr,
                    const vector<Slice> &slices,
                    int stIdx, int endIdx,
                    vector<bool> &PArrMark, Float tol, double weight);
#else
    void simplify_branches(std::list<SSBranchT> &branches,
                           PolyPartition &poly_partition);
#endif
    void SimplifyPolyLineDP(const vector<Pnt3> &oriPArr, vector<bool> &,
                            float tol);
    void simplify_one_branch(SSBranchT &b, double init_thresh, const std::vector<Pnt3> &contour);
    bool IsLineInsidePolygon(const std::vector<Pnt3> &line, const std::vector<Pnt3> &polygon);

    void create_skeleton();
    void create_tree_skel(const std::vector<NodeT *> &junc_nodes,
                          std::unordered_map<NodeT *, std::vector<SSBranchT *>> &node_branches,
                          NodeT *root,
                          SSkel<NodeT> &simp_sskel);
    void create_ribon_skel(const std::vector<NodeT *> &term_nodes,
                           const SSBranchT &branch,
                           SSkel<NodeT> &simp_sskel);
    NodeT *node_with_max_num_branches(const std::unordered_map<NodeT *, std::vector<SSBranchT *>> &node_branches);
};

std::ostream &operator<<(std::ostream &out, const XSubToy &subToy);
std::istream &operator>>(std::istream &in, XSubToy &subToy);