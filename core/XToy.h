#pragma once

#include <set>
#include <igl/NormalType.h>
#include <igl/arap_dof.h>

#include "MemoryPool.h"
#include "XSubToy.h"

// skeletonization
#include "straight_skeletor.h"
#include "ssnode_with_subtoy_info.h"

// Rig & Animation
#include "Skeleton.h"
#include "Bone.h"
#include "Animation.h"
#include "BoneBoneCopyMap.h"

// Painting
#include "Color.h"
#include "Texture.h"
#include "BrushPainter.h"

class XToy
{
private:
    MemoryPool<XSubToy> m_subtoyPool;

public:
    std::string m_name;
    // pointer/iterator invalidation http://www.cplusplus.com/forum/general/174320/
    //!!! Never Never Never
    // use vector1 to store tempory XSubToy, and use vector2 to store pointers to vector1's element
    // every time you push_back(), it will reallocate, and pointers in vector2 is no longer valid
    std::vector<XSubToy *> m_subToyRoots; // in most cases, m_subToyRoots.size()==1, m_subToyRoots[0].type == TORSO
    std::vector<XSubToy *> m_subToys;
    static int NUM_INI_SUBTOYS; // how many space allocated initially for subtoys

public:
    XToy();
    ~XToy();
    void ClearSubToys();
    void SetName(std::string name) { m_name = name; }
    bool load(const string folder_name);
    bool save(const string folder_name);

    XSubToy *AddSubToy();
    XSubToy *AddSymSubToy(XSubToy *other, SymetryPlane sym_plane, double plane_pos);
    XSubToy *GetCurToy() { return m_curSubToyIdx < 0 ? nullptr : m_subToys[m_curSubToyIdx]; }
    XSubToy *GetParentToy() { return m_parentToyIdx < 0 ? nullptr : m_subToys[m_parentToyIdx]; }
    void DelSubToy(XSubToy *subToy);
    void SetupParentChildRelation3D(XSubToy *subToy);
    void ResetRoots(); // after parent-child relationship change by move/rotate subtoy, call this

public:
    // Fast Automatic Skinning -----------------------------------------------------
    // https://www.cs.utah.edu/~ladislav/jacobson12fast/jacobson12fast.html
    Skeleton<Bone> *skel = nullptr; // visualization
    Skeleton<Bone> *restSkel = nullptr;

    // Use bone's last_T to *set* the transformation of the bone, i.e. don't
    // use bones' rotation/translation/etc to set T. Thus last_T should be
    // directly edited ("Dialed in") for each bone.
    bool dial_in_each_T; //skel->draw_according_to_last_T

    // #V by 3 Matrix of mesh vertex 3D positions
    Eigen::MatrixXd V, V2D;
    // #F by 3 Matrix of face (triangle) indices
    Eigen::MatrixXi F, F2D;
    // #Tets by 4 Matrix of tet (indices), empty means surface
    Eigen::MatrixXi Tets;

    // Original #V by #original_weights Matrix of per-mesh vertex, per-handle
    // weights unsorted.
    Eigen::MatrixXd OW;
    // Extra weights, #V by #extra_weights Matrix of per-mesh vertex,
    // per-handle weights (probably don't partition unity)
    Eigen::MatrixXd EW;
    // #V by
    // min(#original_weights+#extra_weights,MAX_NUM_WEIGHTS_PER_VERTEX)
    // Matrix of per-mesh vertex, per-handle weights sorted by value
    Eigen::MatrixXd W, W2D;
    // min(#original_weights+#extra_weights,MAX_NUM_WEIGHTS_PER_VERTEX)
    // Matrix of per-mesh vertex, per-handle indices corresponding to handles
    // of weights in W sorted by value
    Eigen::MatrixXi WI; //ToDO: remove
    // LBS matrix as being used by ARAP_DOF
    Eigen::MatrixXd M, M2D, Mcol;
    // used to store CPU version of deformed mesh vertices and normals
    Eigen::MatrixXd cpuV, cpuV2D;
    // "horizontally stacked" transformation matrices
    // If a single transformation is a #rows by #cols matrix then T is a
    // #rows by #actual_weights*#cols matrix
    Eigen::MatrixXd T;
    // "vertically stacked" transformation matrices
    // If a single transformation is a 3 by 4 matrix then T_vertical is a
    // #actual_weights*4 by 3 matrix
    Eigen::MatrixXd T_vertical;

    /////////////////////////////////////////////////////////////////////////
    // Auto DOF
    /////////////////////////////////////////////////////////////////////////
    bool auto_dof = false;
    bool bypass_auto_dof = false; //ToDO
    int num_groups = 0;
    int num_extra_weights = 0;
    int max_iters = 1;
    double tol = 0;
    igl::ArapDOFData<Eigen::MatrixXd, double> arap_dof;
    // Current/Last frame's solution
    Eigen::MatrixXd L;
    // Methods -------------------------------------
    // Returns a const reference to elements we're auto-doffing over. Either to
    // (triangles) F or (tetrahedra) Tets
    const Eigen::MatrixXi &elements();
    // Called after loading V,F,Tets
    bool initialize_mesh();
    // Initialize weights, sets up W,WI based on TW
    bool initialize_weights();
    // Initialize transformations for each handle to Identity
    bool initialize_transformations();

    // Loads a mesh from a file into V,F
    // Inputs:
    //   mesh_file_name  path to mesh file (obj/off)
    // Returns true on success, false on errors
    bool load_mesh_from_file(const std::string mesh_file_name);

    // Input:
    //  weights_file_name  path to .dmat matrix file containing weights
    // Returns true on success, false on error
    bool load_weights(const std::string weights_file_name);

    bool load_bone_roots_animation(const std::string bone_roots_file_name);

    // Load bone roots from a given .bf file
    //
    // sets skel->roots
    //
    // Input:
    //   bone_roots_file_name  path to .bf bone roots file
    //   must_match  whether bone forest must perfectly match current one
    //   no_recompute  don't recompute auto dof even if it's using it. Better match
    //     dof_types!! it's your funeral
    // Returns true on success, false on error
    bool load_bone_roots(
        const std::string bone_roots_file_name,
        const bool must_match = false,
        const bool no_recompute = false);

    bool save_bone_roots(const std::string bone_roots_file_name);

    // Save bone roots animation to a given .bf file and subsequent files see
    // load_bone_roots_animation for naming convention
    //
    // Input:
    //   bone_roots_file_name  file name of first pose
    bool save_bone_roots_animation(const std::string bone_roots_file_name);

public:
    TMesh m_mesh3D, m_mesh3DTmp;
    int m_curSubToyIdx = -1;
    int m_parentToyIdx = -1;
    bool m_usePickedParent = false;
    int m_pickFid = -1; // picked face id on m_curSubToy's m_curMesh3D
    Vec3 m_pickFbc;     // barycenter of picked face on m_curSubToy's m_curMesh3D
    Pnt3 m_pickFPnt;
    bool m_isSubToyMoved = true; // Initially set this to true to merged newly-created sub-meshes

    int pick_subtoy_impl(const int sx, const int sy,
                         const int width, const int height,
                         float *mvp);

    bool pick_subtoy(const int sx, const int sy,
                     const int width, const int height,
                     float *mvp,
                     bool ctrl_down = false);

    bool pick_parent_subtoy(const int sx, const int sy,
                            const int width, const int height,
                            float *mvp);

    void clear_deform_data();
    bool move_subtoy(const int sx, const int sy,
                     const int width, const int height,
                     float *mvp);
    void rotate_subtoy(int axis, double angle); // angle is in degree, need to transform to rad
    void scale_subtoy(double scl);
    void translate_sskel(XSubToy *subToy, const Vec3 &trans_update);

    void compute_weights2D(int width, int height, float *mvp);
    void compute_weightsTet();
    void correct_mesh_orientation();
    void correct_mesh_normals();
    void fill_mesh_holes();
    bool compute_V3D(); // merge

    // Set default values for automatic degrees of freedom related variables
    // Returns true on success, false on error
    bool initialize_auto_dof();
    bool initialize_auto_dof1();
    // Reinitialize auto dof without compleletly redoing precomputation, should
    // be called only when free/fixed change
    bool reinitialize_auto_dof();
    bool reinitialize_auto_dof1();

    bool drag_bone(int sx, int sy,
                   int width, int height,
                   float *viewMatrix, float *mvpMatrix,
                   bool right_click, bool shift_down, bool ctrl_down);

    void gather_subB(int width, int height, float *mvpMatrix, XSubToy *subToy);
    // gather transformations for subtoy according to bones inside subtoy
    void gather_subT(int width, int height, float *mvpMatrix, XSubToy *subToy);

    Bone *pick_bone(int sx, int sy, int width, int height, float *mvpMatrix, bool shift_down, bool ctrl_down);

    void set_rest_pose();

    bool transformations();

public:
    // Interactive SKeletonization/ Straight Skeleton-------------------------------------
    double m_merge_thresh = 30;
    double m_prune_thresh = 30;
    double m_collapse_thresh = 10;
    SSkel<SSNodeWithSubToyInfo> m_rawSSkel; // skeleton after Branch-Level DP simplfication
    SSkel<SSNodeWithSubToyInfo> m_sskel;
    std::unordered_map<XSubToy *, SSNodeSub *> m_subSSkelRoots;
    std::vector<XSubToy *> m_pickedSubToys;
    void pick_all_subtoys();
    void unpick_all_subtoys();
    void pick_current_subtoy();

    void global_skel_optim();
    void merge_prune_collapse();
    void connect_subskel(XSubToy *parent, XSubToy *child);
    void build_global_sskel();

    void restore_state();
    // 1. ToDO: dp simplification, branch level
    void change_bone_complexity(int level); // dp simplification

    // 2. junction joints merging, part level & global level
    double avg_bone_len();
    void merge_junc_nodes(int merge_thresh);
    void collect_nodes(SSkel<SSNodeSub> &sskel,
                       std::vector<SSNodeSub *> &junc_nodes,
                       std::vector<SSNodeSub *> &term_nodes);
    void merge_junction_nodes(SSkel<SSNodeSub> &sskel, double merge_thresh);

    // 3. short-jt-bone trimming, part level & global level
    double max_jt_edge_len();
    void prune_jt_edges(int prune_thresh);
    void find_jt_edges(SSkel<SSNodeSub> &sskel, std::vector<SSEdge<SSNodeSub> *> &jt_es);
    void prune_short_jt_edges(SSkel<SSNodeSub> &sskel, int prune_thresh);

    // 4. short-internal-bone collapsing, part level & global level
    double max_bone_len();
    void collapse_short_edges(int collapse_thresh);
    void collapse_short_edges(SSkel<SSNodeSub> &sskel, int collapse_thresh);

    void visualize_global_sskel();
    void visualize_global_sskel(Skeleton<Bone> *vis_skel);
    void add_subskel_to_global(XSubToy *subToy);
    SSNodeSub *find_root(XSubToy *rootToy);
    void set_zdepth(XSubToy *subToy, double zdepth);
    void add_sym_skel(SymetryPlane sym_plane, double plane_pos, XSubToy *orig_subToy, XSubToy *sym_subToy);

    // skeletonization by Contrained Delauny Triangulation and Chordal Axis, Thinning
    std::map<XSubToy *, bool> is_bone_created;
    void compute_bone_chordal_axis();
    void compute_bone_medial_axis();
    void add_sym_skel(SymetryPlane sym_plane, Bone *sel_bone);
    void add_sym_skel(SymetryPlane sym_plane, Bone *sel_bone, double plane_pos);

public:
    // -------------- Animation ----------------------------
    // global animation time increased by 33ms each frame (i.e. assuming 30FPS playback)
    float anim_timer;

    Animation<BoneBoneCopyMap> animation;
    // Animation start time
    double animation_start_seconds;
    // Currently animating
    bool animating;
    // Transition type to be used for next keyframe
    TransitionType transition_type;
    // number of seconds to interpolat e this keyframe and the next one
    double animation_interp_secs;
    bool update_animation(double t);
    bool start_animating();
    bool stop_animating();

public:
    // Painting -------------------------------------
    int m_texWidth;   //texture image width, height is the same
    int m_blockWidth; //triangle block width, height is the same
    int m_blockSize;  //triangle block size in a row
    BrushPainter m_brushPainter;
    vector<ColorI> m_vPaintColor; // all colors that has paint with
    vector<Texture *> m_textures;

    UINT CreateTexture();
    Texture *GetTexture(UINT i);
    void ClearTexture();
    int GetPaintColorIdx(const ColorI &color);
    void InitTexturePaint(const ColorI &color);
    void TexturePaintDraw(const ColorI &paintColor);

#ifdef ToDEL
    void SetupParentChildRelation(XSubToy *subToy);
#endif
};

std::ostream &operator<<(std::ostream &out, const XToy &toy);
std::istream &operator>>(std::istream &in, XToy &subToy);