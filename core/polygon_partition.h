#pragma once

#include "straight_skeletor.h"
#include "slice_polygon.h"
#include "CSpline.h"

class PolyPartition
{
public:
    typedef SSNode NodeT;
    typedef SSBranch<NodeT> SSBranchT;

    PolyPartition(const std::vector<Pnt3> &polygon, std::list<SSBranchT> &branches);

    enum Convexity
    {
        FLAT, // exactly equals to PI
        CONVEX,
        REFLEX
    };

    class PVert
    {
    public:
        PVert(int id, const Pnt3 &pos)
            : id_(id), pos_(pos),
              id_reflex_cluster_(-1),
              branch_(nullptr),
              is_delimeter_(false) {}

        int id() const { return id_; }
        const Pnt3 &pos() const { return pos_; }
        int id_reflex_cluster() const { return id_reflex_cluster_; }
        SSBranchT *branch() { return branch_; }
        bool is_reflex() const { return sign_ == REFLEX; }
        double angle() const { return angle_; }
        bool is_processed() const { return branch_ != nullptr; }
        bool is_delimeter() const { return is_delimeter_; }

        void set_id_reflex_cluster(int cluster_id) { id_reflex_cluster_ = cluster_id; }
        void set_branch(SSBranchT *branch) { branch_ = branch; }
        void set_sign(Convexity sign) { sign_ = sign; }
        void set_angle(double angle) { angle_ = angle; }
        void mark_as_delim() { is_delimeter_ = true; }

    private:
        int id_;
        int id_reflex_cluster_; //ToDO: remove
        SSBranchT *branch_;
        Pnt3 pos_;
        Convexity sign_;
        double angle_;
        bool is_delimeter_;
    };

    struct ReflexCluster
    {
        ReflexCluster(int cluster_id) : id(cluster_id) {}
        void add_vert(int i) { vertices.push_back(i); }

        int id;
        int id_sharp; // the most 'sharp' vert in this
        std::vector<int> vertices;
    };

    typedef typename std::vector<ReflexCluster>::iterator ClusterIter;
    typedef typename std::list<SSBranchT>::iterator BranchIter;
    typedef typename std::vector<Slice>::iterator SliceIter;

    const std::vector<Pnt3> &polygon() const { return polygon_; }
    std::unordered_map<SSBranchT *, std::vector<Slice>> &branch_slices() { return branch_slices_; }
    std::unordered_map<SSBranchT *, std::vector<Slice>> &branch_vslices() { return branch_vslices_; }
    std::unordered_map<SSBranchT *, std::vector<Pnt3>> &branch_densamples() { return branch_densamples_; }
    std::unordered_map<SSBranchT *, std::vector<Pnt3>> &branch_unisamples() { return branch_unisamples_; }
    std::unordered_map<SSBranchT *, std::vector<Pnt3>> &branch_polygons() { return branch_polygons_; }
    std::unordered_map<SSBranchT *, std::vector<Pnt3>> &branch_smthpolys() { return branch_smthpolys_; }
    std::unordered_map<SSBranchT *, std::vector<int>> &branch_silidxs() { return branch_silidxs_; }

    void partition(); // ToDO: make this private

    // for short junction-junction branch, we don't create bounded polygon for it
    // neigther is the Shape Energy used in DP simplication
    bool has_polygon(SSBranchT *b) { return branch_polygons_[b].size() > 0; }

private:
    void process_tt_branch(SSBranchT &b);
    void process_jt_branch(SSBranchT &b);
    void process_jj_branch(SSBranchT &b);

    void sample_spline(CSpline &spline,
                       std::vector<Pnt3> &dense_samples,
                       std::vector<Pnt3> &uniform_samples);
    void slice_along_unisamples(const std::vector<Pnt3> &uniform_samples,
                                std::vector<Slice> &slices);

    void cal_vslices(const std::vector<Slice> &slices,
                     std::vector<Slice> &vslices);

    void find_branch_segs(SSBranchT *b,
                          std::vector<int> &segs1st,
                          std::vector<int> &segs2nd);

    void connect(const std::vector<int> &segs1st,
                 const std::vector<int> &segs2nd,
                 std::vector<int> &branch_silidx);

    void connect(const std::vector<int> &segs1st,
                 const std::vector<int> hole,
                 const std::vector<int> &segs2nd,
                 std::vector<int> &branch_silidx);

    /**
     * @brief Create a polygon from junction-terminal branch
     * segs1st--filled_range--segs2nd
     * Input
     * @param b 
     * @param segs1st :segments along branch
     * @param segs2nd : segments along branch
     * 
     * Output
     * @param branch_polgyon 
     * @param branch_silidx 
     * swap segs1st and segs2nd if there is zero in filled_range
     */
    void create_jtbranch_polygon(SSBranchT *b,
                                 std::vector<int> &segs1st,
                                 std::vector<int> &segs2nd,
                                 std::vector<Pnt3> &branch_polgyon,
                                 std::vector<int> &branch_silidx);

    void create_jtbranch_smthpoly(const std::vector<Pnt3> &branch_polgyon,
                                  std::vector<Pnt3> &branch_smthpoly);

    /**
     * @brief Create a polygon from junction-junction branch
     * segs1st--filled_range--segs2nd--filled_range--segs1st.front
     * Input
     * @param b 
     * @param segs1st :segments along branch
     * @param segs2nd : segments along branch
     * 
     * Output
     * @param branch_polgyon 
     * @param branch_silidx 
     */
    void create_jjbranch_polygon(SSBranchT *b,
                                 std::vector<int> &segs1st,
                                 std::vector<int> &segs2nd,
                                 std::vector<Pnt3> &branch_polgyon,
                                 std::vector<int> &branch_silidx);

    void label_convexity();
    void find_reflex_clusters();
    void set_id_sharp_clusters();
    void cut(int i0, int i1, SSBranchT *b);
    int convexity(const Pnt3 &p1, const Pnt3 &p2, const Pnt3 &p3);
    double vert_angle(int id) { return verts_[id].angle(); }

    // input
    const std::vector<Pnt3> &polygon_;
    std::list<SSBranchT> &branches_; // branch will be sorted

    // intermediate
    BBox2 bbox_;
    double diag_len_;
    std::vector<ReflexCluster> clusters_;
    std::unordered_map<SSBranchT *, bool> isCut_;
    std::unordered_map<SSBranchT *, std::vector<Slice>> branch_slices_;
    std::unordered_map<SSBranchT *, std::vector<Slice>> branch_vslices_; // vertical slices: make the central axis as a horizontal line
    std::unordered_map<SSBranchT *, CSpline> branch_splines_;
    std::unordered_map<SSBranchT *, std::vector<Pnt3>> branch_densamples_; // dense samples from spline
    std::unordered_map<SSBranchT *, std::vector<Pnt3>> branch_unisamples_;

    std::unordered_map<SSBranchT *, std::vector<Pnt3>> branch_polygons_;
    std::unordered_map<SSBranchT *, std::vector<Pnt3>> branch_smthpolys_;
    std::unordered_map<SSBranchT *, std::vector<int>> branch_segs1st_;
    std::unordered_map<SSBranchT *, std::vector<int>> branch_segs2nd_;
    // branch_segs1st_ + continous idx + branch_segs2nd_ + (the next idx of branch_segs2nd (maybe))
    std::unordered_map<SSBranchT *, std::vector<int>> branch_silidxs_;

    // output
    std::vector<PVert> verts_;
};