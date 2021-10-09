#pragma once
#include <vector>
#include <list>
#include <cstdio>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include "Skeleton.h"
#include "common.h"

enum DegreeOfFreedomType
{
    // readBF, writeBF already expect this order
    DOF_TYPE_FIXED_POSITION = 0,
    DOF_TYPE_FREE = 1,
    DOF_TYPE_FIXED_ALL = 2,
    DOF_TYPE_FIXED_LINEAR = 3,
    NUM_DOF_TYPES = 4
};

static std::vector<std::string> DofTypeNames = {
    "DOF_TYPE_FIXED_POSITION",
    "DOF_TYPE_FREE",
    "DOF_TYPE_FIXED_ALL",
    "DOF_TYPE_FIXED_LINEAR"};

typedef Eigen::Transform<double, 3, Eigen::Affine> Tform3;
typedef Eigen::Quaterniond Quat;

class Bone
{
public:
    // Positional offset from parent bone, if parent is NULL then offset is
    // taken from origin
    Eigen::Vector3d offset;
    // Current rotation applied to this bone
    Eigen::Quaterniond rotation;
    // Current translation applied to this bone, usually for non-roots this
    // will be (0,0,0)
    Eigen::Vector3d translation;
    // Current stretch applied to this bone, scalar value where 1.0 means
    // identity stretch
    double stretch;
    // Current twist angle applied to this bone, for special use with STBS, in
    // an LBS rig this will always be 0 and twisting must be "handled" by
    // rotation, scalar radian angle where 0 means no twist
    double twist;

    DegreeOfFreedomType tip_dof_type;
    // Last transformation passed from skinning program (what was actually used
    // in skinning regardless of the transformation stored above), used for
    // drawing and picking
    Tform3 last_T;
    // Container class parent
    Skeleton<Bone> *skel;

    // Parent Bone, NULL only if this bone is a root
    Bone *parent = nullptr;
    // Index in weights associated with this rig
    int wi;
    // Children bones, may be empty
    std::vector<Bone *> children;

    // Whether bone is currently draggin out a rotation
    bool dragging_rotation;

    int subtoy_id = -1;
    int level = 0; // 0: major branch, 1 sub-branch
    bool isAttachBone = false;
    bool isAttachBoneActivated = false;

public:
    // Inputs:
    //   parent_  parent bone {NULL}
    //   offset_  offset from parent bone {(0,0,0)}
    Bone(
        Skeleton<Bone> *skel,
        Bone *parent_,
        const Eigen::Vector3d &offset_,
        int subtoy_id_ = -1);

    // Safe copy constructor, copy transformations etc but not heirarchy
    // information.
    // Inputs:
    //   that  other bone
    Bone(const Bone *that);
    ~Bone();

    // Sets the weight index of this bone. Results in error if bone is root
    // or weight is less than 0
    // Inputs:
    //   wi  new weight index for this bone
    // Returns this
    Bone *set_wi(int wi);
    // Gets the weights index of this bone. Results in error if wi has not yet
    // been set, if this is a root, or if wi is non-negative
    // Returns this->wi
    int get_wi() const;
    // Returns true only if wi has been set
    bool wi_is_set() const;

    Bone *set_parent(Bone *parent);

    // Returns pointer to parent, NULL for roots
    const Bone *get_parent() const;
    // Get list of children bones
    const std::vector<Bone *> &get_children() const;
    // Returns true if is root (aka parent == null)
    bool is_root() const;
    // Reset transformation to identity
    void reset();
    // Split this bone into two
    Bone *split();

    // Forward kinematics----------------------------------------------------------
    // Computes the current orientation via Forward kinematics, i.e composing
    // orientations starting from the root.
    // Returns quaternion representation of this bones orientation in 3d
    Quat orientation() const;

    // Inputs:
    //  according_to_last_T  whether to use last_T or bone's transformation
    //  average_children_tails  whether to use an average of what this bone's
    //    children think it is or use this bone's position
    // Returns the current position of the tip of this bone, the point shared
    // by this bone and its children if any
    Eigen::Vector3d tip(
        const bool according_to_last_T,
        const bool average_children_tails) const;
    // Compute tip as where it is being drawn
    Eigen::Vector3d tip_as_drawn() const;
    // Returns the rest position of the tip of this bone.
    Eigen::Vector3d rest_tip() const;
    // Inputs:
    //  according_to_last_T  whether to use last_T or bone's transformation
    // Returns the current position of the tail of this non-root bone, the
    // point shared by this bone and its parent.
    // Error if this is a root bone
    Eigen::Vector3d tail(const bool according_to_last_T) const;
    // Compute tail as where it is being drawn
    Eigen::Vector3d tail_as_drawn() const;
    // Returns the rest position of the ail of this non-root bone.
    Eigen::Vector3d rest_tail() const;
    // Returns the current affine transformation via Forward Kinematics of this
    // bone
    Eigen::Transform<double, 3, Eigen::Affine> affine() const;
    // Returns the current rotated frame via Forward Kinematics of this
    // bone
    Eigen::Quaterniond rotated_frame() const;
    // Apply affine transformation to offsets of descendents
    // Inputs:
    //    A  affine transformation
    //
    void apply_affine_to_kin(const Tform3 &A);

    //---------------------------User Interaction---------------------------------
    static const double BONE_WI_UNSET;
    static const double BONE_POINT_RADIUS;
    static const double BONE_DIRECTED_LINE_SEGMENT_WIDTH;

    static double POINT_COLOR[3];
    static double FREE_POINT_COLOR[3];
    static double FIXED_POINT_COLOR[3];
    static double LINEAR_POINT_COLOR[3];
    static double ATTACH_BONE_COLOR[3];

    static double DIRECTED_LINE_SEGMENT_COLOR[3];
    static double SELECTED_DIRECTED_LINE_SEGMENT_COLOR[3];

    static double DRAG_LINE_GUIDE_COLOR[3];
    static double DRAG_LINE_COLOR[3];

    // Keep track if line segment or tip is currently selected or hovered over
    // Keep track of what used to be selected before processing click
    bool is_line_segment_selected;
    bool is_line_segment_hover;
    bool was_line_segment_selected;
    bool is_tip_selected;
    bool is_tip_hover;
    bool was_tip_selected;
    bool is_tip_changed = true;
    // True if down has been called after last up
    bool is_down;
    bool is_selected;
    // Mouse location at last down or drag call
    int last_x;
    int last_y;
    // Mouse location at down
    int down_x;
    int down_y;

    // sx, sy: screen-space coordinate
    bool down(int sx, int sy,
              int width, int height,
              float *mvpMatrix,
              bool shift_down, bool ctrl_down);

    void up();

    bool drag(int sx, int sy,
              int width, int height,
              float *viewMatrix, float *mvpMatrix,
              bool right_click, bool shift_down, bool ctrl_down);

    void tip_color(double pcolor[3]) const;
    void line_segment_color(double lcolor[3]) const;
    void translate(const Vec3 &trans_update)
    {
        offset += trans_update;
        is_tip_changed = true;
    }

    //---------------------------User Interaction----------------------------------end
    // Debug
    void print();
};

// ToDO: test
// Gather a list of pointers to all bone roots and their ancestors from a list
// of bones
//
// It's your funeral if your bone roots have closed loops
//
// Inputs:
//   BR  list of bone roots
// Returns list of bone pointers in what should be considered RANDOM ORDER
//
inline std::vector<Bone *> gather_bones(const std::vector<Bone *> &BR)
{
    std::list<Bone *> Q;
    for (std::vector<Bone *>::const_iterator bit = BR.begin();
         bit != BR.end(); bit++)
    {
        Q.push_back(*bit);
    }

    std::vector<Bone *> B;
    while (!Q.empty())
    {
        Bone *b = Q.front();
        Q.pop_front();
        B.push_back(b);
        // Add children to queue
        std::vector<Bone *> children = b->get_children();
        Q.insert(Q.end(), children.begin(), children.end());
    }
    return B;
}

// Gather a list of pointers to all bone roots and their ancestors from a list
// of bones
//
// It's your funeral if your bone roots have closed loops
//
// Inputs:
//   BR  list of bone roots
// Returns list of bone pointers in what should be considered RANDOM ORDER
//
inline std::vector<Bone *> gather_bones(const std::vector<Bone *> &BR, int subtoy_id)
{
    std::list<Bone *> Q;
    for (std::vector<Bone *>::const_iterator bit = BR.begin();
         bit != BR.end(); bit++)
    {
        Q.push_back(*bit);
    }

    std::vector<Bone *> B;
    while (!Q.empty())
    {
        Bone *b = Q.front();
        Q.pop_front();
        if (b->subtoy_id == subtoy_id)
            B.push_back(b);
        // Add children to queue
        std::vector<Bone *> children = b->get_children();
        Q.insert(Q.end(), children.begin(), children.end());
    }
    return B;
}

inline std::vector<Bone *> gather_descendants(Bone *BR)
{
    std::list<Bone *> Q;
    Q.push_back(BR);

    std::vector<Bone *> B;
    while (!Q.empty())
    {
        Bone *b = Q.front();
        Q.pop_front();
        B.push_back(b);
        // Add children to queue
        std::vector<Bone *> children = b->get_children();
        Q.insert(Q.end(), children.begin(), children.end());
    }
    return B;
}
// Read a bones forest from a file, returns a list of bone roots
// Input:
//   file_name  path to .bf bones tree file
//   skel  skeleton container
// Output:
//   BR  list of bone roots
// Returns true on success, false on errors
inline bool read_BF(const char *file_name, Skeleton<Bone> *skel, std::vector<Bone *> &BR)
{
    FILE *fp = fopen(file_name, "r");
    if (NULL == fp)
    {
        printf("IOError: read_BF could not open %s", file_name);
        return false;
    }
    destroy_bone_roots(BR);

    std::vector<Bone *> bones;
    std::vector<int> parent_indices;

    const int MAX_LINE_LENGTH = 500;
    char line[MAX_LINE_LENGTH];
    int lineno = 1;
    int max_wi = -1;

    while (fgets(line, MAX_LINE_LENGTH, fp) != NULL)
    {
        int wi;
        int parent_index;
        Eigen::Vector3d offset;
        Eigen::Quaterniond rotation(1, 0, 0, 0);
        Eigen::Vector3d translation(0, 0, 0);
        double stretch = 1.0;
        double twist = 0.0;
        Tform3 T(Tform3::Identity());
        // 0 - DOF_TYPE_FIXED_POSITION,
        // 1 - DOF_TYPE_FREE,
        // 2 - DOF_TYPE_FIXED_ALL,
        // 3 - DOF_TYPE_FIXED_LINEAR
        int dof_type = -1;
        // number of variables read
        int numvar;
        numvar = sscanf(line,
                        "%d %d "
                        "%lg %lg %lg "
                        "%lg %lg %lg %lg "
                        "%lg %lg %lg "
                        "%lg %lg "
                        "%lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg "
                        "%d",
                        &wi, &parent_index,
                        &offset(0), &offset(1), &offset(2),
                        &rotation.w(), &rotation.x(), &rotation.y(), &rotation.z(),
                        &translation(0), &translation(1), &translation(2),
                        &stretch, &twist,
                        &T.affine()(0, 0), &T.affine()(0, 1), &T.affine()(0, 2), &T.affine()(0, 3),
                        &T.affine()(1, 0), &T.affine()(1, 1), &T.affine()(1, 2), &T.affine()(1, 3),
                        &T.affine()(2, 0), &T.affine()(2, 1), &T.affine()(2, 2), &T.affine()(2, 3),
                        &dof_type);

        if (numvar < 26)
        {
            // Legacy
            int numvar = sscanf(line, "%d %d %lg %lg %lg",
                                &wi,
                                &parent_index,
                                &offset(0), &offset(1), &offset(2));
            if (5 != numvar)
            {
                fprintf(stderr, "ERROR: read_BF() bad format on line %d", lineno);
                return false;
            }
        }

        Bone *b;
        if (parent_index == -1)
        {
            //Root
            BR.insert(BR.end(), new Bone(skel, NULL, offset));
        }
        else if (parent_index < 0)
        {
            fprintf(stderr,
                    "ERROR: read_BF() bad format on line %d,"
                    " parent index (%d) should be -1 or >=0",
                    lineno,
                    parent_index);
            fclose(fp);
            return false;
        }
        else
        {
            b = new Bone(skel, NULL, offset);
        }
        max_wi = std::max(wi, max_wi);
        if (wi >= -1)
        {
            b->set_wi(wi);
        }
        b->rotation = rotation;
        b->translation = translation;
        b->stretch = stretch;
        b->twist = twist;
        b->last_T = T;

        if (numvar >= 27)
        {
            switch (dof_type)
            {
            case 0:
                b->tip_dof_type = DOF_TYPE_FIXED_POSITION;
                break;
            case 1:
                b->tip_dof_type = DOF_TYPE_FREE;
                break;
            case 2:
                b->tip_dof_type = DOF_TYPE_FIXED_ALL; //ToDO: figure this out
                break;
            case 3:
                b->tip_dof_type = DOF_TYPE_FIXED_LINEAR;
                break;
            default:
                fprintf(stderr, "Error: read_BF.h: unsupported dof_type %d\n",
                        dof_type);
                fclose(fp);
                return false;
            }
        }
        bones.push_back(b);
        parent_indices.push_back(parent_index);
        lineno++;
    }
    printf("bones.size(): %d\n", (int)bones.size());
    fclose(fp);

    std::vector<bool> weight_index_taken;
    if (max_wi > 0)
    {
        weight_index_taken.resize(max_wi + 1, false);
    }
    // loop over bones and parent indices
    for (int i = 0; i < (int)bones.size(); i++)
    {
        if (parent_indices[i] != -1)
        {
            if (parent_indices[i] >= (int)bones.size())
            {
                fprintf(stderr,
                        "ERROR: read_BF() bad format on line %d,"
                        " parent index (%d) should be -1 or <#bones (%d)",
                        i,
                        parent_indices[i],
                        (int)bones.size());
                return false;
            }

            bones[i]->set_parent(bones[parent_indices[i]]);
        }
    }
    return true;
}

// // C: control point position (Bone Tip)
// // BE: index into C constructing Bone Edges
// inline bool gather_bone_handles(const std::vector<Bone *> &BR,
//                                 Eigen::MatrixXd &C,
//                                 Eigen::MatrixXi &BE)
// {
//     using namespace std;
//     using namespace Eigen;

//     vector<Bone *> bones = gather_bones(BR);

//     C.resize(bones.size(), 3);
//     BE.resize(bones.size() - BR.size(), 2);
//     int bidx = 0;
//     for (int i = 0; i < (int)bones.size(); i++)
//     {
//         Bone *b = bones[i];
//         Vector3d tip = b->rest_tip();
//         C(i, 0) = tip(0);
//         C(i, 1) = tip(1);
//         C(i, 2) = tip(2);
//         if (b->get_parent() != nullptr)
//         {
//             int parent_idx = -1;
//             for (int j = 0; j < bones.size(); j++)
//             {
//                 if (bones[j] == b->get_parent())
//                 {
//                     parent_idx = j;
//                     break;
//                 }
//             }
//             assert(parent_idx != -1);

//             BE(bidx, 0) = parent_idx;
//             BE(bidx, 1) = i;
//             bidx++;
//         }
//     }
// }

inline bool write_BF(const char *file_name, std::vector<Bone *> &BR)
{

    FILE *fp = fopen(file_name, "w");
    if (NULL == fp)
    {
        printf("IOError: write_BF could not open %s", file_name);
        return false;
    }
    std::vector<Bone *> B = gather_bones(BR);
    // Build map from bone pointers to index in B
    std::map<const Bone *, int> B2I;
    // Map NULL to -1
    B2I[NULL] = -1;
    int i = 0;
    for (std::vector<Bone *>::iterator bit = B.begin(); bit != B.end(); bit++)
    {
        B2I[*bit] = i;
        i++;
    }
    // Print line for each bone
    for (std::vector<Bone *>::iterator bit = B.begin(); bit != B.end(); bit++)
    {
        Bone *b = (*bit);
        Tform3 T = b->last_T;
        int dof_type = 0;
        switch (b->tip_dof_type)
        {
        case DOF_TYPE_FIXED_POSITION:
            dof_type = 0;
            break;
        case DOF_TYPE_FREE:
            dof_type = 1;
            break;
        case DOF_TYPE_FIXED_ALL:
            dof_type = 2;
            break;
        case DOF_TYPE_FIXED_LINEAR:
            dof_type = 3;
            break;
        default:
            assert(false);
        }
        fprintf(fp,
                "%d %d "
                "%lg %lg %lg "
                "%lg %lg %lg %lg "
                "%lg %lg %lg "
                "%lg %lg "
                "%lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg "
                "%d "
                "\n",
                b->get_wi(), B2I[b->get_parent()],
                b->offset[0], b->offset[1], b->offset[2],
                b->rotation.w(), b->rotation.x(), b->rotation.y(), b->rotation.z(),
                b->translation.x(), b->translation.y(), b->translation.z(),
                b->stretch, b->twist,
                T.affine()(0, 0), T.affine()(0, 1), T.affine()(0, 2), T.affine()(0, 3),
                T.affine()(1, 0), T.affine()(1, 1), T.affine()(1, 2), T.affine()(1, 3),
                T.affine()(2, 0), T.affine()(2, 1), T.affine()(2, 2), T.affine()(2, 3),
                dof_type);
    }
    fclose(fp);
    return true;
}

// Check whether 2 sets of bone roots contain the same tree structure
// Inputs:
//   AR first set of bone roots
//   BR second set of bone roots
inline bool bone_roots_match(
    const std::vector<Bone *> &AR,
    const std::vector<Bone *> &BR)
{
    using namespace std;
    if (AR.size() != BR.size())
    {
        // different number of roots
        return false;
    }
    // Insert roots into search queue
    std::list<Bone *> AQ;
    std::list<Bone *> BQ;
    for (
        std::vector<Bone *>::const_iterator ait = AR.begin();
        ait != AR.end();
        ait++)
    {
        AQ.push_back(*ait);
    }
    for (
        std::vector<Bone *>::const_iterator bit = BR.begin();
        bit != BR.end();
        bit++)
    {
        BQ.push_back(*bit);
    }
    assert(AQ.size() == BQ.size());

    // Keep track of all bones that get popped
    std::vector<Bone *> A;
    std::vector<Bone *> B;
    while (!AQ.empty() && !BQ.empty())
    {
        assert(AQ.size() == BQ.size());
        Bone *a = AQ.back();
        AQ.pop_back();
        Bone *b = BQ.back();
        BQ.pop_back();
        // Add to list
        A.push_back(a);
        B.push_back(b);
        // get children
        std::vector<Bone *> b_children = b->get_children();
        std::vector<Bone *> a_children = a->get_children();
        // a and b should have same number of children
        if (a_children.size() != b_children.size())
        {
            return false;
        }
        // Add children to queue
        AQ.insert(AQ.end(), a_children.begin(), a_children.end());
        BQ.insert(BQ.end(), b_children.begin(), b_children.end());
    }
    assert(AQ.size() == BQ.size());
    assert(A.size() == B.size());
    return true;
}
