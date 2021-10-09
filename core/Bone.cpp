#include <vector>

#include "Bone.h"
#include "EGL.h"

using namespace Eigen;
using namespace std;

template <typename Scalar>
static void project_to_line(
    const Scalar px,
    const Scalar py,
    const Scalar pz,
    const Scalar sx,
    const Scalar sy,
    const Scalar sz,
    const Scalar dx,
    const Scalar dy,
    const Scalar dz,
    Scalar &projpx,
    Scalar &projpy,
    Scalar &projpz,
    Scalar &t,
    Scalar &sqrd)
{
    // vector from source to destination
    Scalar dms[3];
    dms[0] = dx - sx;
    dms[1] = dy - sy;
    dms[2] = dz - sz;
    Scalar v_sqrlen = dms[0] * dms[0] + dms[1] * dms[1] + dms[2] * dms[2];
    // line should have some length
    assert(v_sqrlen != 0);
    // vector from point to source
    Scalar smp[3];
    smp[0] = sx - px;
    smp[1] = sy - py;
    smp[2] = sz - pz;
    t = -(dms[0] * smp[0] + dms[1] * smp[1] + dms[2] * smp[2]) / v_sqrlen;
    // P projectred onto line
    projpx = (1.0 - t) * sx + t * dx;
    projpy = (1.0 - t) * sy + t * dy;
    projpz = (1.0 - t) * sz + t * dz;
    // vector from projected point to p
    Scalar pmprojp[3];
    pmprojp[0] = px - projpx;
    pmprojp[1] = py - projpy;
    pmprojp[2] = pz - projpz;
    sqrd = pmprojp[0] * pmprojp[0] + pmprojp[1] * pmprojp[1] + pmprojp[2] * pmprojp[2];
}

template <typename Scalar>
static void project_to_line(
    const Scalar px,
    const Scalar py,
    const Scalar pz,
    const Scalar sx,
    const Scalar sy,
    const Scalar sz,
    const Scalar dx,
    const Scalar dy,
    const Scalar dz,
    Scalar &t,
    Scalar &sqrd)
{
    Scalar projpx;
    Scalar projpy;
    Scalar projpz;
    return project_to_line(
        px, py, pz, sx, sy, sz, dx, dy, dz,
        projpx, projpy, projpz, t, sqrd);
}

static bool inside_point(
    const double qx,
    const double qy,
    const double cx,
    const double cy,
    double r)
{
    return (qx - cx) * (qx - cx) + (qy - cy) * (qy - cy) - r * r < 0;
}

static bool inside_line_segment(const double qx,
                                const double qy,
                                const double sx,
                                const double sy,
                                const double dx,
                                const double dy,
                                double r)
{
    if (inside_point(qx, qy, sx, sy, r))
        return true;
    if (inside_point(qx, qy, dx, dy, r))
        return true;

    double t, sqrd;
    project_to_line(
        qx, qy, 0.0,
        sx, sy, 0.0,
        dx, dy, 0.0,
        t, sqrd);
    return (t <= 1 && t >= 0) && (sqrd <= r * r);
}

const double Bone::BONE_WI_UNSET = -3;
const double Bone::BONE_POINT_RADIUS = 10;
// const double Bone::BONE_POINT_RADIUS = 5;
const double Bone::BONE_DIRECTED_LINE_SEGMENT_WIDTH = 3;

// double Bone::POINT_COLOR[3] = {239. / 255., 213. / 255., 46. / 255.};
// double Bone::POINT_COLOR[3] = {0., 0., 1.};
double Bone::POINT_COLOR[3] = {102. / 255., 178. / 255., 255. / 255.};
double Bone::FREE_POINT_COLOR[3] = {113. / 255., 239. / 255., 46. / 255.};
double Bone::FIXED_POINT_COLOR[3] = {239. / 255., 113. / 255., 46. / 255.};
double Bone::LINEAR_POINT_COLOR[3] = {253. / 255., 130. / 255., 244. / 255.};
double Bone::ATTACH_BONE_COLOR[3] = {1., 0., 0.};

double Bone::DIRECTED_LINE_SEGMENT_COLOR[3] = {106., 106., 255.};
double Bone::SELECTED_DIRECTED_LINE_SEGMENT_COLOR[3] = {
    DIRECTED_LINE_SEGMENT_COLOR[0] * 0.5,
    DIRECTED_LINE_SEGMENT_COLOR[1] * 0.5,
    DIRECTED_LINE_SEGMENT_COLOR[2] * 0.5};

double Bone::DRAG_LINE_GUIDE_COLOR[3] = {106., 106., 255.};
double Bone::DRAG_LINE_COLOR[3] = {239., 46., 46.};

Bone::Bone(
    Skeleton<Bone> *skel_,
    Bone *parent_,
    const Eigen::Vector3d &offset_,
    int subtoy_id_) : subtoy_id(subtoy_id_),
                      offset(offset_),
                      rotation(Quaterniond(1, 0, 0, 0)),
                      translation(Vector3d(0, 0, 0)),
                      stretch(1.0),
                      twist(0),
                      tip_dof_type(DOF_TYPE_FIXED_POSITION),
                      last_T(Tform3::Identity()),
                      skel(skel_),
                      parent(NULL),
                      wi(BONE_WI_UNSET),
                      is_line_segment_selected(false),
                      is_line_segment_hover(false),
                      is_tip_selected(false),
                      is_tip_hover(false),
                      is_selected(false),
                      is_down(false),
                      dragging_rotation(false)
{
    if (parent_ != NULL)
    {
        std::vector<Bone *> B = gather_bones(skel->roots);
        set_wi(B.size() - skel->roots.size());
        set_parent(parent_);
        // M_DEBUG << this
        //         << " I am a child, my wi is " << get_wi() << " my parent's wi is " << parent_->get_wi()
        //         << " subtoy_id=" << subtoy_id << endl;
    }
    else
    {
        set_wi(-1);
        // M_DEBUG << this
        //         << " I am a root, subtoy_id=" << subtoy_id << endl;
    }
}

Bone::Bone(const Bone *that) : parent(NULL)
{
    skel = that->skel;
    // Copy everything that is safe to copy
    offset = that->offset;
    rotation = that->rotation;
    translation = that->translation;
    stretch = that->stretch;
    twist = that->twist;

    is_line_segment_selected = that->is_line_segment_selected;
    is_line_segment_hover = that->is_line_segment_hover;
    is_tip_selected = that->is_tip_selected;
    is_tip_hover = that->is_tip_hover;
    tip_dof_type = that->tip_dof_type;
    last_T = that->last_T;
    wi = that->wi;
    dragging_rotation = that->dragging_rotation;

    down_x = that->down_x;
    down_y = that->down_y;
    last_x = that->last_x;
    last_y = that->last_y;
}

Bone::~Bone()
{
    // Tell parent that I'm dead
    if (parent != NULL)
    {
        parent->children.erase(
            remove(parent->children.begin(), parent->children.end(), this),
            parent->children.end());
    }
    // Delete children (assuming they will tell me that they died)
    while (children.size() > 0)
    {
        delete (*children.begin());
    }
    // cout << __FILE__ << " " << __LINE__ << "Bone destroyed this=" << this << endl;
}

Bone *Bone::set_wi(int wi)
{
    this->wi = wi;
    return this;
}

int Bone::get_wi() const
{
    return wi;
}

bool Bone::wi_is_set() const
{
    return wi != BONE_WI_UNSET;
}

Bone *Bone::set_parent(Bone *parent)
{
    // Don't make roots this way
    assert(parent != NULL);
    // Only let this happen once!
    assert(this->parent == NULL);
    this->parent = parent;
    // Tell parent she has just given birth
    this->parent->children.push_back(this);

    return this;
}

const Bone *Bone::get_parent() const
{
    return this->parent;
}

const std::vector<Bone *> &Bone::get_children() const
{
    return children;
}

bool Bone::is_root() const
{
    return parent == NULL;
}

void Bone::reset()
{
    translation = Vector3d(0, 0, 0);
    rotation = Quaterniond(1, 0, 0, 0);
    stretch = 1.0;
    twist = 0.0;
    last_T = Tform3::Identity();
    is_tip_changed = true;
}

Bone *Bone::split()
{
    Bone *newBone = nullptr;
    // find sel_bone's parent
    // find sel_bone's children
    // detach sel_bone's children from sel_bone
    // create new_bone
    // set sel_bone as new_bone's parent
    // set new_bone's children as sel_bone's children
    if (parent)
    {
        std::vector<Bone *> children_tmp;
        for (Bone *b : children)
        {
            children_tmp.push_back(b);
            b->parent = nullptr;
        }
        children.clear();
        Vector3d off = 0.5 * (rest_tip() - rest_tail());
        this->offset = off;
        newBone = new Bone(skel, this, off);
        for (Bone *b : children_tmp)
        {
            b->set_parent(newBone);
        }
        newBone->set_wi(gather_bones(skel->roots).size() - skel->roots.size());
    }
    return newBone;
}

// Forward kinematics----------------------------------------------------------
// Computes the current orientation via Forward kinematics, i.e composing
// orientations starting from the root.
// Returns quaternion representation of this bones orientation in 3d
Eigen::Quaterniond Bone::orientation() const
{
    // assert(stretch == 1);
    // assert(twist == 0);
    Quat p_orientation;
    if (parent == NULL)
    {
        // Root base case
        // Identity rotation
        p_orientation = Quat(1, 0, 0, 0);
    }
    else
    {
        p_orientation = parent->orientation();
    }
    // Compose with this rotation
    return p_orientation * rotation;
}

// Inputs:
//  according_to_last_T  whether to use last_T or bone's transformation
//  average_children_tails  whether to use an average of what this bone's
//    children think it is or use this bone's position
// Returns the current position of the tip of this bone, the point shared
// by this bone and its children if any
Eigen::Vector3d Bone::tip(
    const bool according_to_last_T,
    const bool average_children_tails) const
{
    // assert(stretch == 1);
    // assert(twist == 0);
    if (average_children_tails && children.size() > 0)
    {
        Vector3d t(0, 0, 0);
        vector<Bone *> children = get_children();
        for (
            vector<Bone *>::iterator cit = children.begin();
            cit != children.end();
            cit++)
        {
            assert((*cit)->parent == this);
            t += (*cit)->tail(according_to_last_T);
        }
        t /= children.size();
        // cout << __FILE__ << " " << __LINE__ << " average_children_tails: " << t.transpose() << endl;
        return t;
    }
    else
    {
        if (according_to_last_T)
        {
            // cout << __FILE__ << " " << __LINE__ << " according_to_last_T: " << according_to_last_T << endl;
            return last_T * rest_tip();
        }
        else
        {
            // cout << __FILE__ << " " << __LINE__ << " not according_to_last_T" << endl;
            return affine() * rest_tip();
        }
    }
}

Eigen::Vector3d Bone::tip_as_drawn() const
{
    bool according_to_last_T = skel->draw_according_to_last_T;
    bool average_children_tails =
        skel->average_children_tails_to_draw_non_weighted_roots &&
        is_root() && wi < 0;
    return tip(according_to_last_T, average_children_tails);
}

// Inputs:
//  according_to_last_T  whether to use last_T or bone's transformation
// Returns the current position of the tail of this non-root bone, the
// point shared by this bone and its parent.
// Error if this is a root bone
Eigen::Vector3d Bone::tail(const bool according_to_last_T) const
{
    // assert(stretch == 1);
    // assert(twist == 0);
    assert(parent != NULL);
    if (according_to_last_T)
    {
        return last_T * parent->rest_tip();
    }
    else
    {
        return affine() * parent->rest_tip();
    }
}

Eigen::Vector3d Bone::tail_as_drawn() const
{
    if (skel->draw_connected_to_parent)
    {
        return parent->tip_as_drawn();
    }
    return tail(skel->draw_according_to_last_T);
}

// Returns the rest position of the tip of this bone.
Eigen::Vector3d Bone::rest_tip() const
{
    Vector3d p_rest_tip;
    if (parent == NULL)
    {
        // Root base case
        p_rest_tip = Vector3d(0, 0, 0);
    }
    else
    {
        p_rest_tip = parent->rest_tip();
    }
    return p_rest_tip + offset;
}

// Returns the rest position of the ail of this non-root bone.
Eigen::Vector3d Bone::rest_tail() const
{
    assert(parent != NULL);
    return parent->rest_tip();
}

// Returns the current affine transformation via Forward Kinematics of this
// bone
Eigen::Transform<double, 3, Eigen::Affine> Bone::affine() const
{
    // assert(stretch == 1);
    // assert(twist == 0);
    Tform3 p_affine;
    if (parent == NULL)
    {
        p_affine.setIdentity();
    }
    else
    {
        p_affine = parent->affine();
    }
    return p_affine.rotate(rotation).translate(translation);
}

// Returns the current rotated frame via Forward Kinematics of this
// bone
Eigen::Quaterniond Bone::rotated_frame() const
{
    assert(twist == 0);
    Quat q(1, 0, 0, 0);
    if (parent != NULL)
    {
        q = parent->rotated_frame();
    }
    return q * rotation;
}

// Apply affine transformation to offsets of descendents
// Inputs:
//    A  affine transformation
//
void Bone::apply_affine_to_kin(const Tform3 &A)
{
    vector<Bone *> children = get_children();
    Tform3 B = A;
    B.translation() = Vector3d(0, 0, 0);
    for (
        vector<Bone *>::iterator cit = children.begin();
        cit != children.end();
        cit++)
    {
        (*cit)->offset = A * (*cit)->offset;
        (*cit)->is_tip_changed = true;
        // recursive call
        (*cit)->apply_affine_to_kin(B);
    }
}

bool Bone::down(int sx, int sy, int width, int height,
                float *mvpMatrix, bool shift_down, bool ctrl_down)
{
    // Keep track of click position
    down_x = sx;
    down_y = sy;
    last_x = sx;
    last_y = sy;

    // reset to false on each mouse down
    is_tip_hover = false;
    is_line_segment_hover = false;

    // Keep track of what used to be selected
    was_tip_selected = is_tip_selected;
    was_line_segment_selected = is_line_segment_selected;

    // Check endpoint //ToDO: consider z
    Vector3d d = tip_as_drawn();
    Vector3d sd;
    EGL::project(d.x(), d.y(), d.z(), width, height, mvpMatrix, sd[0], sd[1], sd[2]);

    bool in_tip = inside_point(sx, sy, sd.x(), sd.y(), BONE_POINT_RADIUS);

    if (skel->get_editing() && in_tip && shift_down)
    {
        // M_DEBUG << " Add new bone " << endl;
        std::vector<Bone *> B = gather_bones(skel->roots);
        Vector3d off(0, 0, 0);
        Bone *b = new Bone(skel, this, off);
        // b->set_wi(B.size());
        b->set_wi(B.size() - skel->roots.size());
        b->is_tip_selected = true;
        b->is_selected = true;
        b->is_down = true;
        // Mark this as not selected
        is_tip_selected = false;
        is_selected = false;
        is_down = false;
        // skel->print(); //ToDO
        return true;
    }
    // Check bone segment
    bool in_line_seg = false;
    if (!is_root() && !in_tip)
    {
        Vector3d s = tail_as_drawn(); // start
        Vector3d ss;
        EGL::project(s.x(), s.y(), s.z(), width, height, mvpMatrix, ss[0], ss[1], ss[2]); // screen coordinate for screen

        if (!inside_point(sx, sy, ss.x(), ss.y(), BONE_POINT_RADIUS))
        {
            bool bone_len = (ss.x() - sd.x()) * (ss.x() - sd.x()) + (ss.y() - sd.y()) * (ss.y() - sd.y());
            if (bone_len > 1e-3)
            {
                in_line_seg = inside_line_segment(
                    sx, sy,
                    ss.x(), ss.y(),
                    sd.x(), sd.y(), BONE_DIRECTED_LINE_SEGMENT_WIDTH);
            }
        }
    }

    is_tip_selected =
        (in_tip ? (ctrl_down ? !is_tip_selected : true)
                : ctrl_down && is_tip_selected);
    // // ToDO: fix the bug BBW flying
    // is_line_segment_selected =
    //     (in_line_seg ? (ctrl_down ? !is_line_segment_selected : true) : ctrl_down && is_line_segment_selected);

    is_selected = is_tip_selected || is_line_segment_selected;
    is_down = in_tip || in_line_seg;
    if (is_line_segment_selected)
    {
        M_DEBUG << "---------------->line_segment_selected" << endl;
        print();
    }

    // cout << __FILE__ << " " << __LINE__ << " "
    //      << " is_tip_selected=" << is_tip_selected << " is_line_segment_selected=" << is_line_segment_selected << endl;
    return is_down;
}

void Bone::up()
{
    is_down = false;
    dragging_rotation = false;
}

void Bone::tip_color(double pcolor[3]) const
{
    if (isAttachBone)
    {
        copy(FIXED_POINT_COLOR, FIXED_POINT_COLOR + 3, pcolor);
        return;
    }

    switch (tip_dof_type)
    {
    case DOF_TYPE_FIXED_ALL:
        copy(FIXED_POINT_COLOR, FIXED_POINT_COLOR + 3, pcolor);
        break;
    case DOF_TYPE_FREE:
        copy(FREE_POINT_COLOR, FREE_POINT_COLOR + 3, pcolor);
        break;
    case DOF_TYPE_FIXED_LINEAR:
        copy(LINEAR_POINT_COLOR, LINEAR_POINT_COLOR + 3, pcolor);
        break;
    case DOF_TYPE_FIXED_POSITION:
    default:
        copy(POINT_COLOR, POINT_COLOR + 3, pcolor);
    }
}

bool Bone::drag(int sx, int sy,
                int width, int height,
                float *viewMatrix, float *mvpMatrix,
                bool right_click, bool shift_down, bool ctrl_down)
{

    if (is_selected)
    {
        if (right_click)
        {
            dragging_rotation = true;
            // M_DEBUG << " Bone::drag rotation" << endl;
            if (is_tip_selected)
            {
                // M_DEBUG << " Bone::drag is_tip_selected" << endl;
                Vector3d axis;
                EGL::view_dir(viewMatrix, axis[0], axis[1], axis[2]);
                axis.normalize();

                // ToDO: change bone's dot version
                double angle = -2.0 * M_PI * 360.0 * ((sx - last_x) / 400.0) / 360.0;
                // Get change in rotation as quaternion
                Quat delta_rot = Quat(Eigen::AngleAxis<double>(angle, axis));
                // Offset *ALL* rotations so that rotations are about the parent (self in
                // case of roots)
                Vector3d tail = offset;
                if (!is_root())
                {
                    tail = parent->rest_tip();
                }

                if (skel->get_editing())
                {
                    // cout << __FILE__ << " " << __LINE__ << " Bone::drag is_editing" << endl;
                    Tform3 A(Tform3::Identity());
                    A.rotate(delta_rot);
                    if (!is_root())
                    {
                        // rotate my offset, if I'm not a root
                        offset = A * offset;
                    }
                    // rotate offsets of descendents
                    apply_affine_to_kin(A);
                }
                else
                {
                    Vector3d ot = (rotation * (translation + tail)) - tail;
                    // cout << __FILE__ << " " << __LINE__
                    //      << "tail=" << tail.transpose()
                    //      << "translation=" << translation.transpose()
                    //      << " Bone::drag rotation\n"
                    //      << delta_rot.matrix()
                    //      << " oriented translation=" << ot.transpose()
                    //      << endl;
                    rotation = delta_rot * rotation;
                    // center rotations around offset point for roots
                    translation = rotation.inverse() * (tail + ot) - tail;
                    // why rotation.inverse not rotation, because need mapping back to worldspace
                }

                is_tip_changed = true;
            }
            else if (is_line_segment_selected)
            {
                // twist about bone
                assert(!is_root());
                Vector3d d = rest_tip();
                Vector3d s = parent->rest_tip();
                Vector3d axis = s - d;
                double angle = -2.0 * M_PI * 360.0 * ((sx - last_x) / 400.0) / 360.0;
                // Offset *ALL* rotations so that rotations are about the parent (self in
                // case of roots)
                Vector3d tail = offset;
                Quat delta_rot = Quat(Eigen::AngleAxis<double>(angle, axis));
                if (!is_root())
                {
                    tail = parent->rest_tip();
                }

                if (skel->get_editing())
                {
                    // rotate offsets of descendents
                    Tform3 A(Tform3::Identity());
                    A.rotate(delta_rot);
                    if (!is_root())
                    {
                        // rotate my offset, if I'm not a root
                        offset = A * offset;
                    }
                    // rotate offsets of descendents
                    apply_affine_to_kin(A);
                }
                else
                {
                    Vector3d ot = (rotation * (translation + tail)) - tail;
                    rotation = delta_rot * rotation;
                    // center rotations around offset point for roots
                    translation = rotation.inverse() * (tail + ot) - tail; // why rotation.inverse not rotation
                }
            }
            else
            {
                assert(false);
            }
        }
        else
        {
            Vector3d old_tip = tip_as_drawn();
            Vector3d sold_tip;
            EGL::project(old_tip.x(), old_tip.y(), old_tip.z(), width, height, mvpMatrix, sold_tip[0], sold_tip[1], sold_tip[2]);
            double sz = sold_tip[2];
            // cout << __FILE__ << " " << __LINE__ << " old_tip=" << old_tip.transpose() << " sold_tip=" << sold_tip.transpose() << endl;

            Vector3d new_tip;
            EGL::unproject(sx, sy, sz, width, height, mvpMatrix, new_tip[0], new_tip[1], new_tip[2]);
            // cout << __FILE__ << " " << __LINE__ << " sx=" << sx << " sy=" << sy << " sz=" << sz << " width=" << width
            //      << " height=" << height << " mvpMatrix=\n"
            //      << Map<Matrix4f>(mvpMatrix) << endl;

            Vector3d at = new_tip - old_tip;
            Vector3d p_at(0, 0, 0);

            // cout << __FILE__ << " " << __LINE__ << ": new_tip=" << new_tip.transpose()
            //      << " old_tip=" << old_tip.transpose()
            //      << " sold_tip=" << sold_tip.transpose() << endl;

            if (parent != NULL)
            {
                p_at = parent->affine().translation();
            }
            Vector3d trans_update = orientation().inverse() * at;
            if (skel->get_editing())
            {
                if (is_line_segment_selected)
                {
                    parent->offset += trans_update;
                    parent->is_tip_selected = true;
                }
                else
                {
                    // M_DEBUG << "offset=" << offset.transpose() << ", trans_update=" << trans_update.transpose() << endl;
                    offset += trans_update;
                    is_tip_changed = true;
                }

                // Comment the following three lines to make all descendants move as this bone
                Tform3 A(Tform3::Identity());
                A.translate(-trans_update);
                apply_affine_to_kin(A);
            }
            else
            {
                // set translation to put rest tip at new tip position
                translation += trans_update;
                is_tip_changed = true;
                // cout << __FILE__ << " " << __LINE__ << " trans_update=" << trans_update.transpose() << " translation=" << translation.transpose() << endl;
            }
        }
    }

    last_x = sx;
    last_y = sy;
    return is_selected;
}

void Bone::line_segment_color(double lcolor[3]) const
{
    // Get line segment color
    copy(DIRECTED_LINE_SEGMENT_COLOR, DIRECTED_LINE_SEGMENT_COLOR + 3, lcolor);
    // if (is_line_segment_hover && !is_down)
    // {
    //     lcolor[0] += HOVER_COLOR_OFFSET[0];
    //     lcolor[1] += HOVER_COLOR_OFFSET[1];
    //     lcolor[2] += HOVER_COLOR_OFFSET[2];
    // }
    // else
    if (is_line_segment_selected)
    {
        copy(
            SELECTED_DIRECTED_LINE_SEGMENT_COLOR,
            SELECTED_DIRECTED_LINE_SEGMENT_COLOR + 3,
            lcolor);
    }
}

void Bone::print()
{
    printf("isRoot:%d wi=%d\n", is_root(), get_wi());
    printf("offset: %g %g %g\n", offset[0], offset[1], offset[2]);
    printf("translation: %g %g %g\n", translation[0], translation[1], translation[2]);
    printf("rotation: %g %g %g %g\n", rotation.w(), rotation.x(), rotation.y(), rotation.z());
    printf("stretch: %g\n", stretch);
    printf("twist: %g\n", twist);
    printf("\n");
}