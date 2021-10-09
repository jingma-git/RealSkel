#pragma once
#include <iostream>
#include <vector>
#include <algorithm>

#include "common.h"
#include "EGL.h"

template <typename BoneType>
class Skeleton;

template <typename BoneType>
void destroy_bone_roots(std::vector<BoneType *> &BR)
{
    // Clear bone roots list
    for (typename std::vector<BoneType *>::iterator bit = BR.begin(); bit != BR.end(); bit++)
    {
        delete *bit;
        *bit = nullptr;
    }
    BR.clear();
}

// No-op default
template <typename BoneType>
inline void no_op(const Skeleton<BoneType> &)
{
    return;
}

// Predicate to return bone is_selected field
template <typename BoneType>
inline bool is_bone_selected(BoneType *b)
{
    return b->is_selected;
}

template <typename BoneType>
class Skeleton
{
public:
    std::vector<BoneType *> roots;
    // Type definition for a callback that will receive a const reference to
    // this skeleton as input and returns a void pointer
    // Method
    typedef std::function<void(const Skeleton<BoneType> &)> SkeletonCallback;
    // Callback called after setting editing field
    SkeletonCallback after_set_editing; //ToDO: remove

    // Editting rest positon
    bool editing;
    // Is adding a Bone or Point Handle
    bool point_handle_mode = false;

public:
    Skeleton()
        : after_set_editing(no_op<BoneType>),
          draw_according_to_last_T(true),
          draw_connected_to_parent(false),
          average_children_tails_to_draw_non_weighted_roots(true) {}

    ~Skeleton()
    {
        destroy_bone_roots(roots);
    }

    // Draw bones (and also respond to UI) using last_T to define positions
    // rather than transformations stored in bones
    bool draw_according_to_last_T;
    // Draw bone segments such that they *always* connect this bone to parent,
    // regardless of stored transformations (i.e. non-trivial translations)
    bool draw_connected_to_parent;
    // Draw non-weighted roots as average of children's tails, otherwise do not
    // draw them
    bool average_children_tails_to_draw_non_weighted_roots;

    // // Inputs:
    // //   deselect_others deselect any other selected bones
    // // Return pointer to first bone found to be selected
    // // Returns NULL if no bones are selected
    // BoneType *find_selected(const bool deselect_others = false);
    // // Return a list of pointers to bones found to be selected
    // std::vector<BoneType *> find_all_selected();

    bool set_editing(const bool v)
    {
        using namespace std;
        editing = v;
        if (editing)
        {
            std::vector<BoneType *> B = gather_bones(roots);
            // call reset() for each bone
            std::for_each(B.begin(), B.end(), std::mem_fun(&BoneType::reset));
            // after_set_editing(*this); //ToDO: figure this out
        }
        return editing;
    }

    bool get_editing() const
    {
        return editing;
    }

    void reset()
    {
        vector<BoneType *> B = gather_bones(roots);
        for (typename vector<BoneType *>::iterator bit = B.begin();
             bit != B.end(); bit++)
        {
            BoneType *b = *bit;
            b->reset();
        }
    }

    BoneType *pick_bone(int sx, int sy, int width, int height, float *mvpMatrix, bool shift_down, bool ctrl_down)
    {
        using namespace std;
        using namespace Eigen;

        bool bone_down = false;
        bool reset_selection = true;

        vector<BoneType *> B = gather_bones(roots);
        vector<BoneType *> sel_bs; // selected bones
        BoneType *nearest_b = nullptr;
        double min_z = std::numeric_limits<double>::max();
        for (typename vector<BoneType *>::iterator bit = B.begin();
             bit != B.end(); bit++)
        {
            BoneType *b = *bit;
            if (b->down(sx, sy, width, height, mvpMatrix, shift_down, ctrl_down))
            {
                bone_down = true;
                sel_bs.push_back(b);

                Vector3d d = b->tip_as_drawn();
                Vector3d sd;
                EGL::project(d.x(), d.y(), d.z(), width, height, mvpMatrix, sd[0], sd[1], sd[2]);
                cout << __FILE__ << " " << __LINE__ << " " << b << " sd=" << sd.transpose() << endl;
                if (sd[2] < min_z)
                {
                    min_z = sd[2];
                    nearest_b = b;
                }
            }
        }

        if (sel_bs.size() > 1)
        {
            for (typename vector<BoneType *>::iterator bit = sel_bs.begin();
                 bit != sel_bs.end(); bit++)
            {
                BoneType *b = *bit;
                if (b != nearest_b)
                {
                    b->is_tip_selected = false;
                    b->is_line_segment_selected = false;
                    b->is_selected = false;
                    b->is_tip_changed = false;
                }
            }
        }

        // add new root bone
        if (editing && !bone_down && shift_down)
        {
            cout << __FILE__ << " " << __LINE__ << " Adding root bone..." << endl;
            Vector3d off;
            EGL::unproject(sx, sy, width, height, mvpMatrix, off[0], off[1], off[2]);
            BoneType *b = new BoneType(this, NULL, off);

            roots.insert(roots.end(), b);

            if (point_handle_mode)
            {
                b->set_wi(B.size());
            }
            else
            {
                b->set_wi(-1);
                if (b->down(sx, sy, width, height, mvpMatrix, shift_down, ctrl_down))
                {
                    bone_down = true;
                }
            }
            nearest_b = b->children[0];
        }
        return nearest_b;
    }

    bool drag_bone(int sx, int sy,
                   int width, int height,
                   float *viewMatrix, float *mvpMatrix,
                   bool right_click, bool shift_down, bool ctrl_down)
    {

        using namespace std;
        bool bone_drag = false;

        // get bones
        vector<BoneType *> B = gather_bones(roots);
        // check bones
        for (typename vector<BoneType *>::iterator bit = B.begin(); bit != B.end(); bit++)
        {
            BoneType *b = (*bit);
            if (b->drag(sx, sy, width, height, viewMatrix, mvpMatrix, right_click, shift_down, ctrl_down))
            {
                bone_drag = true;
            }
        }
        return bone_drag;
    }

    void up()
    {
        using namespace std;
        // get bones
        vector<BoneType *> B = gather_bones(roots);
        // check bones
        for (typename vector<BoneType *>::iterator bit = B.begin(); bit != B.end(); bit++)
        {
            BoneType *b = (*bit);
            b->up();
        }
    }

    BoneType *find_selected()
    {
        using namespace std;
        vector<BoneType *> B = gather_bones(roots);
        for (BoneType *b : B)
        {
            if (b->is_selected)
            {
                return b;
            }
        }
        return nullptr;
    }

    std::vector<BoneType *> find_all_selected()
    {
        using namespace std;
        vector<BoneType *> sel;
        vector<BoneType *> B = gather_bones(roots);
        typename vector<BoneType *>::iterator bi = B.begin() - 1;
        do
        {
            bi = find_if(bi + 1, B.end(), is_bone_selected<BoneType>);
            if (bi == B.end())
            {
                break;
            }
            sel.push_back(*bi);
        } while (true);
        return sel;
    }

    void delete_bone()
    {
        std::vector<BoneType *> selected = find_all_selected();
        for (BoneType *b : selected)
        {
            if (b)
            {
                delete b;
                b = nullptr;
            }
        }
    }
    void print()
    {
        using namespace std;
        std::vector<BoneType *> B = gather_bones(roots);
        // check bones
        std::cout << "=======================roots Size=" << roots.size() << std::endl;
        for (int i = 0; i < B.size(); i++)
        {
            BoneType *b = B[i];
            std::vector<BoneType *> children = b->get_children();
            cout << "Bone" << i << "[" << b << "]"
                 << ": is_root=" << b->is_root();
            if (!b->is_root())
            {
                cout << " parent_wi=" << b->get_parent()->get_wi();
            }
            cout << " wi=" << b->get_wi()
                 << " offset=" << b->offset.transpose()
                 << " children[size=" << children.size() << "]: ";

            for (int i = 0; i < children.size(); i++)
            {
                cout << children[i]->get_wi() << " ";
            }
            cout << endl;
        }
        std::cout << "============================" << std::endl;
    }
};