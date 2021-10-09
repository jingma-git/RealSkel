#pragma once
#include "Bone.h"
#include "Lerp.h"

inline bool copy_bone_roots_data(
    const std::vector<Bone *> &AR,
    const std::vector<Bone *> &BR)
{
    using namespace std;
    if (AR.size() != BR.size())
    {
        // different number of roots
        fprintf(
            stderr,
            "Error: copy_bone_roots_data.h: wrong number of roots %ld != %ld\n",
            AR.size(), BR.size());
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

    while (!AQ.empty() && !BQ.empty())
    {
        assert(AQ.size() == BQ.size());
        Bone *a = AQ.back();
        AQ.pop_back();
        Bone *b = BQ.back();
        BQ.pop_back();

        // copy a into b by lerping (a,b) to b
        lerp(a, b, 0, b);

        // get children
        std::vector<Bone *> b_children = b->get_children();
        std::vector<Bone *> a_children = a->get_children();
        // a and b should have same number of children
        if (a_children.size() != b_children.size())
        {
            fprintf(
                stderr,
                "Error: copy_bone_roots_data.h: wrong number of children %ld != %ld\n",
                a_children.size(), b_children.size());
            return false;
        }
        // Add children to queue
        AQ.insert(AQ.end(), a_children.begin(), a_children.end());
        BQ.insert(BQ.end(), b_children.begin(), b_children.end());
    }

    assert(AQ.size() == BQ.size());
    return true;
}