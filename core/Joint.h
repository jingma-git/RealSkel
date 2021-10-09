#pragma once

#include "common.h"
#include <ostream>
struct Joint
{
    int idx = -1; // idx in XSubToy.m_joints, see XSubToy.h
    int group_idx;
    Pnt3 pos;

    // undirected graph
    std::vector<int> adj; // adj=parent+children
    // directed graph
    int parent;
    std::vector<int> children;

    Joint() {}
    Joint(const Pnt3 &pos_) : pos(pos_) {}
    Joint(int idx_, const Pnt3 &pos_) : idx(idx_), pos(pos_) {}

    void add_adj(int joint_idx)
    {
        if (std::find(adj.begin(), adj.end(), joint_idx) == adj.end())
        {
            // cout << __FILE__ << " " << __LINE__ << " joint " << idx << " add adjJoint " << joint_idx << endl;
            adj.push_back(joint_idx);
        }
    }
};

inline std::ostream &operator<<(std::ostream &out, const Joint &j)
{
    using namespace std;
    out << "joint" << j.idx << " " << j.pos.transpose() << endl;
    out << "adj: ";
    for (int i = 0; i < j.adj.size(); i++)
    {
        out << j.adj[i] << " ";
    }
    out << endl;
    return out;
}