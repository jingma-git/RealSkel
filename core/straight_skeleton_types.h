#pragma once
#include "common.h"

struct SSNode;

template <typename NodeT>
struct SSEdge;

enum SSNodeType
{
    NODE_JUNCTION,
    NODE_NORMAL,
    NODE_TERMINAL,
    SSNODE_INVALID
};

static std::string node_names[4] = {"NODE_JUNCTION", "NODE_NORMAL", "NODE_TERMINAL", "SSNODE_INVALID"};

struct SSNode
{
    int idx;
    Pnt3 pos;
    std::vector<void *> adj_edges;
    double radius = 0;

    SSNode() : idx(-1) {}
    SSNode(int idx_, const Pnt3 &pos_, double radius_) : idx(idx_), pos(pos_), radius(radius_) {}
    SSNode(int x_, int y_) : idx(-1), pos(x_, y_, 0) {}
    SSNode(const Pnt3 &pos_) : idx(-1), pos(pos_) {}

    void set_index(int idx_) { idx = idx_; }
    void set_pos(const Pnt3 &pos_) { pos = pos_; }
    void set_radius(double radius_) { radius = radius_; }

    void erase_adje(void *e)
    {
        adj_edges.erase(std::remove(adj_edges.begin(), adj_edges.end(), e));
    }

    SSNodeType type() const
    {
        if (adj_edges.size() >= 3)
        {
            return NODE_JUNCTION;
        }
        else if (adj_edges.size() == 2)
        {
            return NODE_NORMAL;
        }
        else if (adj_edges.size() == 1)
        {
            return NODE_TERMINAL;
        }
        else
        {
            return SSNODE_INVALID;
        }
    }

    friend std::ostream &operator<<(std::ostream &out, const SSNode &n)
    {
        out << "n=" << n.idx << ", pos=" << n.pos.transpose()
            << " radius=" << n.radius << ", adj_edges=" << n.adj_edges.size();
        return out;
    }

    friend std::istream &operator>>(std::istream &in, SSNode &node)
    {
        return in;
    }
};

template <class NodeT>
struct SSEdge
{
    int idx;
    NodeT *node0;
    NodeT *node1;
    SSEdge() : idx(-1), node0(nullptr), node1(nullptr) {}
    SSEdge(NodeT *node0_, NodeT *node1_) : idx(-1), node0(node0_), node1(node1_) {}
    SSEdge(int idx_, NodeT *node0_, NodeT *node1_) : idx(idx_), node0(node0_), node1(node1_) {}

    void set_index(int idx_) { idx = idx_; }

    double len() const
    {
        return (node0->pos - node1->pos).norm();
    }

    friend std::ostream &operator<<(std::ostream &out, const SSEdge &e)
    {
        out << "e" << e.idx << "[n" << e.node0->idx << "-n" << e.node1->idx << "] len=" << e.len();
        return out;
    }
};
