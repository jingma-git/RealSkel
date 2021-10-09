#pragma once

class XSubToy;

struct SSNodeWithSubToyInfo : public SSNode
{
    XSubToy *subToy;
    int subToyID = -1;
    SSNodeWithSubToyInfo(int x, int y) : SSNode(x, y) {}
    SSNodeWithSubToyInfo(const Pnt3 &pos_) : SSNode(pos_) {}
    void set_subtoy(XSubToy *subToy_)
    {
        subToy = subToy_;
        subToyID = subToy->GetID();
    }

    friend std::ostream &operator<<(std::ostream &out, const SSNodeWithSubToyInfo &n)
    {
        out << "n=" << n.idx << ", pos=" << n.pos.transpose()
            << " radius=" << n.radius << ", adj_edges=" << n.adj_edges.size()
            << " subToy=" << n.subToyID;
        return out;
    }
};

typedef SSNodeWithSubToyInfo SSNodeSub;
