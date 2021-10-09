#pragma once

#include <list>
#include <unordered_map>

#include "straight_skeletor.h"
#include "Bone.h"

template <typename NodeT>
inline void sskel_to_bone(const SSkel<NodeT> &sskel, NodeT *root, Bone *boneRoot)
{
    typedef SSEdge<NodeT> EdgeT;
    // M_DEBUG << " sskel_to_bone sskel nodes=" << sskel.nodes.size() << " edge=" << sskel.edges.size() << std::endl;
    /*input*/
    const std::vector<NodeT *> &nodes = sskel.nodes;
    const std::vector<EdgeT *> &edges = sskel.edges;
    /*output*/
    Skeleton<Bone> *skel = boneRoot->skel;

    std::list<NodeT *> Q;
    std::list<Bone *> QS;
    std::unordered_map<NodeT *, bool> isVisit;
    Q.push_back(root);
    QS.push_back(boneRoot);
    int num_v = 0;
    int num_b = 0;
    while (!Q.empty())
    {
        NodeT *pNode = Q.front();
        Bone *pBone = QS.front();
        Q.pop_front();
        QS.pop_front();
        ++num_v;
        // M_DEBUG << " pNode" << pNode->idx << ": " << pNode->adj_edges.size() << std::endl;
        if (!isVisit[pNode])
        {
            isVisit[pNode] = true;
            for (int i = 0; i < pNode->adj_edges.size(); i++)
            {
                EdgeT *e = static_cast<EdgeT *>(pNode->adj_edges[i]);
                NodeT *cNode = e->node0 == pNode ? e->node1 : e->node0;
                if (!isVisit[cNode])
                {
                    // M_DEBUG << " visit cnode" << cNode->idx << std::endl;
                    Bone *cBone = new Bone(skel, pBone, cNode->pos - pNode->pos);
                    Q.push_back(cNode);
                    QS.push_back(cBone);
                    ++num_b;
                }
            }
        }
    }

    // M_DEBUG << " sskel_to_bone num_v=" << num_v << " num_b=" << num_b << std::endl;
}