#pragma once

#include "straight_skeletor.h"
#include "ssnode_with_subtoy_info.h"

inline void writeSSkel(const std::string fname, const SSkel<SSNodeSub> &sskel)
{
    typedef SSNodeSub NodeT;
    typedef SSEdge<NodeT> EdgeT;

    int num_nodes = static_cast<int>(sskel.nodes.size());
    int num_edges = static_cast<int>(sskel.edges.size());

    FILE *fptr = fopen(fname.c_str(), "w");
    if (fptr == NULL)
    {
        M_DEBUG << "Cannot open " << fname << "!" << endl;
        return;
    }

    fprintf(fptr, "%d %d\n", num_nodes, num_edges);
    // idx pos radius
    for (int i = 0; i < num_nodes; ++i)
    {
        NodeT *node = sskel.nodes[i];
        const Pnt3 &pos = node->pos;
        fprintf(fptr, "%d %lg %lg %lg %lg %d\n", node->idx, pos[0], pos[1], pos[2], node->radius, node->subToy->GetID());
    }
    for (int i = 0; i < num_edges; ++i)
    {
        EdgeT *edge = sskel.edges[i];
        fprintf(fptr, "%d %d\n", edge->node0->idx, edge->node1->idx);
    }

    fclose(fptr);
}

inline void readSSkel(const std::string fname, SSkel<SSNodeSub> &sskel)
{
    typedef SSNodeSub NodeT;
    typedef SSEdge<NodeT> EdgeT;

    FILE *fptr = fopen(fname.c_str(), "r");
    if (fptr == NULL)
    {
        M_DEBUG << "Cannot open " << fname << "!" << endl;
        return;
    }

    const int LINE_LEN = 500;
    char line[LINE_LEN];

    sskel.clear();
    int num_nodes = 0;
    int num_edges = 0;
    fgets(line, LINE_LEN, fptr);
    sscanf(line, "%d %d", &num_nodes, &num_edges);
    // idx pos radius
    for (int i = 0; i < num_nodes; ++i)
    {
        fgets(line, LINE_LEN, fptr);
        int idx;
        Pnt3 pos;
        double radius;
        int subToyID;
        sscanf(line, "%d %lg %lg %lg %lg %d\n", &idx, &pos[0], &pos[1], &pos[2], &radius, &subToyID);
        NodeT *node = sskel.add_node(pos);
        node->set_radius(radius);
        node->subToyID = subToyID;
    }

    for (int i = 0; i < num_edges; ++i)
    {
        fgets(line, LINE_LEN, fptr);
        int i0, i1;
        sscanf(line, "%d %d", &i0, &i1);
        NodeT *node0 = sskel.nodes[i0];
        NodeT *node1 = sskel.nodes[i1];
        sskel.add_edge(node0, node1);
    }

    fclose(fptr);
}
