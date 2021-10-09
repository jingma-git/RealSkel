#pragma once
#include "straight_skeleton_types.h"
#include "XFunctions.h"
#include "MemoryPool.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/create_straight_skeleton_2.h>
#include <unordered_map>
#include <unordered_set>
#include <memory>

#ifndef SSKEL_DEBUG
//#define SSKEL_DEBUG
#endif

enum BranchType
{
    JJ_BRANCH,
    JT_BRANCH,
    TT_BRANCH
};

template <typename NodeT>
struct SSBranch
{
    int id;
    NodeT *st;
    NodeT *end;
    std::vector<Pnt3> pnts;
    std::vector<double> radius;
    std::vector<Pnt3> simp_pnts; // DP simplification on pnts
    std::vector<double> simp_radius;
    double dp_thresh = 5;

    SSBranch() {}
    SSBranch(NodeT *st_, NodeT *end_) : st(st_), end(end_) {}

    void translate(const Vec3 &trans_update)
    {
        TranslateLine(pnts, trans_update);
        TranslateLine(simp_pnts, trans_update);
    }

    void rotate(const Eigen::Matrix3d &R)
    {
        RotateLine(pnts, R);
        RotateLine(simp_pnts, R);
    }

    void scale(double scl)
    {
        ScaleLine(pnts, scl);
        ScaleLine(simp_pnts, scl);
    }

    void set_id(int id_) { id = id_; }
    std::string id_str() const { return std::to_string(id); }
    double len() const
    {
        double sum = 0;
        for (int i = 0; i < pnts.size() - 1; i++)
        {
            sum += (pnts[i + 1] - pnts[i]).norm();
        }
        return sum;
    }

    size_t psize() const { return pnts.size(); }
    Pnt3 &p(size_t i) { return pnts[i]; }
    const Pnt3 &p(size_t i) const { return pnts[i]; }
    Pnt3 &lastp() { return pnts.back(); }
    const Pnt3 &lastp() const { return pnts.back(); }
    Pnt3 &lastp2nd() { return pnts[pnts.size() - 2]; }
    const Pnt3 &lastp2nd() const { return pnts[pnts.size() - 2]; }
    void copy_pnts_to_simp_pnts()
    {
        simp_pnts.reserve(pnts.size());
        for (const Pnt3 &p : pnts)
        {
            simp_pnts.push_back(p);
        }
    }

    void copy_radius_to_simp_radius()
    {
        simp_radius.reserve(radius.size());
        for (double r : radius)
        {
            simp_radius.push_back(r);
        }
    }

    double max_radius()
    {
        double max_r = std::numeric_limits<double>::min();
        for (double r : radius)
        {
            if (r > max_r)
                max_r = r;
        }
        return max_r;
    }

    BranchType type() const
    {
        if (st->adj_edges.size() >= 3 && end->adj_edges.size() >= 3)
        {
            return JJ_BRANCH;
        }
        else if (st->adj_edges.size() == 1 && end->adj_edges.size() == 1)
        {
            return TT_BRANCH;
        }
        else
        {
            return JT_BRANCH;
        }
    }

    bool operator<(const SSBranch &other) const
    {
        return this->len() > other.len();
    }

    std::string type_name() const
    {
        if (st->adj_edges.size() >= 3 && end->adj_edges.size() >= 3)
        {
            return "JJ_BRANCH";
        }
        else if (st->adj_edges.size() == 1 && end->adj_edges.size() == 1)
        {
            return "TT_BRANCH";
        }
        else
        {
            return "JT_BRANCH";
        }
    }

    friend std::ostream &operator<<(std::ostream &out, const SSBranch &b)
    {
        out << b.type_name() << b.id << " "
            << b.st->idx << "-" << b.end->idx << " "
            << "pnts=" << b.pnts.size() << " "
            << "simp_pnts=" << b.simp_pnts.size() << " "
            << "len=" << b.len() << " "
            << "dp_thresh=" << b.dp_thresh << endl;
        return out;
    }
};

template <typename NodeT>
class SSkel
{
public:
    typedef SSEdge<NodeT> EdgeT; // Edge Type
    std::vector<NodeT *> nodes;
    std::vector<EdgeT *> edges;
    NodeT *root;

private:
    MemoryPool<NodeT> nodePool;
    MemoryPool<EdgeT> edgePool;

public:
    SSkel()
    {
        reserve_memory(128); // 128 for large skel, 32 for small skel
    }

    SSkel(int nodeCapacity)
    {
        reserve_memory(nodeCapacity);
    }

    ~SSkel()
    {
        // return element's space to pool
        clear();
        // release pool's memory
        nodePool.Clear();
        edgePool.Clear();
    }

    void reserve_memory(int nodeCapacity)
    {
        nodePool.SetBlockSize(nodeCapacity);
        edgePool.SetBlockSize(nodeCapacity);
        nodes.reserve(nodeCapacity);
        edges.reserve(nodeCapacity);
    }

    void clear()
    {
        for (NodeT *n : nodes)
            nodePool.Recycle(n);
        nodes.clear();

        for (EdgeT *e : edges)
            edgePool.Recycle(e);
        edges.clear();
    }

    NodeT *add_node(int x, int y)
    {
        NodeT *ptr = nodePool.Request();
        new (ptr) NodeT(x, y);
        ptr->set_index(nodes.size());
        nodes.push_back(ptr);
        return ptr;
    }

    NodeT *add_node(const Pnt3 &pos)
    {
        NodeT *ptr = nodePool.Request();
        new (ptr) NodeT(pos);
        ptr->set_index(nodes.size());
        nodes.push_back(ptr);
        return ptr;
    }

    void delete_node_fromSkel(NodeT *node)
    {
        int nidx = node->idx;
        nodePool.Recycle(node);
        if (nidx != nodes.size() - 1)
        {
            NodeT *nback = nodes.back();
            nback->set_index(nidx);
            nodes[nidx] = nback;
        }
        nodes.pop_back();
    }

    EdgeT *add_edge(NodeT *node0, NodeT *node1)
    {
        EdgeT *ptr = edgePool.Request();
        new (ptr) EdgeT(node0, node1);
        ptr->set_index(edges.size());
        edges.push_back(ptr);
        node0->adj_edges.push_back(ptr);
        node1->adj_edges.push_back(ptr);
        return ptr;
    }

    EdgeT *get_edge(int i) const
    {
        return edges[i];
    }

    void delete_edge_fromSkel(EdgeT *edge)
    {
        int eidx = edge->idx;
        edgePool.Recycle(edge);
        if (eidx != edges.size() - 1)
        {
            EdgeT *eback = edges.back();
            eback->set_index(eidx);
            edges[eidx] = eback;
        }
        edges.pop_back();
    }

    NodeT *find_node_with_max_branches() const
    {
        int max_b = -1;
        NodeT *max_n = nullptr;
        for (NodeT *node : nodes)
        {
            if (static_cast<int>(node->adj_edges.size()) > max_b)
            {
                max_b = node->adj_edges.size();
                max_n = node;
            }
        }
        return max_n;
    }

    NodeT *find_closest(const Pnt3 &p) const
    {
        double min_dis = INT_MAX;
        NodeT *clo;
        for (NodeT *node : nodes)
        {
            double dis = (node->pos - p).norm();
            if (dis < min_dis)
            {
                min_dis = dis;
                clo = node;
            }
        }
        return clo;
    }

    NodeT *find_closest_end_node(const Pnt3 &p) const
    {
        double min_dis = INT_MAX;
        NodeT *clo;
        for (NodeT *node : nodes)
        {
            if (node->type() == NODE_TERMINAL)
            {
                double dis = (node->pos - p).norm();
                if (dis < min_dis)
                {
                    min_dis = dis;
                    clo = node;
                }
            }
        }
        return clo;
    }

    void collect_nodes(std::vector<NodeT *> &junc_nodes, std::vector<NodeT *> &term_nodes) const
    {
        for (NodeT *node : nodes)
        {
            if (node->type() == NODE_JUNCTION)
            {
                junc_nodes.push_back(node);
            }
            else if (node->type() == NODE_TERMINAL)
            {
                term_nodes.push_back(node);
            }
        }
    }

    void set_root(NodeT *root_)
    {
        root = root_;
    }

    EdgeT *find_shortest_edge()
    {
        double min_len = std::numeric_limits<double>::max();
        EdgeT *min_e;
        for (EdgeT *e : edges)
        {
            double len = e->len();
            if (len < min_len)
            {
                min_len = len;
                min_e = e;
            }
        }
        return min_e;
    }

    NodeT *collapse_edge(EdgeT *edge)
    {
        // std::cout << "collapse " << *edge << " len=" << edge->len() << std::endl;
        NodeT *n0 = edge->node0;
        NodeT *n1 = edge->node1;

        // find position for new node
        Pnt3 pos;
        double radius;
        if (n0->adj_edges.size() == 2 && n1->adj_edges.size() == 1) // use terminal node's position
        {
            pos = n1->pos;
            radius = n1->radius;
        }
        else if (n0->adj_edges.size() == 1 && n1->adj_edges.size() == 2) // use terminal node's position
        {
            pos = n0->pos;
            radius = n0->radius;
        }
        else if (n0->adj_edges.size() == 2 && n1->adj_edges.size() == 2) // use both normal_nodes' avg
        {
            pos = (n0->pos + n1->pos) * 0.5;
            radius = (n0->radius + n1->radius) * 0.5;
        }
        else if (n0->adj_edges.size() >= 3 && n1->adj_edges.size() <= 2) // use junction node's position
        {
            pos = n0->pos;
            radius = n0->radius;
        }
        else if (n0->adj_edges.size() <= 2 && n1->adj_edges.size() >= 3) // use junction node's position
        {
            pos = n1->pos;
            radius = n1->radius;
        }
        else if (n0->adj_edges.size() >= 3 && n1->adj_edges.size() >= 3) // use both junction_nodes' avg
        {
            pos = (n0->pos + n1->pos) * 0.5;
            radius = (n0->radius + n1->radius) * 0.5;
        }
        else
        {
            assert(false && "edge's node must have adjacent edges!");
        }

        NodeT *newnode = static_cast<NodeT *>(add_node(pos));
        newnode->set_radius(radius);
        // M_DEBUG << "Collapse Edge " << *edge << endl;
        // // change adj_edges's node to newnode
        // M_DEBUG << "first: " << *n0 << endl;
        for (int i = 0; i < n0->adj_edges.size(); ++i)
        {
            EdgeT *adje = static_cast<EdgeT *>(n0->adj_edges[i]);
            if (adje != edge)
            {
                // M_DEBUG << "change adje " << *adje << endl;
                if (adje->node0 == n0)
                {
                    adje->node0 = newnode;
                    newnode->adj_edges.push_back(adje);
                }
                else if (adje->node1 == n0)
                {
                    // M_DEBUG << "adje->node1=" << *adje->node1 << endl;
                    // assert(adje->node1 == n0 && "the other node must equal to n0!");
                    adje->node1 = newnode;
                    newnode->adj_edges.push_back(adje);
                }
                // std::cout << " to " << *adje << std::endl;
            }
        }

        // M_DEBUG << "second: " << *n1 << endl;
        for (int i = 0; i < n1->adj_edges.size(); ++i)
        {
            EdgeT *adje = static_cast<EdgeT *>(n1->adj_edges[i]);
            if (adje != edge)
            {
                // M_DEBUG << "change adje " << *adje << endl;
                if (adje->node1 == n1)
                {
                    adje->node1 = newnode;
                    newnode->adj_edges.push_back(adje);
                }
                else if (adje->node0 == n1)
                {
                    // M_DEBUG << "adje->node0=" << *adje->node0 << endl;
                    // assert(adje->node0 == n1 && "the other node must equal to n1!");
                    adje->node0 = newnode;
                    newnode->adj_edges.push_back(adje);
                }
                // std::cout << " to " << *adje << std::endl;
            }
        }
        // M_DEBUG << " newnode: " << *newnode << std::endl;

        // release resources for old deleted nodes
        delete_node_fromSkel(n0);
        delete_node_fromSkel(n1);
        delete_edge_fromSkel(edge);
        return newnode;
    }

    // collapse edge by attatch from's edges to to's edges
    void collapse_edge_from_to(EdgeT *edge, NodeT *from, NodeT *to)
    {
        NodeT *n0 = edge->node0;
        NodeT *n1 = edge->node1;
        assert((n0 == from && n1 == to) || (n0 == to && n1 == from));

        to->adj_edges.erase(std::remove(to->adj_edges.begin(), to->adj_edges.end(), edge), to->adj_edges.end());
        for (size_t i = 0; i < from->adj_edges.size(); ++i)
        {
            EdgeT *adje = static_cast<EdgeT *>(from->adj_edges[i]);
            if (adje != edge)
            {
                if (adje->node0 == from)
                {
                    adje->node0 = to;
                }
                else if (adje->node1 == from)
                {
                    // assert(adje->node1 == from);
                    adje->node1 = to;
                }
                to->adj_edges.push_back(adje);
            }
        }

        delete_node_fromSkel(from);
        delete_edge_fromSkel(edge);
    }

    void merge_junction_nodes(double merge_thresh)
    {
        std::vector<NodeT *> junc_nodes, term_nodes;
        collect_nodes(junc_nodes, term_nodes);
        M_DEBUG << "merge_junction_nodes " << junc_nodes.size() << endl;

        std::unordered_map<NodeT *, int> cluster_id;
        int num_cluster = 0;
        for (NodeT *junc_node : junc_nodes)
        {
            cluster_id[junc_node] = -1;
        }

        // 1.a build junction nodes cluster
        for (NodeT *junc_node : junc_nodes)
        {
            if (cluster_id[junc_node] == -1)
            {
                std::list<NodeT *> Q;
                Q.push_back(junc_node);

                while (!Q.empty())
                {
                    NodeT *cur_node = Q.front();
                    Q.pop_front();
                    cluster_id[cur_node] = num_cluster;
                    int n = static_cast<int>(cur_node->adj_edges.size());
                    for (int i = 0; i < n; ++i)
                    {
                        EdgeT *e = static_cast<EdgeT *>(cur_node->adj_edges[i]);
                        if (e->len() < merge_thresh)
                        {
                            NodeT *adj_node = (e->node0 == cur_node) ? e->node1 : e->node0;
                            if (adj_node->type() == NODE_JUNCTION && cluster_id[adj_node] == -1)
                            {
                                Q.push_back(adj_node);
                            }
                        }
                    }
                }

                ++num_cluster;
            }
        }

        M_DEBUG << "num_cluster= " << num_cluster << endl;
        std::vector<std::vector<NodeT *>> clusters;
        clusters.resize(num_cluster);
        for (auto it = cluster_id.begin(); it != cluster_id.end(); ++it)
        {
            NodeT *node = it->first;
            int id = it->second;
            clusters[id].push_back(node);
        }

        // 1.b merge junction nodes in one cluster
        int c_id = 0;
        for (std::vector<NodeT *> &cluster : clusters)
        {
            if (cluster.size() > 1)
            {
                // find cluster center, cluster internal edges, cluster peripheral edges
                Pnt3 cent(0, 0, 0);
                double radius = 0;
                for (NodeT *node : cluster)
                {
                    cent += node->pos; // center
                    radius += node->radius;
                }
                cent /= (double)cluster.size();
                radius /= (double)cluster.size();
                M_DEBUG << "cluster" << c_id << ", cent=" << cent.transpose() << " size=" << cluster.size() << endl;
                // add new node to sskel
                NodeT *newNode = add_node(cent);
                newNode->setRadius(radius);
                // replace out_es' in_cluster_node as center(newNode)
                std::set<EdgeT *> in_es;
                for (NodeT *node : cluster)
                {
                    int n = static_cast<int>(node->adj_edges.size());
                    for (int i = 0; i < n; ++i)
                    {
                        EdgeT *e = static_cast<EdgeT *>(node->adj_edges[i]);
                        NodeT *adj_node = (e->node0 == node) ? e->node1 : e->node0;
                        if (std::find(cluster.begin(), cluster.end(), adj_node) != cluster.end())
                        {
                            in_es.insert(e);
                        }
                        else
                        {
                            if (e->node0 == node)
                            {
                                e->node0 = newNode;
                            }
                            else if (e->node1 == node)
                            {
                                e->node1 = newNode;
                            }
                            newNode->adj_edges.push_back(e); // peripheral edges
                        }
                    }
                }
                // destroy in_cluster_edges
                for (EdgeT *edge : in_es)
                {
                    delete_edge_fromSkel(edge);
                }
                // destroy in_cluster_nodes
                for (NodeT *node : cluster)
                {
                    delete_node_fromSkel(node);
                }
            }

            ++c_id;
        }

        M_DEBUG << "merge_junction_nodes successfully!" << endl;
    }

    double max_jt_edge_len()
    {
        double max_len = -1.0;
        // std::vector<NodeT *> junc_nodes, term_nodes;
        // collect_nodes(junc_nodes, term_nodes);

        // for (NodeT *junc_node : junc_nodes)
        // {
        //     int n = static_cast<int>(junc_node->adj_edges.size());
        //     for (int i = 0; i < n; ++i)
        //     {
        //         EdgeT *e = static_cast<EdgeT *>(junc_node->adj_edges[i]);
        //         NodeT *adj_node = (e->node0 == junc_node) ? e->node1 : e->node0;
        //         if (adj_node->type() == NODE_TERMINAL)
        //         {
        //             double len = e->len();
        //             if (len > max_len)
        //             {
        //                 max_len = len;
        //             }
        //         }
        //     }

        // }
        for (EdgeT *e : edges)
        {
            if ((e->node0->type() == NODE_JUNCTION && e->node1->type() == NODE_TERMINAL) ||
                (e->node1->type() == NODE_TERMINAL && e->node0->type() == NODE_JUNCTION))
            {
                double len = e->len();
                if (len > max_len)
                {
                    max_len = len;
                }
            }
        }
        return max_len;
    }

    double max_edge_len()
    {
        double max_len = -1.0;
        for (EdgeT *e : edges)
        {
            double len = e->len();
            if (len > max_len)
                max_len = len;
        }
        return max_len;
    }

    double avg_edge_len()
    {
        if (edges.size() == 0)
            return 0;

        double sum = 0;
        for (EdgeT *e : edges)
        {
            sum += e->len();
        }

        return sum /= edges.size();
    }

    void find_jt_edges(std::vector<EdgeT *> &jt_edges)
    {
        for (EdgeT *e : edges)
        {
            if (e->node0->type() == NODE_JUNCTION && e->node1->type() == NODE_TERMINAL)
            {
                jt_edges.push_back(e);
            }
            else if (e->node1->type() == NODE_JUNCTION && e->node0->type() == NODE_TERMINAL)
            {
                jt_edges.push_back(e);
            }
        }
    }

    void prune_short_jt_edges(double prune_thresh)
    {
        M_DEBUG << " prune_short_jt_edges" << endl;

        while (true)
        {

            std::vector<EdgeT *> jt_es;
            find_jt_edges(jt_es);
            M_DEBUG << "jt_es=" << jt_es.size() << endl;
            if (jt_es.size() == 0)
                break;

            std::sort(jt_es.begin(), jt_es.end(), [](EdgeT *a, EdgeT *b)
                      { return a->len() < b->len(); });

            EdgeT *e = jt_es[0];
            M_DEBUG << "jt_es[0]=" << jt_es.front()->len() << " jt_es.back()=" << jt_es.back()->len() << endl;
            if (e->len() > prune_thresh)
                break;

            NodeT *junc_node, *adj_node;
            if (e->node0->type() == NODE_JUNCTION)
            {
                junc_node = e->node0;
                adj_node = e->node1;
            }
            else
            {
                junc_node = e->node1;
                adj_node = e->node0;
            }

            junc_node->adj_edges.erase(std::remove(junc_node->adj_edges.begin(), junc_node->adj_edges.end(), e),
                                       junc_node->adj_edges.end());
            delete_edge_fromSkel(e);
            delete_node_fromSkel(adj_node);
        }

        M_DEBUG << " prune_short_jt_edges successfully" << endl;
    }

    void collapse_short_nn_edges(double collapse_thresh)
    {
        while (true)
        {
            if (edges.size() == 1) // only has one bone left
                break;

            std::vector<EdgeT *> nn_edges;
            int n = static_cast<int>(edges.size());
            for (int i = 0; i < n; ++i)
            {
                EdgeT *e = static_cast<EdgeT *>(edges[i]);
                if (e->node0->type() == NODE_NORMAL && e->node1->type() == NODE_NORMAL)
                    nn_edges.push_back(e);
            }

            if (nn_edges.size() < 1)
                break;

            std::sort(nn_edges.begin(), nn_edges.end(), [&](EdgeT *a, EdgeT *b)
                      { return a->len() < b->len(); }); // This is important to prevent duplicate edges or chaos edge order in orignal sskel

            EdgeT *e = nn_edges[0];
            if (e->len() >= collapse_thresh) // the minimum edge's length is greater than thresh
                break;
            collapse_edge(e);
        }
    }

    void split_edge(EdgeT *edge, NodeT *node)
    {
        EdgeT *old_e = edge;
        NodeT *old_n0 = edge->node0;
        NodeT *old_n1 = edge->node1;
        EdgeT *new_e = add_edge(old_n0, node); // (old_n0/node)->adj_edges.push_back(new_e)
        old_n0->erase_adje(old_e);
        old_e->node0 = node;
        node->adj_edges.push_back(old_e);
        node->set_radius((old_n0->radius + old_n1->radius) * 0.5);
    }

    void rotate(const Eigen::Matrix3d &rot)
    {
        for (auto it = nodes.begin(); it != nodes.end(); ++it)
        {
            (*it)->pos = rot * (*it)->pos;
        }
    }

    void scale(double scl)
    {
        for (auto it = nodes.begin(); it != nodes.end(); ++it)
        {
            (*it)->pos = scl * (*it)->pos;
            (*it)->radius = scl * (*it)->radius;
        }
    }

    void translate(const Vec3 &trans_update)
    {
        for (NodeT *node : nodes)
        {
            node->pos += trans_update;
        }
    }

    void simplify_by_collapse_short_edges(int max_iter = 3)
    {
        int iter = 0;
        M_DEBUG << "simplify_by_collapse_short_edges" << std::endl;
        while (iter < max_iter)
        {
            M_DEBUG << "...iter" << iter << " edges=" << edges.size() << std::endl;
            double thresh = 0.5 * avg_edge_len();
            //M_DEBUG << " avg_len=" << avg_len() << std::endl;
            bool need_collapse = false;
            for (int i = 0; i < edges.size(); ++i)
            {
                EdgeT *e = edges[i];
                if (e->len() < thresh)
                {
                    collapse_edge(e);
                    need_collapse = true;
                }
            }
            if (!need_collapse) // all edges' length is greater than thresh
            {
                break;
            }
            ++iter;
        }
    }

    void print()
    {
        std::cout << "-----------------------------------------" << std::endl;
        std::cout << "nodes=" << nodes.size() << std::endl;
        for (int i = 0; i < nodes.size(); ++i)
        {
            NodeT *n = nodes[i];
            std::cout << *n << std::endl;
        }
        std::cout << "edges=" << edges.size() << std::endl;
        for (int i = 0; i < edges.size(); ++i)
        {
            EdgeT *e = edges[i];
            std::cout << *e << std::endl;
        }
        std::cout << "-----------------------------------------" << std::endl;
    }
};

template <typename NodeT>
void GenSymSSkel(const SymetryPlane sym_plane, const double plane_pos, const SSkel<NodeT> &other, SSkel<NodeT> &sskel)
{
    typedef SSEdge<NodeT> EdgeT;
    std::unordered_map<NodeT *, NodeT *> node_map;
    // nodes
    for (NodeT *node : other.nodes)
    {
        const Pnt3 &p = node->pos;
        Pnt3 newP;
        if (sym_plane == SYM_XY_PLANE)
        {
            newP = Pnt3(p.x(), p.y(), 2 * plane_pos - p.z());
        }
        else if (sym_plane == SYM_YZ_PLANE)
        {
            newP = Pnt3(2 * plane_pos - p.x(), p.y(), p.z());
        }
        NodeT *newNode = sskel.add_node(newP);
        newNode->set_radius(node->radius);
        node_map[node] = newNode;
    }
    //edges
    for (EdgeT *edge : other.edges)
    {
        NodeT *n0 = node_map[edge->node0];
        NodeT *n1 = node_map[edge->node1];
        sskel.add_edge(n0, n1);
    }
    sskel.set_root(node_map[other.root]);
}

template <typename NodeT>
void find_branches_jnode(NodeT *node,
                         std::list<SSBranch<NodeT>> &branches,
                         std::unordered_map<NodeT *, std::vector<SSBranch<NodeT> *>> &node_branches)
{
    typedef SSEdge<NodeT> EdgeT;
    typedef SSBranch<NodeT> SSBranchT;
    //  std::cout << "search junction node" << node->idx << std::endl;
    for (int i = 0; i < node->adj_edges.size(); ++i)
    {
        EdgeT *e = static_cast<EdgeT *>(node->adj_edges[i]);
        // std::cout << "............start search edge" << e->node0->idx << "-" << e->node1->idx << std::endl;
        SSBranchT b;
        b.st = node;
        b.pnts.push_back(node->pos);
        b.radius.push_back(node->radius);

        NodeT *cur = e->node0 == node ? e->node1 : e->node0;
        std::unordered_map<NodeT *, bool> isVisit;
        isVisit[node] = true;
        while (1)
        {
            // M_DEBUG << " node" << cur->idx << ": " << node_names[cur->type()] << " adj_edges=" << cur->adj_edges.size() << std::endl;
            b.pnts.push_back(cur->pos);
            b.radius.push_back(cur->radius);

            isVisit[cur] = true;
            if (cur->type() != NODE_NORMAL) // could be junction or terminal
                break;
            for (int i = 0; i < cur->adj_edges.size(); ++i)
            {
                EdgeT *edge = static_cast<EdgeT *>(cur->adj_edges[i]);
                NodeT *adj_n = edge->node0 == cur ? edge->node1 : edge->node0;
                if (!isVisit[adj_n])
                {
                    cur = adj_n;
                    break;
                }
            }
        }
        b.end = cur;
        b.copy_pnts_to_simp_pnts();
        b.copy_radius_to_simp_radius();
        // In case for JJBranch, b will get searched twice
        typename std::list<SSBranchT>::iterator bit = branches.begin();
        for (; bit != branches.end(); ++bit)
        {
            if ((bit->st == b.st && bit->end == b.end) ||
                (bit->st == b.end && bit->end == b.st))
            {
                break;
            }
        }

        if (bit == branches.end()) // not found
        {
            b.set_id(static_cast<int>(branches.size()));
            branches.push_back(b);
            node_branches[node].push_back(&branches.back());
            // M_DEBUG << "add branch " << *node_branches[node].back() << std::endl;
        }
        else
        {
            node_branches[node].push_back(&(*bit));
            // M_DEBUG << "found branch " << *node_branches[node].back() << std::endl;
        }
    }
}

template <typename NodeT>
void collect_node_branches(std::list<SSBranch<NodeT>> &branches,
                           std::unordered_map<NodeT *, std::vector<SSBranch<NodeT> *>> &node_branches)
{
    // M_DEBUG << "collect_node_branches=" << branches.size() << endl;
    typedef SSBranch<NodeT> SSBranchT;
    for (auto bit = branches.begin(); bit != branches.end(); ++bit)
    {
        SSBranchT *b = &(*bit);
        // M_DEBUG << *b << endl;
        if (b->type() == JT_BRANCH)
        {
            // M_DEBUG << "JT_BRANCH" << endl;
            node_branches[b->st].push_back(b);
        }
        else if (b->type() == JJ_BRANCH)
        {
            // M_DEBUG << "JJ_BRANCH" << endl;
            node_branches[b->st].push_back(b);
            node_branches[b->end].push_back(b);
        }
    }
}

template <typename NodeT>
void find_ttbranch(const std::vector<NodeT *> &term_nodes, std::vector<Pnt3> &b) // terminal-terminal
{
    typedef SSEdge<NodeT> EdgeT;
    assert(term_nodes.size() == 2 && "Should only have two terminal nodes for a single branch!");

    NodeT *st = term_nodes.front();
    NodeT *end = term_nodes.back();
    NodeT *cur = st;
    std::unordered_map<NodeT *, bool> isVisit;
    while (true)
    {
        isVisit[cur] = true;
        b.push_back(cur->pos);

        if (cur == end)
            break;

        // find next adjacent node
        NodeT *adj_n = nullptr;
        for (int i = 0; i < cur->adj_edges.size(); ++i)
        {
            EdgeT *e = static_cast<EdgeT *>(cur->adj_edges[i]);
            adj_n = e->node0 == cur ? e->node1 : e->node0;
            if (!isVisit[adj_n])
            {
                cur = adj_n;
                break;
            }
        }
    }
}

struct SSkeletor
{
    typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
    typedef K::Point_2 Point_2;
    typedef K::Vector_2 Vector_2;
    typedef CGAL::Straight_skeleton_2<K> Ss;
    typedef boost::shared_ptr<Ss> SsPtr;
    typedef typename Ss::Vertex_const_handle Vertex_const_handle;
    typedef typename Ss::Halfedge_const_handle Halfedge_const_handle;
    typedef typename Ss::Halfedge_const_iterator Halfedge_const_iterator;

    static double min_dis_to_contour(const Vertex_const_handle &v0)
    {
        const Point_2 &p0 = v0->point();

        // defining contours
        auto hit = v0->defining_contour_halfedges_begin();
        auto be_it = v0->defining_contour_halfedges_begin();
        double min_dis = std::numeric_limits<double>::max();
        do
        {

            Vertex_const_handle def_v0 = (*hit)->vertex();
            Vertex_const_handle def_v1 = (*hit)->opposite()->vertex();
            const Point_2 &def_p0 = def_v0->point();
            const Point_2 &def_p1 = def_v1->point();
            Vector_2 vec0 = def_p0 - p0;
            Vector_2 vec1 = def_p1 - p0;
            double dis0 = std::sqrt(vec0.squared_length());
            double dis1 = std::sqrt(vec1.squared_length());
            if (dis0 < min_dis)
            {
                min_dis = dis0;
            }
            if (dis1 < min_dis)
            {
                min_dis = dis1;
            }
            ++hit;
        } while (hit != be_it);
        return min_dis;
    }

    template <typename NodeT>
    static void cvtToSSkel(const SsPtr &ss, SSkel<NodeT> &sskel)
    {
        std::unordered_map<Vertex_const_handle, NodeT *> node_map;
        for (Halfedge_const_iterator h = ss->halfedges_begin(); h != ss->halfedges_end(); ++h)
        {
            Vertex_const_handle v0 = h->vertex();
            Vertex_const_handle v1 = h->opposite()->vertex();
            if (h->is_bisector() && h->id() % 2 == 0 && !h->has_infinite_time() && !h->opposite()->has_infinite_time())
            {
                if (v0->is_skeleton() && v1->is_skeleton())
                {
                    const Point_2 &p0 = v0->point();
                    const Point_2 &p1 = v1->point();
                    NodeT *node0 = node_map[v0];
                    NodeT *node1 = node_map[v1];
                    if (node0 == nullptr)
                    {
                        node0 = sskel.add_node(p0.x(), p0.y());
                        node_map[v0] = node0;
                        node0->set_radius(min_dis_to_contour(v0));
                    }

                    if (node1 == nullptr)
                    {
                        node1 = sskel.add_node(p1.x(), p1.y());
                        node_map[v1] = node1;
                        node1->set_radius(min_dis_to_contour(v1));
                    }
                    sskel.add_edge(node0, node1);
                }
            }
        }
    }

    template <typename NodeT>
    static SsPtr skeletonize(const std::vector<Pnt3> &pts_poly,
                             SSkel<NodeT> &sskel)
    {
        std::vector<Point_2> poly;
        poly.reserve(pts_poly.size());
        for (const Pnt3 &p : pts_poly)
        {
            poly.emplace_back(p.x(), p.y());
        }
        // SsPtr ss = CGAL::create_interior_straight_skeleton_2(poly.begin(), poly.end());
        SsPtr ss = CGAL::create_interior_straight_skeleton_2(poly.begin(), poly.end());

#ifdef SSKEL_DEBUG
        std::cout << "Straight skeleton with " << ss->size_of_vertices()
                  << " vertices, " << ss->size_of_halfedges()
                  << " halfedges and " << ss->size_of_faces()
                  << " faces" << std::endl;
#endif

        M_DEBUG << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~start create raw skeleton" << endl;
        SSkel<NodeT> rawSSkel;
        cvtToSSkel(ss, rawSSkel);
        // rawSSkel.print();
        // rawSSkel.draw("output/raw.jpg");

        M_DEBUG << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~start collapsing short edges..." << endl;
        rawSSkel.simplify_by_collapse_short_edges(1);
        // rawSSkel.draw("output/collapse.jpg");
#ifdef SSKEL_DEBUG
        cout << "after pruning" << endl;
        rawSSkel.print();
#endif
        M_DEBUG << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~start simplify skeleton branch by branch" << endl;
        simplify_sskel(rawSSkel, sskel);
        sskel.print();
        // sskel.draw("output/simp.jpg");
        return ss;
    }

    // ToDO: remove this
    template <typename NodeT>
    static SsPtr skeletonize(const std::vector<Pnt3> &pts_poly,
                             SSkel<NodeT> &rawSSkel,
                             SSkel<NodeT> &sskel)
    {
        std::vector<Point_2> poly;
        poly.reserve(pts_poly.size());
        for (const Pnt3 &p : pts_poly)
        {
            poly.emplace_back(p.x(), p.y());
        }
        // SsPtr ss = CGAL::create_interior_straight_skeleton_2(poly.begin(), poly.end());
        SsPtr ss = CGAL::create_interior_straight_skeleton_2(poly.begin(), poly.end());

#ifdef SSKEL_DEBUG
        std::cout << "Straight skeleton with " << ss->size_of_vertices()
                  << " vertices, " << ss->size_of_halfedges()
                  << " halfedges and " << ss->size_of_faces()
                  << " faces" << std::endl;
#endif

        // M_DEBUG << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~start create raw skeleton" << endl;
        cvtToSSkel(ss, rawSSkel);
        // rawSSkel.print();

        // M_DEBUG << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~start collapsing short edges..." << endl;
        rawSSkel.simplify_by_collapse_short_edges(3);
        // rawSSkel.print();
        return ss;
    }
};
