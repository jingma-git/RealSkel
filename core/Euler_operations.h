#pragma once

#include <iostream>
#include <vector>
#include <boost/graph/graph_traits.hpp>

#include "graph_helpers.h"

namespace Euler
{
    /**
* adds a new face defined by a range of vertices (identified by their descriptors,
* `boost::graph_traits<Graph>::%vertex_descriptor`).
* For each pair of consecutive vertices, the corresponding halfedge
* is added in `g` if new, and its connectivity is updated otherwise.
* The face can be added only at the boundary of `g`, or as a new connected component.
*
* @pre `vr` contains at least 3 vertices
* @returns the added face descriptor, or `boost::graph_traits<Graph>::%null_face()` if the face could not be added.
*/
    template <typename Graph, typename VertexRange>
    typename boost::graph_traits<Graph>::face_descriptor
    add_face(const VertexRange &vr, Graph &g)
    {
        using namespace std;

        typedef typename boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;
        typedef typename boost::graph_traits<Graph>::halfedge_descriptor halfedge_descriptor;
        typedef typename boost::graph_traits<Graph>::face_descriptor face_descriptor;
        typedef typename boost::graph_traits<Graph>::edge_descriptor edge_descriptor;

        std::vector<vertex_descriptor> vertices(vr.begin(), vr.end()); // quick and dirty copy
        unsigned int n = (unsigned int)vertices.size();
        //check that every vertex is unique
        std::sort(vertices.begin(), vertices.end());
        if (std::adjacent_find(vertices.begin(), vertices.end()) != vertices.end())
        {
            return boost::graph_traits<Graph>::null_face();
        }
        std::copy(vr.begin(), vr.end(), vertices.begin());
        // don't allow degenerated faces
        if (n <= 2)
        {
            return boost::graph_traits<Graph>::null_face();
        }

        std::vector<halfedge_descriptor> halfedges(n);
        std::vector<bool> is_new(n);

        for (unsigned int i = 0, ii = 1; i < n; ++i, ++ii, ii %= n)
        {
            is_border(vertices[i], g);
            if (!EGL::internal::is_isolated(vertices[i], g) && !is_border(vertices[i], g))
                return boost::graph_traits<Graph>::null_face();

            std::pair<halfedge_descriptor, bool> he = halfedge(vertices[i], vertices[ii], g);
            halfedges[i] = he.first; //collect if exists
            is_new[i] = !(he.second /*true if exists*/);

            if (!is_new[i] && !is_border(halfedges[i], g))
                return boost::graph_traits<Graph>::null_face();
        }

        halfedge_descriptor inner_next, inner_prev,
            outer_next, outer_prev,
            border_next, border_prev,
            patch_start, patch_end;
        // cache for set_next and vertex' set_halfedge
        typedef std::pair<halfedge_descriptor, halfedge_descriptor> NextCacheEntry;
        typedef std::vector<NextCacheEntry> NextCache;
        NextCache next_cache;
        next_cache.reserve(3 * n);

        // re-link patches if necessary
        for (unsigned int i = 0, ii = 1; i < n; ++i, ++ii, ii %= n)
        {
            if (!is_new[i] && !is_new[ii])
            {
                inner_prev = halfedges[i];
                inner_next = halfedges[ii];

                if (next(inner_prev, g) != inner_next)
                {
                    // here comes the ugly part... we have to relink a whole patch

                    // search a free gap
                    // free gap will be between border_prev and border_next
                    outer_prev = opposite(inner_next, g);
                    outer_next = opposite(inner_prev, g);
                    border_prev = outer_prev;
                    do
                    {
                        border_prev = opposite(next(border_prev, g), g);
                    } while (!is_border(border_prev, g) || border_prev == inner_prev);
                    border_next = next(border_prev, g);
                    assert(is_border(border_prev, g));
                    assert(is_border(border_next, g));

                    if (border_next == inner_next)
                        return boost::graph_traits<Graph>::null_face();

                    // other halfedges' indices
                    patch_start = next(inner_prev, g);
                    patch_end = prev(inner_next, g);

                    // relink
                    next_cache.push_back(NextCacheEntry(border_prev, patch_start));
                    next_cache.push_back(NextCacheEntry(patch_end, border_next));
                    next_cache.push_back(NextCacheEntry(inner_prev, inner_next));
                }
            }
        }

        // create missing edges (set hconn_[he].vertex_ for v[i] and v[ii], set hconn_[opposite(he)].face_ to null)
        for (unsigned int i = 0, ii = 1; i < n; ++i, ++ii, ii %= n)
        {
            if (is_new[i])
            {

                edge_descriptor ne = add_edge(vertices[i], vertices[ii], g);
                halfedges[i] = halfedge(ne, g);
                assert(halfedges[i] != boost::graph_traits<Graph>::null_halfedge());

                set_face(opposite(halfedges[i], g), boost::graph_traits<Graph>::null_face(), g); // as it may be recycled we have to reset it
                assert(source(halfedges[i], g) == vertices[i]);
                // cout << __FILE__ << " " << __LINE__ << " create halfedge" << i << " " << halfedges[i] << endl;
            }
        }

        // create the face: set fconn_[f].halfedge_
        face_descriptor f = add_face(g);
        set_halfedge(f, halfedges[n - 1], g);
        // cout << __FILE__ << " " << __LINE__ << " create face " << f << " halfedges=" << halfedge(f, g) << endl;

        // setup halfedges: set vconn_[v].halfedge_, hconn_[he].face_ as newly created face f
        // cout << __FILE__ << " " << __LINE__ << " -----------------------Setup halfedges" << endl;
        for (unsigned int i = 0, ii = 1; i < n; ++i, ++ii, ii %= n)
        {
            vertex_descriptor v = vertices[ii];
            inner_prev = halfedges[i];
            inner_next = halfedges[ii];

            unsigned int id = 0;
            if (is_new[i])
                id |= 1;
            if (is_new[ii])
                id |= 2;

            if (id)
            {
                outer_prev = opposite(inner_next, g);
                outer_next = opposite(inner_prev, g);

                // cout << "id=" << id << " " << v << " inner_prev=" << inner_prev << " inner_next=" << inner_next
                //      << " outer_prev=" << outer_prev << " outer_next=" << outer_next << endl;

                // set outer links
                switch (id)
                {
                case 1: // prev is new, next is old
                    border_prev = prev(inner_next, g);
                    next_cache.push_back(NextCacheEntry(border_prev, outer_next));
                    set_halfedge(v, border_prev, g);
                    break;

                case 2: // next is new, prev is old
                    border_next = next(inner_prev, g);
                    next_cache.push_back(NextCacheEntry(outer_prev, border_next));
                    set_halfedge(v, outer_prev, g);
                    break;

                case 3: // both are new
                {
                    // try to pick a border halfedge with v as target
                    halfedge_descriptor hv = halfedge(v, g);
                    if (hv != boost::graph_traits<Graph>::null_halfedge() && !is_border(hv, g))
                    {
                        for (halfedge_descriptor h_around_v : EGL::halfedges_around_target(hv, g))
                            if (is_border(h_around_v, g))
                            {
                                hv = h_around_v;
                                break;
                            }
                        if (!is_border(hv, g))
                            hv = boost::graph_traits<Graph>::null_halfedge();
                    }

                    if (hv == boost::graph_traits<Graph>::null_halfedge())
                    {
                        set_halfedge(v, outer_prev, g);
                        next_cache.push_back(NextCacheEntry(outer_prev, outer_next));
                    }
                    else
                    {
                        border_prev = hv;
                        border_next = next(border_prev, g);
                        next_cache.push_back(NextCacheEntry(border_prev, outer_next));
                        next_cache.push_back(NextCacheEntry(outer_prev, border_next));
                    }
                    break;
                }
                }

                // set inner link
                next_cache.push_back(NextCacheEntry(inner_prev, inner_next));
            }

            // set face index
            set_face(halfedges[i], f, g);
        }

        // process next halfedge cache, hconn_[he].next_halfedge_, hconn_[he].prev_halfedge_
        typename NextCache::const_iterator ncIt(next_cache.begin()), ncEnd(next_cache.end());
        for (; ncIt != ncEnd; ++ncIt)
            set_next(ncIt->first, ncIt->second, g);

        // adjust vertices' halfedge index
        for (unsigned int i = 0; i < n; ++i)
            EGL::internal::adjust_incoming_halfedge(vertices[i], g);

        return f;
    }
}; // namespace Euler